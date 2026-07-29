#include "runtime_observer.h"

#include <stdio.h>
#include <string.h>

#include "../mailbox/app_mailbox.h"
#include "../safety/safety_runtime.h"
#include "../safety/safety_supervisor.h"
#include "../../config/line_following_profile.h"
#include "../../modules/diagnostics/boot_trace.h"
#include "../../modules/display/runtime_log.h"
#include "../../modules/display/ssd1306/ssd1306.h"
#include "../../modules/line_tracking/recovery/line_recovery.h"
#include "../../modules/motor/safety/motor_safety.h"
#include "../../shared/module_status.h"

#if LINE_FOLLOWING_CONTROL_MODE == LINE_CONTROL_MODE_OFFICIAL_BASELINE
#include "../../modules/line_tracking/controller/line_official_control.h"
#include "../../modules/optional/ybimu/ybimu.h"
#include "../../modules/optional/ybimu/ybimu_config.h"
#else
#include "../../modules/line_tracking/controller/line_cascade_control.h"
#include "../../modules/mpu6050/mpu6050.h"
#include "../../modules/mpu6050/mpu6050_config.h"
#endif

#define CONTROL_LOG_PERIOD_MS (500U)

static bool display_ready;
static bool observed;
static SafetySupervisorState previous_safety;
static LineRecoveryState previous_recovery;
static int16_t previous_left;
static int16_t previous_right;
static MotorSafetyFaultReason previous_fault;
static uint16_t previous_reason;
static bool previous_armed;
static bool previous_direction_wait;
static bool previous_approved;
static bool previous_request_valid;
static int16_t previous_request_left;
static int16_t previous_request_right;
static bool logged_arm_wait_config;
static bool logged_safety_loop;
static bool logged_sensor_frame;
static bool logged_control_request;
static bool logged_test_run;
static bool logged_sensor_wait;
static bool safety_loop_seen;
static bool sensor_frame_seen;
static bool control_request_seen;
static uint8_t previous_task_mask;
#if LINE_FOLLOWING_CONTROL_MODE != LINE_CONTROL_MODE_OFFICIAL_BASELINE
static Mpu6050State previous_mpu_state;
#endif
static uint32_t last_control_log_ms;

static int clamp_log_value(float value)
{
    if (value > 999.0f) {
        return 999;
    }
    if (value < -999.0f) {
        return -999;
    }
    return (int)(value + (value >= 0.0f ? 0.5f : -0.5f));
}

#if LINE_FOLLOWING_CONTROL_MODE != LINE_CONTROL_MODE_OFFICIAL_BASELINE
static void format_signed_3(char *out, int value)
{
    unsigned int magnitude;

    out[0] = value < 0 ? '-' : '+';
    magnitude = value < 0 ? (unsigned int)-value : (unsigned int)value;
    out[1] = (char)('0' + (magnitude / 100U));
    out[2] = (char)('0' + ((magnitude / 10U) % 10U));
    out[3] = (char)('0' + (magnitude % 10U));
}

static const char *imu_state_label(Mpu6050State state)
{
    switch (state) {
    case MPU6050_STATE_STARTUP:
        return "IMU START";
    case MPU6050_STATE_CALIBRATING:
        return "IMU CAL";
    case MPU6050_STATE_READY:
        return "IMU READY";
    default:
        return "IMU DEG";
    }
}
#endif

static const char *recovery_state_label(LineRecoveryState state)
{
    switch (state) {
    case LINE_RECOVERY_SEEK_LEFT:
        return "LINE SEEK L";
    case LINE_RECOVERY_SEEK_RIGHT:
        return "LINE SEEK R";
    case LINE_RECOVERY_ALIGN:
        return "LINE ALIGN";
    case LINE_RECOVERY_STOPPED:
        return "LINE SAFE STOP";
    default:
        return "LINE FOLLOW";
    }
}

#if LINE_FOLLOWING_CONTROL_MODE == LINE_CONTROL_MODE_OFFICIAL_BASELINE
static bool observe_baseline(uint32_t now_ms,
                             const SafetyRuntimeDiagnostics *runtime,
                             char payload[18])
{
    AppLineSample line = {0};
    LineOfficialControlDiagnostics control = {0};
    YbImuSnapshot imu = {0};
    bool redraw = false;
    bool imu_fresh;
    int position;

    if ((uint32_t)(now_ms - last_control_log_ms) < CONTROL_LOG_PERIOD_MS) {
        return false;
    }
    last_control_log_ms = now_ms;
    LineOfficialControl_GetDiagnostics(&control);

    if (AppMailbox_ReadLineSample(&line)) {
        position = line.position.type == LINE_PATTERN_WIDE ?
                       line.position.candidate_position :
                       line.position.stable_position;
        (void)snprintf(payload, 18U, "B%02X P%+d D%c",
                       (unsigned int)line.position.black_bits,
                       position,
                       control.direction < 0 ? 'L' : 'R');
        redraw |= RuntimeLog_Push(now_ms, payload);
    }

    imu_fresh = YbImu_GetSnapshot(&imu) &&
                ModuleStatus_IsFresh(&imu.status, now_ms,
                                     YBIMU_STALE_TIMEOUT_MS);
    if (imu_fresh) {
        (void)snprintf(payload, 18U, "G%+04d C%+04d I%u",
                       clamp_log_value(control.yaw_rate_dps),
                       (int)control.damping_command,
                       control.imu_used ? 1U : 0U);
        redraw |= RuntimeLog_Push(now_ms, payload);
    } else {
        redraw |= RuntimeLog_Push(now_ms, "IMU BYPASS");
    }

    (void)snprintf(payload, 18U, "CMD %03d/%03d",
                   runtime->last_request.valid ?
                       (int)runtime->last_request.left_speed : 0,
                   runtime->last_request.valid ?
                       (int)runtime->last_request.right_speed : 0);
    return redraw | RuntimeLog_Push(now_ms, payload);
}
#else
static bool observe_imu(uint32_t now_ms,
                        const LineRecoveryDiagnostics *recovery,
                        char payload[18])
{
    Mpu6050Snapshot imu = {0};
    Mpu6050State state = Mpu6050_GetState();
    bool redraw = false;
    bool fresh;

    if (!observed || state != previous_mpu_state) {
        redraw |= RuntimeLog_Push(now_ms, imu_state_label(state));
        previous_mpu_state = state;
    }
    if ((uint32_t)(now_ms - last_control_log_ms) < CONTROL_LOG_PERIOD_MS) {
        return redraw;
    }
    last_control_log_ms = now_ms;
    fresh = AppMailbox_ReadImu(&imu) &&
            ModuleStatus_IsFresh(&imu.status, now_ms, MPU6050_STALE_MS);
    if (!fresh) {
        return redraw | RuntimeLog_Push(now_ms, "IMU STALE");
    }
    {
        static const char imu_template[] = "U Y+000 G+000";

        (void)memcpy(payload, imu_template, sizeof(imu_template));
    }
    payload[0] = LineCascadeControl_IsImuUsed() ? 'U' : 'B';
    if (recovery->state == LINE_RECOVERY_SEEK_LEFT ||
        recovery->state == LINE_RECOVERY_SEEK_RIGHT) {
        payload[2] = 'D';
        format_signed_3(&payload[3],
                        clamp_log_value(recovery->yaw_delta_deg));
    } else {
        format_signed_3(&payload[3], clamp_log_value(imu.yaw_angle_deg));
    }
    format_signed_3(&payload[9], clamp_log_value(imu.yaw_rate_dps));
    return redraw | RuntimeLog_Push(now_ms, payload);
}
#endif

void RuntimeObserver_Init(bool ready)
{
    display_ready = ready;
    observed = false;
    previous_safety = SAFETY_READY;
    previous_recovery = LINE_RECOVERY_FOLLOW;
    previous_left = 0;
    previous_right = 0;
    previous_fault = MOTOR_SAFETY_FAULT_NONE;
    previous_reason = SAFETY_REASON_NONE;
    previous_armed = false;
    previous_direction_wait = false;
    previous_approved = false;
    previous_request_valid = false;
    previous_request_left = 0;
    previous_request_right = 0;
    logged_arm_wait_config = false;
    logged_safety_loop = false;
    logged_sensor_frame = false;
    logged_control_request = false;
    logged_test_run = false;
    logged_sensor_wait = false;
    safety_loop_seen = false;
    sensor_frame_seen = false;
    control_request_seen = false;
    previous_task_mask = 0xFFU;
#if LINE_FOLLOWING_CONTROL_MODE != LINE_CONTROL_MODE_OFFICIAL_BASELINE
    previous_mpu_state = MPU6050_STATE_DEGRADED;
#endif
    last_control_log_ms = 0U;
}

void RuntimeObserver_MarkSafetyLoop(void)
{
    safety_loop_seen = true;
}

void RuntimeObserver_MarkSensorFrame(void)
{
    sensor_frame_seen = true;
}

void RuntimeObserver_MarkControlRequest(void)
{
    control_request_seen = true;
}

bool RuntimeObserver_Update(uint32_t now_ms)
{
    MotorSafetyDiagnostics motor = {0};
    SafetyRuntimeDiagnostics runtime = {0};
    LineRecoveryDiagnostics recovery_diagnostics = {0};
    SafetySupervisorState safety;
    LineRecoveryState recovery;
    uint8_t task_mask;
    bool redraw = false;
    char payload[18];

    task_mask = BootTrace_GetTaskMask();
    if (task_mask != previous_task_mask) {
        redraw |= RuntimeLog_PushTaskMask(now_ms, task_mask);
        previous_task_mask = task_mask;
    }
    if (!display_ready) {
        display_ready = Ssd1306_Init();
        if (display_ready) {
            (void)RuntimeLog_Push(now_ms, "OLED OK");
            RuntimeLog_Draw();
            display_ready = Ssd1306_FlushDirty();
            if (!display_ready) {
                (void)RuntimeLog_Push(now_ms, "OLED FAIL");
            }
        }
        return display_ready;
    }

    Motor_Safety_GetDiagnostics(&motor);
    SafetyRuntime_GetDiagnostics(&runtime);
    safety = SafetySupervisor_GetState();
    LineRecovery_GetDiagnostics(&recovery_diagnostics);
    recovery = recovery_diagnostics.state;
#if LINE_FOLLOWING_CONTROL_MODE == LINE_CONTROL_MODE_OFFICIAL_BASELINE
    redraw |= observe_baseline(now_ms, &runtime, payload);
#else
    redraw |= observe_imu(now_ms, &recovery_diagnostics, payload);
#endif

    if (!logged_arm_wait_config && runtime.arm_waiting_for_config) {
        redraw |= RuntimeLog_Push(now_ms, "ARM WAIT CFG");
        logged_arm_wait_config = true;
    }
    if (!logged_safety_loop && safety_loop_seen) {
        redraw |= RuntimeLog_Push(now_ms, "SAFETY TASK");
        logged_safety_loop = true;
    }
    if (!logged_sensor_frame && sensor_frame_seen) {
        redraw |= RuntimeLog_Push(now_ms, "SENSOR FRAME");
        logged_sensor_frame = true;
    }
    if (!logged_control_request && control_request_seen) {
        redraw |= RuntimeLog_Push(now_ms, "CONTROL REQ");
        logged_control_request = true;
    }
    if (!logged_sensor_wait &&
        SafetyRuntime_IsSensorHeartbeatMissing()) {
        redraw |= RuntimeLog_Push(now_ms, "SENSOR WAIT");
        logged_sensor_wait = true;
    }
    if (runtime.last_request.valid &&
        (!previous_request_valid ||
         runtime.last_request.left_speed != previous_request_left ||
         runtime.last_request.right_speed != previous_request_right)) {
        (void)snprintf(payload, sizeof(payload), "%s%03d R%03d", "REQ L",
                       (int)runtime.last_request.left_speed,
                       (int)runtime.last_request.right_speed);
        redraw |= RuntimeLog_Push(now_ms, payload);
    }
    if ((!observed || safety != previous_safety) &&
        safety == SAFETY_RUNNING) {
        redraw |= RuntimeLog_Push(now_ms, "SAFETY RUN");
    }
    if (runtime.last_decision.approved && !previous_approved) {
        redraw |= RuntimeLog_Push(now_ms, "APPROVED");
    } else if (!runtime.last_decision.approved &&
               runtime.last_decision.reason != previous_reason) {
        if (runtime.last_decision.reason == SAFETY_REASON_REQUEST_INVALID) {
            redraw |= RuntimeLog_Push(now_ms, "REJECT REQ");
        } else if (runtime.last_decision.reason == SAFETY_REASON_MOTOR_FAULT) {
            redraw |= RuntimeLog_Push(now_ms, "REJECT MOTOR");
        } else if (runtime.last_decision.reason == SAFETY_REASON_POWER) {
            redraw |= RuntimeLog_Push(now_ms, "REJECT POWER");
        }
    }
    if ((!observed || motor.armed != previous_armed) &&
        motor.armed) {
        redraw |= RuntimeLog_Push(now_ms, "MOTOR ARMED");
    }
    if (!observed || motor.fault_reason != previous_fault) {
        if (motor.fault_reason == MOTOR_SAFETY_FAULT_UART_TIMEOUT) {
            redraw |= RuntimeLog_Push(now_ms, "UART TIMEOUT");
        } else if (motor.fault_reason == MOTOR_SAFETY_FAULT_WATCHDOG) {
            redraw |= RuntimeLog_Push(now_ms, "WATCHDOG");
        }
    }
    if ((!observed ||
         motor.direction_wait != previous_direction_wait) &&
        motor.direction_wait) {
        redraw |= RuntimeLog_Push(now_ms, "DIR WAIT");
    }
    if (!observed || recovery != previous_recovery) {
        redraw |= RuntimeLog_Push(now_ms, recovery_state_label(recovery));
    }
    if (!logged_test_run &&
        (motor.left_applied != 0 || motor.right_applied != 0)) {
        redraw |= RuntimeLog_Push(now_ms, "TEST RUN");
        logged_test_run = true;
    }
    if (observed &&
        (motor.left_applied != previous_left ||
         motor.right_applied != previous_right)) {
        redraw |= RuntimeLog_PushMotor(now_ms, motor.left_applied,
                                       motor.right_applied);
    }

    previous_safety = safety;
    previous_recovery = recovery;
    previous_left = motor.left_applied;
    previous_right = motor.right_applied;
    previous_fault = motor.fault_reason;
    previous_reason = runtime.last_decision.reason;
    previous_armed = motor.armed;
    previous_direction_wait = motor.direction_wait;
    previous_approved = runtime.last_decision.approved;
    previous_request_valid = runtime.last_request.valid;
    previous_request_left = runtime.last_request.left_speed;
    previous_request_right = runtime.last_request.right_speed;
    observed = true;
    if (redraw) {
        RuntimeLog_Draw();
    }
    if (display_ready) {
        display_ready = Ssd1306_FlushNextDirtyPage();
        if (!display_ready) {
            (void)RuntimeLog_Push(now_ms, "OLED FAIL");
        }
    }
    return display_ready;
}
