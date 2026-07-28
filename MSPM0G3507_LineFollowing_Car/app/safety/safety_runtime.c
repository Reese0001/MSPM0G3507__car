#include "safety_runtime.h"

#include <stdbool.h>

#include "../boot/app_boot.h"
#include "../mailbox/app_mailbox.h"
#include "../run/run_controller.h"
#include "safety_supervisor.h"
#include "../../config/line_following_profile.h"
#include "../../config/line_lookup_config.h"
#include "../../config/safety_config.h"
#include "../../modules/diagnostics/boot_trace.h"
#include "../../modules/display/dashboard.h"
#include "../../modules/key/key.h"
#include "../../modules/led/led.h"
#include "../../modules/motor/adapter/motor_adapter.h"
#include "../../modules/motor/safety/motor_safety.h"

static uint32_t sensor_alive_ms;
static uint32_t last_frame_ms;
static bool motor_armed;
static bool arm_waiting_for_config;
static bool arm_blocked_by_fault;
static bool sensor_heartbeat_missing;
static uint8_t latched_fault;
static MotionRequest last_request;
static SafetyDecision last_decision;

static void StopRequest(uint32_t now_ms, MotionRequest *request)
{
    request->left_speed = 0;
    request->right_speed = 0;
    request->timestamp_ms = now_ms;
    request->valid = false;
}

static void UpdateFaultLatch(uint32_t now_ms)
{
    sensor_heartbeat_missing =
        (uint32_t)(now_ms - sensor_alive_ms) >
        APP_SENSOR_HEARTBEAT_TIMEOUT_MS;

    if (Motor_Safety_IsFaultLatched() != 0U) {
        latched_fault = APP_FAULT_MOTOR_UART;
    }
}

static bool MotionRequestHasOutput(const MotionRequest *request)
{
    return request != 0 &&
           (request->left_speed != 0 || request->right_speed != 0);
}

static bool LineRequestCanOverride(const MotionRequest *request,
                                   bool motion_output_seen)
{
    return motion_output_seen || MotionRequestHasOutput(request);
}

static SafetyInputs BuildInputs(void)
{
    SafetyInputs inputs = {0};

    inputs.ultrasonic_required = LINE_FOLLOWING_USE_ULTRASONIC != 0;
    inputs.imu_required = LINE_FOLLOWING_REQUIRE_IMU != 0;
    inputs.vision_required = LINE_FOLLOWING_USE_VISION != 0;
    inputs.start_pressed = true;
    inputs.reset_pressed = false;
    inputs.power_qualified = (LINE_FOLLOWING_POWER_QUALIFIED != 0) &&
                             AppBoot_IsMotorConfigured();
    inputs.motor_fault = Motor_Safety_IsFaultLatched() != 0U;
    return inputs;
}

static MotionRequest BuildRequest(uint32_t now_ms)
{
    MotionRequest request = {0};
    MotionRequest line_request = {0};
    MotorSafetyDiagnostics motor = {0};
    bool motion_output_seen;

    Motor_Safety_GetDiagnostics(&motor);
    motion_output_seen = motor.left_applied != 0 || motor.right_applied != 0;
    (void)RunController_BuildRequest(now_ms, &request);
    if (AppMailbox_ReadMotionRequest(&line_request) &&
        line_request.valid &&
        LineRequestCanOverride(&line_request, motion_output_seen) &&
        (uint32_t)(now_ms - line_request.timestamp_ms) <=
            MOTION_REQUEST_MAX_AGE_MS) {
        request = line_request;
    }
    if (latched_fault != APP_FAULT_NONE) {
        StopRequest(now_ms, &request);
        LED_ON();
    }
    return request;
}

static void ArmWhenReady(void)
{
    bool configured = AppBoot_IsMotorConfigured();
    bool faulted = Motor_Safety_IsFaultLatched() != 0U;

    arm_waiting_for_config = !motor_armed && !configured;
    arm_blocked_by_fault = !motor_armed && faulted;
    if (!motor_armed && configured && !faulted) {
        Motor_Safety_Arm();
        motor_armed = true;
        arm_waiting_for_config = false;
        arm_blocked_by_fault = false;
    }
}

void SafetyRuntime_Init(uint32_t now_ms)
{
    SafetySupervisor_Init();
    RunController_Init();
    sensor_alive_ms = now_ms;
    last_frame_ms = 0U;
    motor_armed = false;
    arm_waiting_for_config = false;
    arm_blocked_by_fault = false;
    sensor_heartbeat_missing = false;
    latched_fault = APP_FAULT_NONE;
    StopRequest(now_ms, &last_request);
    last_decision.approved = false;
    last_decision.left_speed = 0;
    last_decision.right_speed = 0;
    last_decision.reason = SAFETY_REASON_BOOT_GATE;
    last_decision.state = SAFETY_BOOT_SAFE;
}

void SafetyRuntime_OnSensorFrame(uint32_t now_ms)
{
    sensor_alive_ms = now_ms;
    sensor_heartbeat_missing = false;
}

void SafetyRuntime_Step(uint32_t now_ms)
{
    SafetyInputs inputs;
    SafetyDecision decision = {0};
    MotionRequest request;

    RunController_OnKeyEvent(Key_PollEvent());
    ArmWhenReady();
    UpdateFaultLatch(now_ms);
    inputs = BuildInputs();
    request = BuildRequest(now_ms);

    (void)SafetySupervisor_Step(&inputs, &request, now_ms, &decision);
    last_request = request;
    last_decision = decision;
    MotorAdapter_Apply(&decision);

    if ((uint32_t)(now_ms - last_frame_ms) >= MOTOR_UART_MIN_PERIOD_MS) {
        Motor_Safety_Service();
        last_frame_ms = now_ms;
    }
    if (BootTrace_AllTasksOnline()) {
        LED_HeartbeatService(now_ms);
    }
}

bool SafetyRuntime_IsSensorHeartbeatMissing(void)
{
    return sensor_heartbeat_missing;
}

void SafetyRuntime_GetDiagnostics(SafetyRuntimeDiagnostics *out)
{
    if (out == 0) {
        return;
    }
    out->last_request = last_request;
    out->last_decision = last_decision;
    out->motor_armed = motor_armed;
    out->arm_waiting_for_config = arm_waiting_for_config;
    out->arm_blocked_by_fault = arm_blocked_by_fault;
    out->sensor_heartbeat_missing = sensor_heartbeat_missing;
}
