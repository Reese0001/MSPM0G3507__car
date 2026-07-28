#include "runtime_observer.h"

#include "../safety/safety_supervisor.h"
#include "../../modules/diagnostics/boot_trace.h"
#include "../../modules/display/runtime_log.h"
#include "../../modules/display/ssd1306/ssd1306.h"
#include "../../modules/line_tracking/recovery/line_recovery.h"
#include "../../modules/motor/safety/motor_safety.h"

static bool display_ready;
static bool observed;
static SafetySupervisorState previous_safety;
static LineRecoveryState previous_recovery;
static int16_t previous_left;
static int16_t previous_right;
static MotorSafetyFaultReason previous_fault;
static bool previous_direction_wait;
static bool logged_safety_loop;
static bool logged_sensor_frame;
static bool logged_control_request;
static bool logged_test_run;
static uint8_t previous_task_mask;

void RuntimeObserver_Init(bool ready)
{
    display_ready = ready;
    observed = false;
    previous_safety = SAFETY_READY;
    previous_recovery = LINE_RECOVERY_FOLLOW;
    previous_left = 0;
    previous_right = 0;
    previous_fault = MOTOR_SAFETY_FAULT_NONE;
    previous_direction_wait = false;
    logged_safety_loop = false;
    logged_sensor_frame = false;
    logged_control_request = false;
    logged_test_run = false;
    previous_task_mask = 0xFFU;
}

bool RuntimeObserver_Update(uint32_t now_ms,
                            const RuntimeObserverInputs *inputs)
{
    MotorSafetyDiagnostics motor = {0};
    SafetySupervisorState safety;
    LineRecoveryState recovery;
    uint8_t task_mask;
    bool redraw = false;

    if (inputs == 0) {
        return display_ready;
    }
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
    safety = SafetySupervisor_GetState();
    recovery = LineRecovery_GetState();

    if (!logged_safety_loop && inputs->safety_loop_seen) {
        redraw |= RuntimeLog_Push(now_ms, "SAFETY TASK");
        logged_safety_loop = true;
    }
    if (!logged_sensor_frame && inputs->sensor_frame_seen) {
        redraw |= RuntimeLog_Push(now_ms, "SENSOR FRAME");
        logged_sensor_frame = true;
    }
    if (!logged_control_request && inputs->control_request_seen) {
        redraw |= RuntimeLog_Push(now_ms, "CONTROL REQ");
        logged_control_request = true;
    }
    if ((!observed || safety != previous_safety) &&
        safety == SAFETY_RUNNING) {
        redraw |= RuntimeLog_Push(now_ms, "MOTOR ARM");
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
    if ((!observed || recovery != previous_recovery) &&
        recovery == LINE_RECOVERY_STOPPED) {
        redraw |= RuntimeLog_Push(now_ms, "LINE LOST");
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
    previous_direction_wait = motor.direction_wait;
    observed = true;
    if (redraw) {
        RuntimeLog_Draw();
        display_ready = Ssd1306_FlushDirty();
        if (!display_ready) {
            (void)RuntimeLog_Push(now_ms, "OLED FAIL");
        }
    }
    return display_ready;
}
