#include "app_tasks.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../boot/app_boot.h"
#include "../mailbox/app_mailbox.h"
#include "../../modules/display/dashboard.h"
#include "../../modules/diagnostics/boot_trace.h"
#include "../../modules/line_tracking/recovery/line_recovery.h"
#include "../safety/safety_supervisor.h"
#include "../../config/line_following_profile.h"
#include "../../config/line_lookup_config.h"
#include "../../config/safety_config.h"
#include "../../bsp/bsp_i2c.h"
#include "../../modules/time/timer.h"
#include "../../modules/led/led.h"
#include "../../modules/line_tracking/controller/line_lookup_control.h"
#include "../../modules/line_tracking/decoder/line_position.h"
#include "../../modules/display/runtime_log.h"
#include "../../modules/display/ssd1306/ssd1306.h"
#include "../../modules/line_tracking/scanner/line_scanner.h"
#include "../../modules/motor/adapter/motor_adapter.h"
#include "../../modules/mpu6050/mpu6050.h"
#include "../../modules/mpu6050/mpu6050_config.h"
#include "../../modules/motor/safety/motor_safety.h"

/* Fixed priorities: fresh sensor data immediately preempts the sensor
 * task through the control notification; safety preempts everything. */
#define APP_TASK_PRIORITY_DISPLAY 1U
#define APP_TASK_PRIORITY_SENSOR  2U
#define APP_TASK_PRIORITY_CONTROL 3U
#define APP_TASK_PRIORITY_SAFETY  4U

#define APP_SENSOR_STACK_WORDS  160U
#define APP_CONTROL_STACK_WORDS 192U
#define APP_SAFETY_STACK_WORDS  160U
#define APP_DISPLAY_STACK_WORDS 160U

/* Service the MPU transaction every fifth 2 ms sensor cycle (10 ms). */
#define APP_IMU_SERVICE_CYCLES 5U

static StaticTask_t sensor_tcb;
static StackType_t sensor_stack[APP_SENSOR_STACK_WORDS];
static StaticTask_t control_tcb;
static StackType_t control_stack[APP_CONTROL_STACK_WORDS];
static StaticTask_t safety_tcb;
static StackType_t safety_stack[APP_SAFETY_STACK_WORDS];
static StaticTask_t display_tcb;
static StackType_t display_stack[APP_DISPLAY_STACK_WORDS];

static TaskHandle_t sensor_task_handle;
static TaskHandle_t control_task_handle;
static TaskHandle_t safety_task_handle;
static TaskHandle_t display_task_handle;

static uint32_t imu_started_ms;
/* Written by their owner tasks, read by SafetyTask for supervision. */
static volatile uint32_t sensor_alive_ms;
static volatile uint32_t control_alive_ms;
/* One-shot task-boundary markers consumed by DisplayTask for field diagnosis. */
static volatile bool safety_loop_seen;
static volatile bool sensor_frame_seen;
static volatile bool control_request_seen;
/* Only SafetyTask writes this; latched codes are never cleared. */
static volatile uint8_t latched_fault = APP_FAULT_NONE;

static void ServiceImu(uint32_t now_ms)
{
    uint32_t started_us = BSP_Time_GetUs();

    Mpu6050_Service(now_ms);
    /* Complete the nonblocking soft-I2C transaction in a bounded burst;
     * SafetyTask and ControlTask can still preempt this loop. */
    while (BSP_I2C_GetStatus() == BSP_I2C_STATUS_BUSY &&
           (uint32_t)(BSP_Time_GetUs() - started_us) < 1000U) {
        BSP_I2C_Service(BSP_Time_GetUs());
    }
    Mpu6050_Service(now_ms);
}

static void PublishImuSnapshot(void)
{
    Mpu6050Snapshot snapshot;

    if (Mpu6050_GetState() == MPU6050_STATE_READY &&
        Mpu6050_GetSnapshot(&snapshot)) {
        AppMailbox_PublishImu(&snapshot);
    }
}

static void SensorTask(void *argument)
{
    TickType_t last_wake;
    uint16_t sequence = 0U;
    uint8_t imu_cycle = 0U;

    BootTrace_TaskOnline(BOOT_TASK_SENSOR);
    last_wake = xTaskGetTickCount();
    (void)argument;
    for (;;) {
        LineSensorSnapshot snapshot;
        uint32_t now_ms;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2U));
        now_ms = Get_Time();
        sensor_alive_ms = now_ms;
        if (LineScanner_ReadFrame(now_ms, &snapshot)) {
            AppLineSample sample;

            sensor_frame_seen = true;
            sample.position = LinePosition_Update(snapshot.black_bits);
            sequence++;
            if (sequence == 0U) {
                sequence = 1U;
            }
            sample.sequence = sequence;
            sample.timestamp_ms = now_ms;
            AppMailbox_PublishLineSample(&sample);
            xTaskNotifyGive(control_task_handle);
        }

        imu_cycle++;
        if (LINE_FOLLOWING_USE_IMU != 0 &&
            imu_cycle >= APP_IMU_SERVICE_CYCLES) {
            imu_cycle = 0U;
            ServiceImu(now_ms);
            PublishImuSnapshot();
        }
    }
}

static bool ImuStartupHold(uint32_t now_ms)
{
    Mpu6050State state;

    if (LINE_FOLLOWING_USE_IMU == 0 ||
        (uint32_t)(now_ms - imu_started_ms) >=
            LINE_FOLLOWING_IMU_STARTUP_TIMEOUT_MS) {
        return false;
    }
    state = Mpu6050_GetState();
    return state == MPU6050_STATE_STARTUP ||
           state == MPU6050_STATE_CALIBRATING ||
           state == MPU6050_STATE_DEGRADED;
}

static bool ReadFreshYawRate(uint32_t now_ms, float *yaw_rate_dps)
{
    Mpu6050Snapshot snapshot;

    if (LINE_FOLLOWING_USE_IMU == 0 ||
        !AppMailbox_ReadImu(&snapshot) ||
        !ModuleStatus_IsFresh(&snapshot.status, now_ms, MPU6050_STALE_MS)) {
        return false;
    }
    *yaw_rate_dps = snapshot.yaw_rate_dps;
    return true;
}

static int8_t PositionSign(int8_t position)
{
    if (position < 0) {
        return -1;
    }
    if (position > 0) {
        return 1;
    }
    return 0;
}

static void BuildMotionRequest(const AppLineSample *sample,
                               uint32_t now_ms,
                               MotionRequest *request)
{
    LineEstimate estimate = {0};
    LineTrendResult trend = {0};
    LineControlOutput follow = {0};
    LineLookupCommand command;
    float yaw_rate_dps = 0.0f;
    bool yaw_fresh;
    bool pattern_known;

    request->left_speed = 0;
    request->right_speed = 0;
    request->timestamp_ms = now_ms;
    request->valid = false;
    if (ImuStartupHold(now_ms)) {
        LinePosition_Reset();
        LineRecovery_Reset();
        return;
    }
    yaw_fresh = ReadFreshYawRate(now_ms, &yaw_rate_dps);

    pattern_known = sample->position.type == LINE_PATTERN_POSITION ||
                    sample->position.type == LINE_PATTERN_WIDE;
    estimate.status.timestamp_ms = sample->timestamp_ms;
    estimate.status.sequence = sample->sequence;
    estimate.status.valid = true;
    estimate.status.health = MODULE_HEALTH_OK;
    estimate.event = sample->position.type == LINE_PATTERN_LOST
                         ? LINE_EVENT_LOST
                         : LINE_EVENT_NONE;
    estimate.error = (float)sample->position.stable_position;
    estimate.predicted_error = (float)sample->position.stable_position;
    estimate.confidence = pattern_known ? 60U : 0U;

    trend.status = estimate.status;
    trend.direction = PositionSign(sample->position.stable_position);

    command = LineLookupControl_Step(sample->position.stable_position,
                                     yaw_rate_dps, yaw_fresh);
    follow.forward = command.base;
    follow.turn = command.diff;
    follow.valid = command.valid;

    (void)LineRecovery_Step(&estimate, &trend, &follow, yaw_rate_dps,
                            yaw_fresh, false, now_ms, request);
}

static void ControlTask(void *argument)
{
    uint16_t last_sequence = 0U;

    BootTrace_TaskOnline(BOOT_TASK_CONTROL);
    (void)argument;
    for (;;) {
        AppLineSample sample;
        MotionRequest request;
        uint32_t now_ms;

        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        now_ms = Get_Time();
        control_alive_ms = now_ms;
        if (!AppMailbox_ReadLineSample(&sample) ||
            sample.sequence == last_sequence) {
            continue;
        }
        last_sequence = sample.sequence;
        BuildMotionRequest(&sample, now_ms, &request);
        AppMailbox_PublishMotionRequest(&request);
        control_request_seen = true;
    }
}

static void SafetyTask(void *argument)
{
    TickType_t last_wake;
    uint32_t last_frame_ms = 0U;
    bool motor_armed = false;

    BootTrace_TaskOnline(BOOT_TASK_SAFETY);
    last_wake = xTaskGetTickCount();
    (void)argument;
    safety_loop_seen = true;
    for (;;) {
        SafetyInputs inputs = {0};
        SafetyDecision decision = {0};
        MotionRequest request = {0};
        uint32_t now_ms;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1U));
        now_ms = Get_Time();

        if (!motor_armed && BootTrace_AllTasksOnline() &&
            AppBoot_IsMotorConfigured() &&
            Motor_Safety_IsFaultLatched() == 0U) {
            Motor_Safety_Arm();
            motor_armed = true;
        }

        /* Latch actionable faults: motor UART first, then silent tasks. */
        if (Motor_Safety_IsFaultLatched() != 0U) {
            latched_fault = APP_FAULT_MOTOR_UART;
        } else if (latched_fault == APP_FAULT_NONE) {
            if ((uint32_t)(now_ms - sensor_alive_ms) >
                APP_SENSOR_HEARTBEAT_TIMEOUT_MS) {
                latched_fault = APP_FAULT_SENSOR_HEARTBEAT;
            } else if ((uint32_t)(now_ms - control_alive_ms) >
                       APP_CONTROL_HEARTBEAT_TIMEOUT_MS) {
                latched_fault = APP_FAULT_CONTROL_HEARTBEAT;
            }
        }

        inputs.ultrasonic_required = LINE_FOLLOWING_USE_ULTRASONIC != 0;
        inputs.imu_required = LINE_FOLLOWING_REQUIRE_IMU != 0;
        inputs.vision_required = LINE_FOLLOWING_USE_VISION != 0;
        /* RESET begins the run; line acquisition must not block arming. */
        inputs.start_pressed = true;
        inputs.reset_pressed = false;
        inputs.power_qualified = (LINE_FOLLOWING_POWER_QUALIFIED != 0) &&
                                 AppBoot_IsMotorConfigured();
        inputs.motor_fault = Motor_Safety_IsFaultLatched() != 0U;

        (void)AppMailbox_ReadMotionRequest(&request);
        if (latched_fault != APP_FAULT_NONE) {
            request.left_speed = 0;
            request.right_speed = 0;
            request.timestamp_ms = now_ms;
            request.valid = false;
            LED_ON();
        }
        (void)SafetySupervisor_Step(&inputs, &request, now_ms, &decision);

        MotorAdapter_Apply(&decision);

        /*
         * RequestSpeed() already sends an immediate zero when a non-zero
         * applied command is rejected.  Keep the regular service frame
         * rate-limited so the highest-priority task cannot monopolize the
         * CPU with repeated blocking UART zero frames.
         */
        if ((uint32_t)(now_ms - last_frame_ms) >= MOTOR_UART_MIN_PERIOD_MS) {
            Motor_Safety_Service();
            last_frame_ms = now_ms;
        }
        if (BootTrace_AllTasksOnline()) {
            LED_HeartbeatService(now_ms);
        }
    }
}

static void DisplayTask(void *argument)
{
    bool display_ready;
    bool observed = false;
    SafetySupervisorState previous_safety = SAFETY_READY;
    LineRecoveryState previous_recovery = LINE_RECOVERY_FOLLOW;
    int16_t previous_left = 0;
    int16_t previous_right = 0;
    MotorSafetyFaultReason previous_fault = MOTOR_SAFETY_FAULT_NONE;
    bool previous_direction_wait = false;
    bool logged_safety_loop = false;
    bool logged_sensor_frame = false;
    bool logged_control_request = false;
    uint8_t previous_task_mask = 0xFFU;

    BootTrace_TaskOnline(BOOT_TASK_DISPLAY);
    display_ready = AppBoot_IsDisplayReady();
    (void)argument;
    for (;;) {
        MotorSafetyDiagnostics motor = {0};
        SafetySupervisorState safety;
        LineRecoveryState recovery;
        uint32_t now_ms;
        uint8_t task_mask;
        bool redraw = false;

        vTaskDelay(pdMS_TO_TICKS(100U));
        now_ms = Get_Time();
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
            continue;
        }

        Motor_Safety_GetDiagnostics(&motor);
        safety = SafetySupervisor_GetState();
        recovery = LineRecovery_GetState();

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
        if ((!observed || motor.direction_wait != previous_direction_wait) &&
            motor.direction_wait) {
            redraw |= RuntimeLog_Push(now_ms, "DIR WAIT");
        }
        if ((!observed || recovery != previous_recovery) &&
            recovery == LINE_RECOVERY_STOPPED) {
            redraw |= RuntimeLog_Push(now_ms, "LINE LOST");
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
    }
}

bool AppTasks_Create(void)
{
    uint32_t now_ms = Get_Time();

    AppMailbox_Init();
    LineScanner_Init();
    LinePosition_Reset();
    LineRecovery_Init();
    SafetySupervisor_Init();
    if (LINE_FOLLOWING_USE_IMU != 0) {
        Mpu6050_Init(now_ms);
    }
    imu_started_ms = now_ms;
    sensor_alive_ms = now_ms;
    control_alive_ms = now_ms;

    sensor_task_handle = xTaskCreateStatic(SensorTask,
                                           "Sensor",
                                           APP_SENSOR_STACK_WORDS,
                                           NULL,
                                           APP_TASK_PRIORITY_SENSOR,
                                           sensor_stack,
                                           &sensor_tcb);
    control_task_handle = xTaskCreateStatic(ControlTask,
                                            "Control",
                                            APP_CONTROL_STACK_WORDS,
                                            NULL,
                                            APP_TASK_PRIORITY_CONTROL,
                                            control_stack,
                                            &control_tcb);
    safety_task_handle = xTaskCreateStatic(SafetyTask,
                                           "Safety",
                                           APP_SAFETY_STACK_WORDS,
                                           NULL,
                                           APP_TASK_PRIORITY_SAFETY,
                                           safety_stack,
                                           &safety_tcb);
    display_task_handle = xTaskCreateStatic(DisplayTask,
                                            "Display",
                                            APP_DISPLAY_STACK_WORDS,
                                            NULL,
                                            APP_TASK_PRIORITY_DISPLAY,
                                            display_stack,
                                            &display_tcb);
    return sensor_task_handle != NULL && control_task_handle != NULL &&
           safety_task_handle != NULL && display_task_handle != NULL;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb,
                                   StackType_t **stack,
                                   uint32_t *depth)
{
    static StaticTask_t idle_tcb;
    static StackType_t idle_stack[configMINIMAL_STACK_SIZE];

    *tcb = &idle_tcb;
    *stack = idle_stack;
    *depth = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t task,
                                   char *task_name)
{
    (void)task;
    (void)task_name;
    BootTrace_Fatal(BOOT_FAULT_STACK_OVERFLOW);
}
