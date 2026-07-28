#include "app_tasks.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../boot/app_boot.h"
#include "../line/line_motion.h"
#include "../log/runtime_observer.h"
#include "../mailbox/app_mailbox.h"
#include "../run/run_controller.h"
#include "../../modules/diagnostics/boot_trace.h"
#include "../safety/safety_supervisor.h"
#include "../../config/line_following_profile.h"
#include "../../config/safety_config.h"
#include "../../modules/time/timer.h"
#include "../../modules/led/led.h"
#include "../../modules/line_tracking/decoder/line_position.h"
#include "../../modules/key/key.h"
#include "../../modules/line_tracking/scanner/line_scanner.h"
#include "../../modules/motor/adapter/motor_adapter.h"
#include "../../modules/motor/safety/motor_safety.h"

#define APP_TASK_PRIORITY_DISPLAY 1U
#define APP_TASK_PRIORITY_SENSOR  2U
#define APP_TASK_PRIORITY_CONTROL 3U
#define APP_TASK_PRIORITY_SAFETY  4U
#define APP_SENSOR_STACK_WORDS  160U
#define APP_CONTROL_STACK_WORDS 192U
#define APP_SAFETY_STACK_WORDS  160U
#define APP_DISPLAY_STACK_WORDS 160U
#define APP_IMU_SERVICE_CYCLES 5U

static StaticTask_t sensor_tcb, control_tcb, safety_tcb, display_tcb;
static StackType_t sensor_stack[APP_SENSOR_STACK_WORDS];
static StackType_t control_stack[APP_CONTROL_STACK_WORDS];
static StackType_t safety_stack[APP_SAFETY_STACK_WORDS];
static StackType_t display_stack[APP_DISPLAY_STACK_WORDS];
static TaskHandle_t sensor_task_handle, control_task_handle;
static TaskHandle_t safety_task_handle, display_task_handle;
static volatile uint32_t sensor_alive_ms;
static volatile bool safety_loop_seen, sensor_frame_seen, control_request_seen;
static volatile uint8_t latched_fault = APP_FAULT_NONE;

static TaskHandle_t CreateTask(TaskFunction_t entry, const char *name,
                               uint32_t words, UBaseType_t priority,
                               StackType_t *stack, StaticTask_t *tcb)
{
    return xTaskCreateStatic(entry, name, words, NULL, priority, stack, tcb);
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
            AppLineMotion_ServiceImu(now_ms);
        }
    }
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
        if (!AppMailbox_ReadLineSample(&sample) ||
            sample.sequence == last_sequence) {
            continue;
        }
        last_sequence = sample.sequence;
        if (AppLineMotion_BuildRequest(&sample, now_ms, &request)) {
            AppMailbox_PublishMotionRequest(&request);
            control_request_seen = true;
        }
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
        MotionRequest line_request = {0};
        uint32_t now_ms;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1U));
        now_ms = Get_Time();
        RunController_OnKeyEvent(Key_PollEvent());

        if (!motor_armed && BootTrace_AllTasksOnline() &&
            AppBoot_IsMotorConfigured() &&
            Motor_Safety_IsFaultLatched() == 0U) {
            Motor_Safety_Arm();
            motor_armed = true;
        }

        if (Motor_Safety_IsFaultLatched() != 0U) {
            latched_fault = APP_FAULT_MOTOR_UART;
        } else if (latched_fault == APP_FAULT_NONE) {
            if ((uint32_t)(now_ms - sensor_alive_ms) >
                APP_SENSOR_HEARTBEAT_TIMEOUT_MS) {
                latched_fault = APP_FAULT_SENSOR_HEARTBEAT;
            }
        }

        inputs.ultrasonic_required = LINE_FOLLOWING_USE_ULTRASONIC != 0;
        inputs.imu_required = LINE_FOLLOWING_REQUIRE_IMU != 0;
        inputs.vision_required = LINE_FOLLOWING_USE_VISION != 0;
        inputs.start_pressed = true;
        inputs.reset_pressed = false;
        inputs.power_qualified = (LINE_FOLLOWING_POWER_QUALIFIED != 0) &&
                                 AppBoot_IsMotorConfigured();
        inputs.motor_fault = Motor_Safety_IsFaultLatched() != 0U;

        (void)RunController_BuildRequest(now_ms, &request);
        if (AppMailbox_ReadMotionRequest(&line_request) &&
            line_request.valid &&
            (uint32_t)(now_ms - line_request.timestamp_ms) <=
                MOTION_REQUEST_MAX_AGE_MS) {
            request = line_request;
        }
        if (latched_fault != APP_FAULT_NONE) {
            request.left_speed = 0;
            request.right_speed = 0;
            request.timestamp_ms = now_ms;
            request.valid = false;
            LED_ON();
        }
        (void)SafetySupervisor_Step(&inputs, &request, now_ms, &decision);

        MotorAdapter_Apply(&decision);

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
    BootTrace_TaskOnline(BOOT_TASK_DISPLAY);
    (void)argument;
    for (;;) {
        RuntimeObserverInputs inputs = {0};
        uint32_t now_ms;

        vTaskDelay(pdMS_TO_TICKS(100U));
        now_ms = Get_Time();
        inputs.safety_loop_seen = safety_loop_seen;
        inputs.sensor_frame_seen = sensor_frame_seen;
        inputs.control_request_seen = control_request_seen;
        (void)RuntimeObserver_Update(now_ms, &inputs);
    }
}

bool AppTasks_Create(void)
{
    uint32_t now_ms = Get_Time();

    AppMailbox_Init();
    LineScanner_Init();
    AppLineMotion_Init(now_ms);
    SafetySupervisor_Init();
    RunController_Init();
    RuntimeObserver_Init(AppBoot_IsDisplayReady());
    sensor_alive_ms = now_ms;

    sensor_task_handle = CreateTask(SensorTask, "Sensor",
        APP_SENSOR_STACK_WORDS, APP_TASK_PRIORITY_SENSOR, sensor_stack,
        &sensor_tcb);
    control_task_handle = CreateTask(ControlTask, "Control",
        APP_CONTROL_STACK_WORDS, APP_TASK_PRIORITY_CONTROL, control_stack,
        &control_tcb);
    safety_task_handle = CreateTask(SafetyTask, "Safety",
        APP_SAFETY_STACK_WORDS, APP_TASK_PRIORITY_SAFETY, safety_stack,
        &safety_tcb);
    display_task_handle = CreateTask(DisplayTask, "Display",
        APP_DISPLAY_STACK_WORDS, APP_TASK_PRIORITY_DISPLAY, display_stack,
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
