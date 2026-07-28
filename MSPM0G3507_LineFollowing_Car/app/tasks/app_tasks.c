#include "app_tasks.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../boot/app_boot.h"
#include "../control/control_runtime.h"
#include "../line/line_motion.h"
#include "../log/runtime_observer.h"
#include "../mailbox/app_mailbox.h"
#include "../safety/safety_runtime.h"
#include "../sensor/sensor_runtime.h"
#include "../../modules/diagnostics/boot_trace.h"
#include "../../modules/time/timer.h"

#define APP_TASK_PRIORITY_DISPLAY 1U
#define APP_TASK_PRIORITY_SENSOR  2U
#define APP_TASK_PRIORITY_CONTROL 3U
#define APP_TASK_PRIORITY_SAFETY  4U
#define APP_SENSOR_STACK_WORDS  160U
#define APP_CONTROL_STACK_WORDS 192U
#define APP_SAFETY_STACK_WORDS  160U
#define APP_DISPLAY_STACK_WORDS 160U

static StaticTask_t sensor_tcb, control_tcb, safety_tcb, display_tcb;
static StackType_t sensor_stack[APP_SENSOR_STACK_WORDS];
static StackType_t control_stack[APP_CONTROL_STACK_WORDS];
static StackType_t safety_stack[APP_SAFETY_STACK_WORDS];
static StackType_t display_stack[APP_DISPLAY_STACK_WORDS];
static TaskHandle_t sensor_task_handle, control_task_handle;
static TaskHandle_t safety_task_handle, display_task_handle;

static TaskHandle_t CreateTask(TaskFunction_t entry, const char *name,
                               uint32_t words, UBaseType_t priority,
                               StackType_t *stack, StaticTask_t *tcb)
{
    return xTaskCreateStatic(entry, name, words, NULL, priority, stack, tcb);
}

static void SensorTask(void *argument)
{
    TickType_t last_wake;

    BootTrace_TaskOnline(BOOT_TASK_SENSOR);
    last_wake = xTaskGetTickCount();
    (void)argument;
    for (;;) {
        uint32_t now_ms;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2U));
        now_ms = Get_Time();
        if (SensorRuntime_Step(now_ms)) {
            SafetyRuntime_OnSensorFrame(now_ms);
            RuntimeObserver_MarkSensorFrame();
            xTaskNotifyGive(control_task_handle);
        }
    }
}

static void ControlTask(void *argument)
{
    BootTrace_TaskOnline(BOOT_TASK_CONTROL);
    (void)argument;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ControlRuntime_RunOnce(Get_Time())) {
            RuntimeObserver_MarkControlRequest();
        }
    }
}

static void SafetyTask(void *argument)
{
    TickType_t last_wake;

    BootTrace_TaskOnline(BOOT_TASK_SAFETY);
    RuntimeObserver_MarkSafetyLoop();
    last_wake = xTaskGetTickCount();
    (void)argument;
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1U));
        SafetyRuntime_Step(Get_Time());
    }
}

static void DisplayTask(void *argument)
{
    BootTrace_TaskOnline(BOOT_TASK_DISPLAY);
    (void)argument;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100U));
        (void)RuntimeObserver_Update(Get_Time());
    }
}

bool AppTasks_Create(void)
{
    uint32_t now_ms = Get_Time();

    AppMailbox_Init();
    AppLineMotion_Init(now_ms);
    SensorRuntime_Init();
    ControlRuntime_Init();
    SafetyRuntime_Init(now_ms);
    RuntimeObserver_Init(AppBoot_IsDisplayReady());

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
