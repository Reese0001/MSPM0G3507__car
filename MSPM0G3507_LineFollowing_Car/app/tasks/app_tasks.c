#include "app_tasks.h"

#include <stdbool.h>

#include "../boot/app_boot.h"
#include "../control/control_runtime.h"
#include "../line/line_motion.h"
#include "../log/runtime_observer.h"
#include "../mailbox/app_mailbox.h"
#include "../safety/safety_runtime.h"
#include "../sensor/sensor_runtime.h"
#include "../../bsp/bsp_i2c.h"
#include "../../modules/diagnostics/boot_trace.h"
#include "../../modules/time/timer.h"

#define APP_TASK_BASE_TICK_MS (1U)

typedef enum {
    APP_TASK_SAFETY = 0,
    APP_TASK_SENSOR,
    APP_TASK_CONTROL,
    APP_TASK_DISPLAY
} AppTaskId;

typedef void (*AppTaskFunction)(uint32_t now_ms);

typedef struct {
    AppTaskId id;
    AppTaskFunction run;
    uint32_t period_ms;
    uint32_t last_run_ms;
} AppTaskSlot;

static bool sensor_frame_pending;

static void safety_task(uint32_t now_ms)
{
    SafetyRuntime_Step(now_ms);
}

static void sensor_task(uint32_t now_ms)
{
    if (SensorRuntime_Step(now_ms)) {
        SafetyRuntime_OnSensorFrame(now_ms);
        RuntimeObserver_MarkSensorFrame();
        sensor_frame_pending = true;
    }
}

static void control_task(uint32_t now_ms)
{
    if (!sensor_frame_pending) {
        return;
    }
    sensor_frame_pending = false;
    if (ControlRuntime_RunOnce(now_ms)) {
        RuntimeObserver_MarkControlRequest();
    }
}

static void display_task(uint32_t now_ms)
{
    (void)RuntimeObserver_Update(now_ms);
}

static AppTaskSlot app_task_slots[] = {
    {APP_TASK_SAFETY, safety_task, APP_TASK_BASE_TICK_MS, 0U},
    {APP_TASK_SENSOR, sensor_task, 2U * APP_TASK_BASE_TICK_MS, 0U},
    {APP_TASK_CONTROL, control_task, APP_TASK_BASE_TICK_MS, 0U},
    {APP_TASK_DISPLAY, display_task, 100U * APP_TASK_BASE_TICK_MS, 0U}
};

static bool task_is_due(const AppTaskSlot *slot, uint32_t now_ms)
{
    return slot != 0 &&
           (uint32_t)(now_ms - slot->last_run_ms) >= slot->period_ms;
}

static void run_task_slot(const AppTaskSlot *slot, uint32_t now_ms)
{
    if (slot != 0 && slot->run != 0) {
        slot->run(now_ms);
    }
}

static void init_task_slots(uint32_t now_ms)
{
    uint8_t index;

    for (index = 0U;
         index < (uint8_t)(sizeof(app_task_slots) / sizeof(app_task_slots[0]));
         index++) {
        app_task_slots[index].last_run_ms = now_ms;
    }
}

void AppTasks_Init(void)
{
    uint32_t now_ms = Get_Time();

    AppMailbox_Init();
    AppLineMotion_Init(now_ms);
    SensorRuntime_Init();
    ControlRuntime_Init();
    SafetyRuntime_Init(now_ms);
    RuntimeObserver_Init(AppBoot_IsDisplayReady());

    BootTrace_TaskOnline(BOOT_TASK_SAFETY);
    BootTrace_TaskOnline(BOOT_TASK_SENSOR);
    BootTrace_TaskOnline(BOOT_TASK_CONTROL);
    BootTrace_TaskOnline(BOOT_TASK_DISPLAY);
    RuntimeObserver_MarkSafetyLoop();

    init_task_slots(now_ms);
    sensor_frame_pending = false;
}

void AppTasks_Poll(uint32_t now_ms)
{
    uint8_t index;

    BSP_I2C_Service(BSP_Time_GetUs());
    for (index = 0U;
        index < (uint8_t)(sizeof(app_task_slots) / sizeof(app_task_slots[0]));
         index++) {
        if (task_is_due(&app_task_slots[index], now_ms)) {
            app_task_slots[index].last_run_ms = now_ms;
            run_task_slot(&app_task_slots[index], now_ms);
        }
    }
}
