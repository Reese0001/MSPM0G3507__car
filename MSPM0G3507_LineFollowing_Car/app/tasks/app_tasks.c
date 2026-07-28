#include "app_tasks.h"

#include <stdbool.h>

#include "../boot/app_boot.h"
#include "../control/control_runtime.h"
#include "../line/line_motion.h"
#include "../log/runtime_observer.h"
#include "../mailbox/app_mailbox.h"
#include "../safety/safety_runtime.h"
#include "../sensor/sensor_runtime.h"
#include "../../modules/diagnostics/boot_trace.h"
#include "../../modules/time/timer.h"

#define APP_SAFETY_PERIOD_MS (1U)
#define APP_SENSOR_PERIOD_MS (2U)
#define APP_DISPLAY_PERIOD_MS (100U)

static uint32_t last_safety_ms;
static uint32_t last_sensor_ms;
static uint32_t last_display_ms;
static bool sensor_frame_pending;

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

    last_safety_ms = now_ms;
    last_sensor_ms = now_ms;
    last_display_ms = now_ms;
    sensor_frame_pending = false;
}

void AppTasks_Poll(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - last_safety_ms) >= APP_SAFETY_PERIOD_MS) {
        SafetyRuntime_Step(now_ms);
        last_safety_ms = now_ms;
    }

    if ((uint32_t)(now_ms - last_sensor_ms) >= APP_SENSOR_PERIOD_MS) {
        if (SensorRuntime_Step(now_ms)) {
            SafetyRuntime_OnSensorFrame(now_ms);
            RuntimeObserver_MarkSensorFrame();
            sensor_frame_pending = true;
        }
        last_sensor_ms = now_ms;
    }

    if (sensor_frame_pending) {
        sensor_frame_pending = false;
        if (ControlRuntime_RunOnce(now_ms)) {
            RuntimeObserver_MarkControlRequest();
        }
    }

    if ((uint32_t)(now_ms - last_display_ms) >= APP_DISPLAY_PERIOD_MS) {
        (void)RuntimeObserver_Update(now_ms);
        last_display_ms = now_ms;
    }
}
