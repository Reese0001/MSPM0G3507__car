#include "ti_msp_dl_config.h"
#include "app/boot/app_boot.h"
#include "app/tasks/app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "modules/motor/safety/motor_safety.h"
#include "modules/display/runtime_log.h"
#include "modules/display/ssd1306/ssd1306.h"
#include "modules/time/timer.h"

static void BootLog(const char *event)
{
    (void)RuntimeLog_Push(Get_Time(), event);
    RuntimeLog_Draw();
    (void)Ssd1306_FlushDirty();
}

int main(void)
{
    SYSCFG_DL_init();
    AppBoot_Init();
    BootLog("CREATE TASK");

    if (!AppTasks_Create()) {
        BootLog("TASK FAIL");
        Motor_Safety_Disarm();
        for (;;) {}
    }
    BootLog("TASKS OK");

    vTaskStartScheduler();
    Motor_Safety_Disarm();
    for (;;) {}
}
