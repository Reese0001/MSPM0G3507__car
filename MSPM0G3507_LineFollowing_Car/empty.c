#include "ti_msp_dl_config.h"
#include "application/app_main.h"
#include "application/freertos/app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor_safety.h"

int main(void)
{
    SYSCFG_DL_init();
    App_Main_Init();

    if (!AppTasks_Create()) {
        Motor_Safety_Disarm();
        for (;;) {}
    }

    vTaskStartScheduler();
    Motor_Safety_Disarm();
    for (;;) {}
}
