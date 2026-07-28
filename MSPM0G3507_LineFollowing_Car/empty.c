#include "ti_msp_dl_config.h"
#include "app/boot/app_boot.h"
#include "app/tasks/app_tasks.h"
#include "modules/display/runtime_log.h"
#include "modules/display/ssd1306/ssd1306.h"
#include "modules/diagnostics/boot_trace.h"
#include "modules/time/timer.h"

static void BootLog(const char *event)
{
    (void)RuntimeLog_Push(Get_Time(), event);
    RuntimeLog_Draw();
    (void)Ssd1306_FlushDirty();
}

void BootTrace_PortStart(void)
{
    BootTrace_Mark(BOOT_TRACE_PORT_START);
}

/*
 * 主循环启动前如果进入 HardFault，OLED 不一定还能工作。
 * 这里直接点亮 D1/D2，给现场一个不依赖 I2C 的异常证据。
 */
void HardFault_Handler(void)
{
    BootTrace_Fatal(BOOT_FAULT_HARDFAULT);
}

void Default_Handler(void)
{
    BootTrace_Fatal(BOOT_FAULT_DEFAULT_IRQ);
}

int main(void)
{
    SYSCFG_DL_init();
    BootTrace_Init();
    BootTrace_Mark(BOOT_TRACE_MAIN);
    AppBoot_Init();
    BootLog("INIT TASKS");
    AppTasks_Init();
    BootTrace_Mark(BOOT_TRACE_TASKS_CREATED);
    BootLog("TASKS OK");
    BootLog("LOOP START");
    BootTrace_Mark(BOOT_TRACE_SCHED_START);

    for (;;) {
        AppTasks_Poll(Get_Time());
    }
}
