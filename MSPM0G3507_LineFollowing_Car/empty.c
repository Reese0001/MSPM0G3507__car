#include "ti_msp_dl_config.h"

#include "app/boot/app_boot.h"
#include "app/tasks/app_tasks.h"
#include "modules/diagnostics/boot_trace.h"
#include "modules/time/timer.h"

void BootTrace_PortStart(void)
{
    BootTrace_Mark(BOOT_TRACE_PORT_START);
}

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
    AppTasks_Init();
    BootTrace_Mark(BOOT_TRACE_TASKS_CREATED);
    BootTrace_Mark(BOOT_TRACE_SCHED_START);

    for (;;) {
        AppTasks_Poll(Get_Time());
    }
}
