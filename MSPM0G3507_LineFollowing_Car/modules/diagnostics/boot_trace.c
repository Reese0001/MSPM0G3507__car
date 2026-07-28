#include "boot_trace.h"

#include "ti_msp_dl_config.h"

static volatile BootTraceStage stage;
static volatile uint8_t task_mask;

static void BootTrace_SetStagePins(BootTraceStage value)
{
    switch (value) {
    case BOOT_TRACE_SCHED_START:
        DL_GPIO_clearPins(LED_PORT, LED_D1_PIN | LED_D2_PIN);
        break;
    case BOOT_TRACE_PORT_START:
        DL_GPIO_clearPins(LED_PORT, LED_D1_PIN);
        DL_GPIO_setPins(LED_PORT, LED_D2_PIN);
        break;
    case BOOT_TRACE_SVC:
        DL_GPIO_setPins(LED_PORT, LED_D1_PIN);
        DL_GPIO_clearPins(LED_PORT, LED_D2_PIN);
        break;
    case BOOT_TRACE_FIRST_RESTORE:
        DL_GPIO_setPins(LED_PORT, LED_D1_PIN | LED_D2_PIN);
        break;
    case BOOT_TRACE_ALL_TASKS:
        DL_GPIO_clearPins(LED_PORT, LED_D1_PIN | LED_D2_PIN);
        break;
    default:
        break;
    }
}

static void BootTrace_Delay(void)
{
    volatile uint32_t count;

    for (count = 0U; count < 800000U; ++count) {
    }
}

void BootTrace_Init(void)
{
    stage = 0;
    task_mask = 0U;
    DL_GPIO_clearPins(LED_PORT, LED_D1_PIN | LED_D2_PIN);
}

void BootTrace_Mark(BootTraceStage value)
{
    stage = value;
    BootTrace_SetStagePins(value);
}

void BootTrace_TaskOnline(uint8_t bit)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t previous_mask;

    __disable_irq();
    previous_mask = task_mask;
    task_mask = (uint8_t)(task_mask | (bit & BOOT_TASK_ALL));
    if (previous_mask != BOOT_TASK_ALL && task_mask == BOOT_TASK_ALL) {
        stage = BOOT_TRACE_ALL_TASKS;
        BootTrace_SetStagePins(BOOT_TRACE_ALL_TASKS);
    }
    __set_PRIMASK(primask);
}

bool BootTrace_AllTasksOnline(void)
{
    return task_mask == BOOT_TASK_ALL;
}

uint8_t BootTrace_GetTaskMask(void)
{
    return task_mask;
}

__attribute__((noreturn)) void BootTrace_Fatal(BootTraceFault fault)
{
    uint8_t pulse;

    DL_GPIO_setPins(LED_PORT, LED_D1_PIN);
    for (;;) {
        for (pulse = 0U; pulse < (uint8_t)fault; ++pulse) {
            DL_GPIO_setPins(LED_PORT, LED_D2_PIN);
            BootTrace_Delay();
            DL_GPIO_clearPins(LED_PORT, LED_D2_PIN);
            BootTrace_Delay();
        }
        BootTrace_Delay();
    }
}
