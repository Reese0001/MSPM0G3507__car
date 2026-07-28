#include "boot_trace.h"

#include "ti_msp_dl_config.h"

#define BOOT_TRACE_PULSE_MS (100U)
#define BOOT_TRACE_GROUP_GAP_MS (700U)
#define BOOT_FATAL_GROUP_GAP_DELAYS (5U)

typedef enum {
    BOOT_PULSE_IDLE = 0,
    BOOT_PULSE_ON,
    BOOT_PULSE_OFF,
    BOOT_PULSE_GAP
} BootTracePulseState;

static volatile BootTraceStage stage;
static volatile uint8_t task_mask;
static uint16_t pulse_elapsed_ms;
static uint8_t pulse_mask;
static uint8_t pulse_target;
static uint8_t pulse_count;
static BootTracePulseState pulse_state;

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

static uint8_t BootTrace_Popcount(uint8_t mask)
{
    uint8_t count = 0U;

    if ((mask & BOOT_TASK_SAFETY) != 0U) {
        ++count;
    }
    if ((mask & BOOT_TASK_SENSOR) != 0U) {
        ++count;
    }
    if ((mask & BOOT_TASK_CONTROL) != 0U) {
        ++count;
    }
    if ((mask & BOOT_TASK_DISPLAY) != 0U) {
        ++count;
    }
    return count;
}

void BootTrace_Init(void)
{
    stage = 0;
    task_mask = 0U;
    pulse_elapsed_ms = 0U;
    pulse_mask = 0U;
    pulse_target = 0U;
    pulse_count = 0U;
    pulse_state = BOOT_PULSE_IDLE;
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

void BootTrace_Tick1ms(void)
{
    uint8_t mask = task_mask;

    if (mask == 0U || mask == BOOT_TASK_ALL) {
        return;
    }
    DL_GPIO_clearPins(LED_PORT, LED_D1_PIN);
    if (mask != pulse_mask) {
        pulse_mask = mask;
        pulse_target = BootTrace_Popcount(mask);
        pulse_count = 1U;
        pulse_elapsed_ms = 0U;
        pulse_state = BOOT_PULSE_ON;
        DL_GPIO_setPins(LED_PORT, LED_D2_PIN);
        return;
    }

    ++pulse_elapsed_ms;
    if (pulse_state == BOOT_PULSE_GAP) {
        if (pulse_elapsed_ms < BOOT_TRACE_GROUP_GAP_MS) {
            return;
        }
        pulse_count = 1U;
        pulse_elapsed_ms = 0U;
        pulse_state = BOOT_PULSE_ON;
        DL_GPIO_setPins(LED_PORT, LED_D2_PIN);
        return;
    }
    if (pulse_elapsed_ms < BOOT_TRACE_PULSE_MS) {
        return;
    }

    pulse_elapsed_ms = 0U;
    if (pulse_state == BOOT_PULSE_ON) {
        DL_GPIO_clearPins(LED_PORT, LED_D2_PIN);
        pulse_state = (pulse_count < pulse_target) ? BOOT_PULSE_OFF
                                                   : BOOT_PULSE_GAP;
    } else {
        ++pulse_count;
        pulse_state = BOOT_PULSE_ON;
        DL_GPIO_setPins(LED_PORT, LED_D2_PIN);
    }
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
    uint8_t gap;
    uint8_t pulse;

    __disable_irq();
    DL_GPIO_clearPins(LED_PORT, LED_D2_PIN);
    DL_GPIO_setPins(LED_PORT, LED_D1_PIN);
    for (;;) {
        for (pulse = 0U; pulse < (uint8_t)fault; ++pulse) {
            DL_GPIO_setPins(LED_PORT, LED_D2_PIN);
            BootTrace_Delay();
            DL_GPIO_clearPins(LED_PORT, LED_D2_PIN);
            BootTrace_Delay();
        }
        for (gap = 0U; gap < BOOT_FATAL_GROUP_GAP_DELAYS; ++gap) {
            BootTrace_Delay();
        }
    }
}
