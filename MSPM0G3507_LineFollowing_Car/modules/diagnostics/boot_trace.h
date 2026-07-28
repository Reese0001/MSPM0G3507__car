#ifndef BOOT_TRACE_H
#define BOOT_TRACE_H

#include <stdbool.h>
#include <stdint.h>

/* Temporary startup checkpoints.  Remove after the scheduler root cause is fixed. */
typedef enum {
    BOOT_TRACE_MAIN = 1,
    BOOT_TRACE_TASKS_CREATED = 2,
    BOOT_TRACE_SCHED_START = 3,
    BOOT_TRACE_PORT_START = 4,
    BOOT_TRACE_SVC = 5,
    BOOT_TRACE_FIRST_RESTORE = 6,
    BOOT_TRACE_SAFETY_TASK = 10,
    BOOT_TRACE_SENSOR_TASK = 11,
    BOOT_TRACE_CONTROL_TASK = 12,
    BOOT_TRACE_DISPLAY_TASK = 13,
    BOOT_TRACE_ALL_TASKS = 14
} BootTraceStage;

typedef enum {
    BOOT_FAULT_ASSERT = 1,
    BOOT_FAULT_HARDFAULT = 2,
    BOOT_FAULT_DEFAULT_IRQ = 3,
    BOOT_FAULT_STACK_OVERFLOW = 4,
    BOOT_FAULT_SCHED_RETURN = 5
} BootTraceFault;

#define BOOT_TASK_SAFETY  (1U << 0)
#define BOOT_TASK_SENSOR  (1U << 1)
#define BOOT_TASK_CONTROL (1U << 2)
#define BOOT_TASK_DISPLAY (1U << 3)
#define BOOT_TASK_ALL     (0x0FU)

void BootTrace_Init(void);
void BootTrace_Mark(BootTraceStage stage);
void BootTrace_TaskOnline(uint8_t bit);
bool BootTrace_AllTasksOnline(void);
uint8_t BootTrace_GetTaskMask(void);
__attribute__((noreturn)) void BootTrace_Fatal(BootTraceFault fault);

#endif
