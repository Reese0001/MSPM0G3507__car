#ifndef BOOT_TRACE_H
#define BOOT_TRACE_H

#include <stdbool.h>
#include <stdint.h>

#if defined(__GNUC__) || defined(__clang__)
#define BOOT_TRACE_NORETURN __attribute__((noreturn))
#else
#define BOOT_TRACE_NORETURN
#endif

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
#define BOOT_TASK_MOTION_REQUIRED \
    (BOOT_TASK_SAFETY | BOOT_TASK_SENSOR | BOOT_TASK_CONTROL)

void BootTrace_Init(void);
void BootTrace_Mark(BootTraceStage stage);
void BootTrace_TaskOnline(uint8_t bit);
void BootTrace_Tick1ms(void);
bool BootTrace_MotionTasksOnline(void);
bool BootTrace_AllTasksOnline(void);
uint8_t BootTrace_GetTaskMask(void);
BOOT_TRACE_NORETURN void BootTrace_Fatal(BootTraceFault fault);

#endif
