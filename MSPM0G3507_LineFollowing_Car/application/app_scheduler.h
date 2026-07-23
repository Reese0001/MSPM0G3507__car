#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*AppTaskFn)(uint32_t now_ms);

typedef struct {
    uint16_t period_ms;
    uint16_t budget_us;
    uint32_t last_ms;
    uint32_t max_runtime_us;
    uint32_t deadline_miss_count;
    AppTaskFn run;
} AppTask;

typedef struct {
    uint16_t period_ms;
    uint16_t budget_us;
    uint32_t max_runtime_us;
    uint32_t deadline_miss_count;
} AppTaskDiagnostics;

void AppScheduler_Init(uint32_t now_ms);
void AppScheduler_Run(uint32_t now_ms);
bool AppScheduler_GetDiagnostics(uint32_t index, AppTaskDiagnostics *out);

#endif
