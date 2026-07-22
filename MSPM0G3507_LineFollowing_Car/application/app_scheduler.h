#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include <stdint.h>

typedef void (*AppTaskFn)(uint32_t now_ms);

typedef struct {
    uint16_t period_ms;
    uint32_t last_ms;
    AppTaskFn run;
} AppTask;

void AppScheduler_Init(uint32_t now_ms);
void AppScheduler_Run(uint32_t now_ms);

#endif
