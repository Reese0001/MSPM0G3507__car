#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdint.h>

void AppTasks_Init(void);
void AppTasks_Poll(uint32_t now_ms);

#endif
