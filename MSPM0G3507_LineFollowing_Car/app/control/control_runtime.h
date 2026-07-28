#ifndef APP_CONTROL_CONTROL_RUNTIME_H
#define APP_CONTROL_CONTROL_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

void ControlRuntime_Init(void);
bool ControlRuntime_RunOnce(uint32_t now_ms);

#endif
