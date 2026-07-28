#ifndef APP_SAFETY_SAFETY_RUNTIME_H
#define APP_SAFETY_SAFETY_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

void SafetyRuntime_Init(uint32_t now_ms);
void SafetyRuntime_OnSensorFrame(uint32_t now_ms);
void SafetyRuntime_Step(uint32_t now_ms);
bool SafetyRuntime_IsSensorHeartbeatMissing(void);

#endif
