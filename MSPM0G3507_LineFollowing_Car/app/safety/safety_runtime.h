#ifndef APP_SAFETY_SAFETY_RUNTIME_H
#define APP_SAFETY_SAFETY_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#include "../../shared/motion_request.h"
#include "../../shared/safety_decision.h"

typedef struct {
    MotionRequest last_request;
    SafetyDecision last_decision;
    bool motor_armed;
    bool arm_waiting_for_config;
    bool arm_blocked_by_fault;
    bool sensor_heartbeat_missing;
} SafetyRuntimeDiagnostics;

void SafetyRuntime_Init(uint32_t now_ms);
void SafetyRuntime_OnSensorFrame(uint32_t now_ms);
void SafetyRuntime_Step(uint32_t now_ms);
bool SafetyRuntime_IsSensorHeartbeatMissing(void);
void SafetyRuntime_GetDiagnostics(SafetyRuntimeDiagnostics *out);

#endif
