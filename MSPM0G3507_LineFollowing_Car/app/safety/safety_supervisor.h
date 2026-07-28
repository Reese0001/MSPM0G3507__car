#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#include "../../shared/module_status.h"
#include "../../shared/motion_request.h"
#include "../../shared/safety_decision.h"

typedef struct {
    ModuleStatus status;
    uint16_t distance_mm;
} SafetySensorInput;

typedef struct {
    SafetySensorInput ultrasonic;
    ModuleStatus imu;
    ModuleStatus vision;
    bool ultrasonic_required;
    bool vision_required;
    bool imu_required;
    bool start_pressed;
    bool reset_pressed;
    bool power_qualified;
    bool motor_fault;
} SafetyInputs;

void SafetySupervisor_Init(void);
void SafetySupervisor_Reinitialize(void);
SafetySupervisorState SafetySupervisor_GetState(void);
bool SafetySupervisor_Step(const SafetyInputs *inputs,
                           const MotionRequest *mission_request,
                           uint32_t now_ms,
                           SafetyDecision *decision);

#endif
