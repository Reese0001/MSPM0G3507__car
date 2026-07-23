#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#include "../modules/common/motion_request.h"
#include "../modules/common/safety_decision.h"
#include "../modules/k230_link/k230_link.h"
#include "../modules/ultrasonic/ultrasonic.h"
#include "../modules/ybimu/ybimu.h"

typedef struct {
    UltrasonicSnapshot ultrasonic;
    YbImuSnapshot imu;
    K230VisionSnapshot vision;
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
