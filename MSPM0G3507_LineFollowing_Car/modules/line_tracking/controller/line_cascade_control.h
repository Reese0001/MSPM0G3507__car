#ifndef LINE_CASCADE_CONTROL_H
#define LINE_CASCADE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "../../../shared/module_status.h"
#include "../../../shared/motion_request.h"
#include "../line_model.h"
#include "line_lookup_control.h"
#include "../../mpu6050/mpu6050.h"

void LineCascadeControl_Init(uint32_t now_ms);
bool LineCascadeControl_IsImuUsed(void);
bool LineCascadeControl_Step(const LineEstimate *estimate,
                             const LineLookupCommand *feedforward,
                             const Mpu6050Snapshot *imu,
                             bool imu_fresh,
                             uint32_t now_ms,
                             LineControlOutput *output);

#endif
