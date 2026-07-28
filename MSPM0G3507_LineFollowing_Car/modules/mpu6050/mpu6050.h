#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#include "../../shared/module_status.h"

typedef enum {
    MPU6050_STATE_STARTUP = 0,
    MPU6050_STATE_CALIBRATING,
    MPU6050_STATE_READY,
    MPU6050_STATE_DEGRADED
} Mpu6050State;

typedef struct {
    ModuleStatus status;
    float yaw_rate_dps;
    float yaw_angle_deg;
} Mpu6050Snapshot;

void Mpu6050_Init(uint32_t now_ms);
void Mpu6050_Service(uint32_t now_ms);
Mpu6050State Mpu6050_GetState(void);
bool Mpu6050_GetSnapshot(Mpu6050Snapshot *out);

#endif
