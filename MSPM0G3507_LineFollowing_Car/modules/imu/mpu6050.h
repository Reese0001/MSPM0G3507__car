#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#include "../../shared/module_status.h"

#define MPU6050_SAMPLE_PERIOD_MS (10U)
#define MPU6050_STALE_TIMEOUT_MS (50U)
/* If OLED yaw damping opposes the expected direction, change only to -1.0f. */
#define MPU6050_GYRO_Z_SIGN (1.0f)
#define MPU6050_GYRO_Y_SIGN (1.0f)

typedef struct {
    ModuleStatus status;
    float gyro_rad_s[3];
    float yaw_angle_deg;
} Mpu6050Snapshot;

void Mpu6050_Init(uint32_t now_ms);
void Mpu6050_Service(uint32_t now_ms);
bool Mpu6050_GetSnapshot(Mpu6050Snapshot *out);
void Mpu6050_ResetYawReference(void);

#endif
