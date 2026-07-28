#ifndef MPU6050_KALMAN_H
#define MPU6050_KALMAN_H

#include <stdbool.h>

typedef struct {
    float angle_deg;
    float bias_dps;
    float p00;
    float p01;
    float p10;
    float p11;
} Mpu6050KalmanState;

void Mpu6050Kalman_Reset(Mpu6050KalmanState *filter);
float Mpu6050Kalman_Update(Mpu6050KalmanState *filter,
                           float gyro_rate_dps,
                           float stationary_bias_sample_dps,
                           float dt_s,
                           bool zero_rate_observed);
float Mpu6050Kalman_GetAngle(const Mpu6050KalmanState *filter);

#endif
