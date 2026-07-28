#include "mpu6050_kalman.h"

#include "mpu6050_config.h"

void Mpu6050Kalman_Reset(Mpu6050KalmanState *filter)
{
    if (filter == 0) {
        return;
    }
    filter->angle_deg = 0.0f;
    filter->bias_dps = 0.0f;
    filter->p00 = 1.0f;
    filter->p01 = 0.0f;
    filter->p10 = 0.0f;
    filter->p11 = 1.0f;
}

float Mpu6050Kalman_Update(Mpu6050KalmanState *filter,
                           float gyro_rate_dps,
                           float stationary_bias_sample_dps,
                           float dt_s,
                           bool zero_rate_observed)
{
    float rate;
    float innovation;
    float s;
    float k0;
    float k1;

    if (filter == 0 || dt_s <= 0.0f) {
        return 0.0f;
    }

    rate = gyro_rate_dps - filter->bias_dps;
    filter->angle_deg += dt_s * rate;
    filter->p00 += dt_s * (dt_s * filter->p11 - filter->p01 -
                           filter->p10 + MPU6050_KALMAN_Q_ANGLE);
    filter->p01 -= dt_s * filter->p11;
    filter->p10 -= dt_s * filter->p11;
    filter->p11 += MPU6050_KALMAN_Q_BIAS * dt_s;

    if (zero_rate_observed) {
        innovation = stationary_bias_sample_dps - filter->bias_dps;
        s = filter->p11 + MPU6050_KALMAN_R_ZERO_RATE;
        if (s > 0.0f) {
            k0 = filter->p01 / s;
            k1 = filter->p11 / s;
            filter->angle_deg += k0 * innovation;
            filter->bias_dps += k1 * innovation;
            filter->p00 -= k0 * filter->p10;
            filter->p01 -= k0 * filter->p11;
            filter->p10 -= k1 * filter->p10;
            filter->p11 -= k1 * filter->p11;
        }
    }

    return filter->angle_deg;
}

float Mpu6050Kalman_GetAngle(const Mpu6050KalmanState *filter)
{
    return filter == 0 ? 0.0f : filter->angle_deg;
}
