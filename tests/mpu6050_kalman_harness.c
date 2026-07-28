#include <assert.h>

#include "modules/mpu6050/mpu6050_kalman.h"

static void assert_close(float value, float target, float tolerance)
{
    float error = value - target;

    if (error < 0.0f) {
        error = -error;
    }
    assert(error <= tolerance);
}

int main(void)
{
    Mpu6050KalmanState filter;
    float angle;

    Mpu6050Kalman_Reset(&filter);
    assert_close(Mpu6050Kalman_GetAngle(&filter), 0.0f, 0.001f);

    for (unsigned int i = 0U; i < 100U; i++) {
        angle = Mpu6050Kalman_Update(
            &filter, 90.0f, 90.0f, 0.01f, false);
    }
    assert(angle > 60.0f);
    assert(angle < 95.0f);

    for (unsigned int i = 0U; i < 200U; i++) {
        angle = Mpu6050Kalman_Update(
            &filter, 0.0f, 0.0f, 0.01f, true);
    }
    assert(angle > 60.0f);
    assert(angle < 95.0f);

    Mpu6050Kalman_Reset(&filter);
    for (unsigned int i = 0U; i < 500U; i++) {
        angle = Mpu6050Kalman_Update(
            &filter, 0.8f, 0.8f, 0.01f, true);
    }
    assert(filter.bias_dps > 0.5f);
    assert(angle > -2.0f);
    assert(angle < 2.0f);

    Mpu6050Kalman_Reset(&filter);
    for (unsigned int i = 0U; i < 100U; i++) {
        (void)Mpu6050Kalman_Update(
            &filter, 90.0f, 90.0f, 0.01f, false);
    }
    (void)Mpu6050Kalman_Update(
        &filter, 67.5f, 0.0f, 0.01f, true);
    assert(filter.bias_dps > -1.0f);
    assert(filter.bias_dps < 1.0f);

    Mpu6050Kalman_Reset(&filter);
    assert_close(Mpu6050Kalman_GetAngle(&filter), 0.0f, 0.001f);
    return 0;
}
