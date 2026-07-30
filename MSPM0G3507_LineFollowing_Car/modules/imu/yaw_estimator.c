#include "yaw_estimator.h"

#define RAD_TO_DEG (57.2957795f)

void YawEstimator_Init(YawEstimator *estimator)
{
    if (estimator != 0) {
        *estimator = (YawEstimator){0};
    }
}

void YawEstimator_CalibrateSample(YawEstimator *estimator,
                                  float gyro_rad_s,
                                  float dt_s)
{
    (void)dt_s;
    if (estimator == 0 || estimator->calibrated) {
        return;
    }
    estimator->bias_sum += gyro_rad_s;
    estimator->calibration_samples++;
}

void YawEstimator_FinishCalibration(YawEstimator *estimator)
{
    if (estimator == 0) {
        return;
    }
    if (estimator->calibration_samples != 0U) {
        estimator->gyro_bias_rad_s =
            estimator->bias_sum / (float)estimator->calibration_samples;
    }
    estimator->calibrated = true;
}

void YawEstimator_Reset(YawEstimator *estimator)
{
    if (estimator != 0) {
        estimator->yaw_angle_deg = 0.0f;
    }
}

void YawEstimator_Update(YawEstimator *estimator,
                         float gyro_rad_s,
                         bool fresh,
                         float dt_s)
{
    float corrected;

    if (estimator == 0 || !fresh || !estimator->calibrated ||
        dt_s <= 0.0f || dt_s > 0.2f) {
        return;
    }
    corrected = gyro_rad_s - estimator->gyro_bias_rad_s;
    estimator->yaw_angle_deg += corrected * dt_s * RAD_TO_DEG;
}
