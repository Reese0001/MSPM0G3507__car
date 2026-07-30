#ifndef YAW_ESTIMATOR_H
#define YAW_ESTIMATOR_H

#include <stdbool.h>

typedef struct {
    float bias_sum;
    float gyro_bias_rad_s;
    float yaw_angle_deg;
    unsigned int calibration_samples;
    bool calibrated;
} YawEstimator;

void YawEstimator_Init(YawEstimator *estimator);
void YawEstimator_CalibrateSample(YawEstimator *estimator,
                                  float gyro_rad_s,
                                  float dt_s);
void YawEstimator_FinishCalibration(YawEstimator *estimator);
void YawEstimator_Reset(YawEstimator *estimator);
void YawEstimator_Update(YawEstimator *estimator,
                         float gyro_rad_s,
                         bool fresh,
                         float dt_s);

#endif
