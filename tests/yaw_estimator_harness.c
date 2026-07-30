#include <assert.h>
#include <math.h>

#include "modules/imu/yaw_estimator.h"

int main(void)
{
    YawEstimator estimator;

    YawEstimator_Init(&estimator);
    for (int i = 0; i < 100; i++) {
        YawEstimator_CalibrateSample(&estimator, 0.1f, 0.01f);
    }
    YawEstimator_FinishCalibration(&estimator);
    YawEstimator_Reset(&estimator);
    YawEstimator_Update(&estimator, 0.1f, true, 0.1f);
    assert(fabsf(estimator.yaw_angle_deg) < 0.01f);
    for (int i = 0; i < 10; i++) {
        YawEstimator_Update(&estimator, 0.2f, true, 0.1f);
    }
    assert(estimator.yaw_angle_deg > 5.0f);
    return 0;
}
