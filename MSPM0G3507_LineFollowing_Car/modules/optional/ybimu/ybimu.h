#ifndef YBIMU_H
#define YBIMU_H

#include <stdbool.h>
#include <stdint.h>

#include "../../../shared/module_status.h"

typedef struct {
    ModuleStatus status;
    float gyro_rad_s[3];
    float mag_uT[3];
    float quat[4];
    float euler_deg[3];
    bool magnetic_heading_healthy;
} YbImuSnapshot;

typedef enum {
    YBIMU_CAL_TYPE_IMU = 0,
    YBIMU_CAL_TYPE_MAG
} YbImuCalibrationType;

typedef enum {
    YBIMU_CAL_IDLE = 0,
    YBIMU_CAL_RUNNING,
    YBIMU_CAL_SUCCESS,
    YBIMU_CAL_FAILED
} YbImuCalibrationState;

void YbImu_Init(uint32_t now_ms);
void YbImu_Service(uint32_t now_ms);
bool YbImu_GetSnapshot(YbImuSnapshot *out);
bool YbImu_RequestCalibration(YbImuCalibrationType type, uint32_t now_ms);
void YbImu_CancelCalibration(void);
YbImuCalibrationState YbImu_GetCalibrationState(void);

#endif
