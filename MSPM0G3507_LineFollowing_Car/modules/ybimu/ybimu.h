#ifndef YBIMU_H
#define YBIMU_H

#include <stdbool.h>
#include <stdint.h>

#include "module_status.h"

typedef struct {
    ModuleStatus status;
    float gyro_rad_s[3];
    float mag_uT[3];
    float quat[4];
    float euler_deg[3];
    bool magnetic_heading_healthy;
} YbImuSnapshot;

void YbImu_Init(uint32_t now_ms);
void YbImu_Service(uint32_t now_ms);
bool YbImu_GetSnapshot(YbImuSnapshot *out);

#endif
