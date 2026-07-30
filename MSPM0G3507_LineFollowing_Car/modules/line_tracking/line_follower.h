#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdbool.h>
#include <stdint.h>

#include "../../shared/app_line_sample.h"
#include "../../shared/motion_request.h"
#include "../imu/mpu6050.h"

typedef enum {
    LINE_FOLLOWER_FOLLOW = 0,
    LINE_FOLLOWER_SEEK_LEFT,
    LINE_FOLLOWER_SEEK_RIGHT
} LineFollowerMode;

typedef enum {
    LINE_IMU_OFF = 0,
    LINE_IMU_OK,
    LINE_IMU_USED
} LineImuState;

typedef struct {
    LineFollowerMode mode;
    LineImuState imu_state;
    int8_t direction;
    int8_t position;
    uint8_t black_bits;
    float yaw_rate_dps;
    float error;
    float predicted_error;
    float trend;
    float yaw_angle_deg;
    float heading_error;
    bool heading_hold;
    int16_t turn_command;
    int16_t imu_correction;
} LineFollowerStatus;

void LineFollower_Init(void);
bool LineFollower_Step(const AppLineSample *line,
                       const Mpu6050Snapshot *imu,
                       bool imu_fresh,
                       uint32_t now_ms,
                       MotionRequest *request,
                       LineFollowerStatus *status);

#endif
