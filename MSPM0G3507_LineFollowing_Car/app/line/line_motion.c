#include "line_motion.h"

#include "../../bsp/bsp_i2c.h"
#include "../../config/line_following_profile.h"
#include "../../modules/line_tracking/controller/line_lookup_control.h"
#include "../../modules/line_tracking/recovery/line_recovery.h"
#include "../../modules/line_tracking/decoder/line_position.h"
#include "../../modules/mpu6050/mpu6050.h"
#include "../../modules/mpu6050/mpu6050_config.h"
#include "../../modules/time/timer.h"
#include "../../shared/module_status.h"

static uint32_t imu_started_ms;

static void ClearRequest(uint32_t now_ms, MotionRequest *request)
{
    if (request == 0) {
        return;
    }
    request->left_speed = 0;
    request->right_speed = 0;
    request->timestamp_ms = now_ms;
    request->valid = false;
}

static void PublishImuSnapshot(void)
{
    Mpu6050Snapshot snapshot;

    if (Mpu6050_GetState() == MPU6050_STATE_READY &&
        Mpu6050_GetSnapshot(&snapshot)) {
        AppMailbox_PublishImu(&snapshot);
    }
}

static bool ImuStartupHold(uint32_t now_ms)
{
    Mpu6050State state;

    if (LINE_FOLLOWING_USE_IMU == 0 ||
        (uint32_t)(now_ms - imu_started_ms) >=
            LINE_FOLLOWING_IMU_STARTUP_TIMEOUT_MS) {
        return false;
    }
    state = Mpu6050_GetState();
    return state == MPU6050_STATE_STARTUP ||
           state == MPU6050_STATE_CALIBRATING ||
           state == MPU6050_STATE_DEGRADED;
}

static bool ReadFreshYawRate(uint32_t now_ms, float *yaw_rate_dps)
{
    Mpu6050Snapshot snapshot;

    if (LINE_FOLLOWING_USE_IMU == 0 ||
        !AppMailbox_ReadImu(&snapshot) ||
        !ModuleStatus_IsFresh(&snapshot.status, now_ms, MPU6050_STALE_MS)) {
        return false;
    }
    *yaw_rate_dps = snapshot.yaw_rate_dps;
    return true;
}

static int8_t PositionSign(int8_t position)
{
    if (position < 0) {
        return -1;
    }
    if (position > 0) {
        return 1;
    }
    return 0;
}

void AppLineMotion_Init(uint32_t now_ms)
{
    LinePosition_Reset();
    LineRecovery_Init();
    if (LINE_FOLLOWING_USE_IMU != 0) {
        Mpu6050_Init(now_ms);
    }
    imu_started_ms = now_ms;
}

void AppLineMotion_ServiceImu(uint32_t now_ms)
{
    uint32_t started_us = BSP_Time_GetUs();

    Mpu6050_Service(now_ms);
    while (BSP_I2C_GetStatus() == BSP_I2C_STATUS_BUSY &&
           (uint32_t)(BSP_Time_GetUs() - started_us) < 1000U) {
        BSP_I2C_Service(BSP_Time_GetUs());
    }
    Mpu6050_Service(now_ms);
    PublishImuSnapshot();
}

bool AppLineMotion_BuildRequest(const AppLineSample *sample,
                                uint32_t now_ms,
                                MotionRequest *request)
{
    LineEstimate estimate = {0};
    LineTrendResult trend = {0};
    LineControlOutput follow = {0};
    LineLookupCommand command;
    float yaw_rate_dps = 0.0f;
    bool yaw_fresh;
    bool pattern_known;

    ClearRequest(now_ms, request);
    if (sample == 0 || request == 0 ||
        sample->position.type == LINE_PATTERN_NOISE) {
        return false;
    }
    if (ImuStartupHold(now_ms)) {
        LinePosition_Reset();
        LineRecovery_Reset();
        return false;
    }
    yaw_fresh = ReadFreshYawRate(now_ms, &yaw_rate_dps);

    pattern_known = sample->position.type == LINE_PATTERN_POSITION ||
                    sample->position.type == LINE_PATTERN_WIDE;
    estimate.status.timestamp_ms = sample->timestamp_ms;
    estimate.status.sequence = sample->sequence;
    estimate.status.valid = true;
    estimate.status.health = MODULE_HEALTH_OK;
    estimate.event = sample->position.type == LINE_PATTERN_LOST
                         ? LINE_EVENT_LOST
                         : LINE_EVENT_NONE;
    estimate.error = (float)sample->position.stable_position;
    estimate.predicted_error = (float)sample->position.stable_position;
    estimate.confidence = pattern_known ? 60U : 0U;

    trend.status = estimate.status;
    trend.direction = PositionSign(sample->position.stable_position);

    command = LineLookupControl_Step(sample->position.stable_position,
                                     yaw_rate_dps, yaw_fresh);
    follow.forward = command.base;
    follow.turn = command.diff;
    follow.valid = command.valid;

    return LineRecovery_Step(&estimate, &trend, &follow, yaw_rate_dps,
                             yaw_fresh, false, now_ms, request);
}
