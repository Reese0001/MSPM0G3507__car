#include "line_motion.h"

#include "../../config/line_following_profile.h"
#include "../../modules/line_tracking/decoder/line_position.h"
#include "../../modules/line_tracking/recovery/line_recovery.h"
#include "../../shared/module_status.h"

#if LINE_FOLLOWING_CONTROL_MODE == LINE_CONTROL_MODE_OFFICIAL_BASELINE
#include "../../modules/line_tracking/controller/line_official_control.h"
#include "../../modules/optional/ybimu/ybimu.h"
#include "../../modules/optional/ybimu/ybimu_config.h"
#else
#include "../../modules/line_tracking/controller/line_cascade_control.h"
#include "../../modules/line_tracking/controller/line_lookup_control.h"
#include "../../modules/line_tracking/prediction/line_direction_predictor.h"
#include "../../modules/mpu6050/mpu6050.h"
#include "../../modules/mpu6050/mpu6050_config.h"
#endif

static void ClearRequest(uint32_t now_ms, MotionRequest *request)
{
    if (request != 0) {
        request->left_speed = 0;
        request->right_speed = 0;
        request->timestamp_ms = now_ms;
        request->valid = false;
    }
}

#if LINE_FOLLOWING_CONTROL_MODE == LINE_CONTROL_MODE_OFFICIAL_BASELINE

static bool ReadFreshYbImu(uint32_t now_ms, YbImuSnapshot *snapshot)
{
    return LINE_FOLLOWING_USE_YBIMU != 0 &&
           snapshot != 0 &&
           YbImu_GetSnapshot(snapshot) &&
           ModuleStatus_IsFresh(&snapshot->status, now_ms,
                                YBIMU_STALE_TIMEOUT_MS);
}

static bool BuildOfficialBaselineRequest(const AppLineSample *sample,
                                         uint32_t now_ms,
                                         MotionRequest *request)
{
    LineOfficialControlResult control;
    YbImuSnapshot imu = {0};
    bool imu_fresh = ReadFreshYbImu(now_ms, &imu);
    float yaw_rate_dps = imu_fresh ?
                             imu.gyro_rad_s[2] * YBIMU_RAD_TO_DEG :
                             0.0f;

    if (!LineOfficialControl_Step(&sample->position,
                                  sample->sequence,
                                  sample->timestamp_ms,
                                  yaw_rate_dps,
                                  imu_fresh,
                                  &control)) {
        return false;
    }

    /* 找线只服从灰度历史方向；IMU 过期或异常不能阻止电机请求。 */
    return LineRecovery_Step(&control.estimate,
                             control.recovery_direction,
                             &control.follow,
                             0.0f,
                             0.0f,
                             false,
                             false,
                             now_ms,
                             request);
}

#else

static uint32_t imu_started_ms;
static const LineEvent event_by_pattern[] = {
    LINE_EVENT_LOST,
    LINE_EVENT_NONE,
    LINE_EVENT_WIDE_BLACK,
    LINE_EVENT_NONE
};

static void PublishImuSnapshot(void)
{
    Mpu6050Snapshot snapshot;

    if (Mpu6050_GetSnapshot(&snapshot)) {
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

static bool ReadFreshImu(uint32_t now_ms, Mpu6050Snapshot *snapshot)
{
    return LINE_FOLLOWING_USE_IMU != 0 &&
           snapshot != 0 &&
           AppMailbox_ReadImu(snapshot) &&
           ModuleStatus_IsFresh(&snapshot->status, now_ms, MPU6050_STALE_MS);
}

static bool BuildAssistedRequest(const AppLineSample *sample,
                                 uint32_t now_ms,
                                 MotionRequest *request)
{
    LineEstimate estimate = {0};
    LineLookupCommand lookup = {0};
    LineControlOutput follow = {0};
    Mpu6050Snapshot imu = {0};
    int8_t predicted_direction;
    int8_t control_position;
    bool imu_fresh;
    bool pattern_known;

    if (ImuStartupHold(now_ms)) {
        LinePosition_Reset();
        LineDirectionPredictor_Reset();
        LineRecovery_Reset();
        LineCascadeControl_Init(now_ms);
        return false;
    }
    if ((unsigned int)sample->position.type >
            (unsigned int)LINE_PATTERN_NOISE ||
        sample->position.type == LINE_PATTERN_NOISE) {
        LineCascadeControl_Init(now_ms);
        return false;
    }
    imu_fresh = ReadFreshImu(now_ms, &imu);
    pattern_known = sample->position.type == LINE_PATTERN_POSITION ||
                    sample->position.type == LINE_PATTERN_WIDE;
    estimate.status.timestamp_ms = sample->timestamp_ms;
    estimate.status.sequence = sample->sequence;
    estimate.status.valid = true;
    estimate.status.health = MODULE_HEALTH_OK;
    estimate.event = event_by_pattern[sample->position.type];
    estimate.confidence = pattern_known ? 60U : 0U;

    control_position = sample->position.type == LINE_PATTERN_WIDE ?
                           sample->position.candidate_position :
                           sample->position.stable_position;
    estimate.error = (float)control_position;
    estimate.predicted_error = (float)control_position;
    if (sample->position.type == LINE_PATTERN_POSITION ||
        (sample->position.type == LINE_PATTERN_WIDE &&
         control_position != 0)) {
        LineDirectionPredictor_Record(control_position);
    }
    predicted_direction = LineDirectionPredictor_Predict();
    if (pattern_known) {
        lookup = LineLookupControl_Step(control_position);
    }
    if (!LineCascadeControl_Step(&estimate, &lookup,
                                 imu_fresh ? &imu : 0, imu_fresh,
                                 now_ms, &follow)) {
        follow.valid = false;
    }
    return LineRecovery_Step(&estimate, predicted_direction, &follow,
                             imu.yaw_angle_deg, imu.yaw_rate_dps,
                             imu_fresh, false, now_ms, request);
}

#endif

void AppLineMotion_Init(uint32_t now_ms)
{
    LinePosition_Reset();
    LineRecovery_Init();
#if LINE_FOLLOWING_CONTROL_MODE == LINE_CONTROL_MODE_OFFICIAL_BASELINE
    LineOfficialControl_Init();
    if (LINE_FOLLOWING_USE_YBIMU != 0) {
        YbImu_Init(now_ms);
    }
#else
    LineDirectionPredictor_Reset();
    LineCascadeControl_Init(now_ms);
    if (LINE_FOLLOWING_USE_IMU != 0) {
        Mpu6050_Init(now_ms);
    }
    imu_started_ms = now_ms;
#endif
}

void AppLineMotion_ServiceImu(uint32_t now_ms)
{
#if LINE_FOLLOWING_CONTROL_MODE == LINE_CONTROL_MODE_OFFICIAL_BASELINE
    if (LINE_FOLLOWING_USE_YBIMU != 0) {
        YbImu_Service(now_ms);
    }
#else
    if (LINE_FOLLOWING_USE_IMU != 0) {
        Mpu6050_Service(now_ms);
        PublishImuSnapshot();
    }
#endif
}

bool AppLineMotion_BuildRequest(const AppLineSample *sample,
                                uint32_t now_ms,
                                MotionRequest *request)
{
    ClearRequest(now_ms, request);
    if (sample == 0 || request == 0) {
        return false;
    }
#if LINE_FOLLOWING_CONTROL_MODE == LINE_CONTROL_MODE_OFFICIAL_BASELINE
    return BuildOfficialBaselineRequest(sample, now_ms, request);
#else
    return BuildAssistedRequest(sample, now_ms, request);
#endif
}
