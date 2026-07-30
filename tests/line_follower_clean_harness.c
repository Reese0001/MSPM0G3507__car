#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "modules/line_tracking/line_follower.h"

static AppLineSample sample(uint8_t bits, uint16_t sequence, uint32_t now_ms)
{
    AppLineSample value;

    value.position = LinePosition_Update(bits);
    value.sequence = sequence;
    value.timestamp_ms = now_ms;
    return value;
}

int main(void)
{
    MotionRequest request;
    MotionRequest held;
    LineFollowerStatus status;
    Mpu6050Snapshot imu = {0};
    AppLineSample line;

    LinePosition_Reset();
    LineFollower_Init();

    line = sample(0x06U, 1U, 2U);
    assert(LineFollower_Step(&line, &imu, false, 2U, &request, &status));
    assert(request.valid && request.left_speed == 400 &&
           request.right_speed == 400);
    assert(status.imu_state == LINE_IMU_OFF);

    /* Bit 0 is the left outer sensor (X2). */
    line = sample(0x01U, 2U, 4U);
    assert(LineFollower_Step(&line, &imu, false, 4U, &request, &status));
    line = sample(0x01U, 3U, 6U);
    assert(LineFollower_Step(&line, &imu, false, 6U, &request, &status));
    line = sample(0x01U, 4U, 8U);
    assert(LineFollower_Step(&line, &imu, false, 8U, &request, &status));
    line = sample(0x01U, 5U, 10U);
    assert(LineFollower_Step(&line, &imu, false, 10U, &request, &status));
    assert(request.left_speed > request.right_speed);

    LinePosition_Reset();
    LineFollower_Init();

    /* Bit 3 is the right outer sensor (X4). */
    line = sample(0x08U, 2U, 4U);
    assert(LineFollower_Step(&line, &imu, false, 4U, &request, &status));
    line = sample(0x08U, 3U, 6U);
    assert(LineFollower_Step(&line, &imu, false, 6U, &request, &status));
    assert(request.left_speed < request.right_speed);
    assert(status.direction > 0);

    line = sample(0x00U, 4U, 8U);
    assert(LineFollower_Step(&line, &imu, false, 8U, &request, &status));
    assert(status.mode == LINE_FOLLOWER_SEEK_RIGHT);
    assert(request.left_speed == 0 && request.right_speed == 100);

    line = sample(0x06U, 5U, 10U);
    assert(LineFollower_Step(&line, &imu, false, 10U, &request, &status));
    assert(status.mode == LINE_FOLLOWER_SEEK_RIGHT);
    line = sample(0x06U, 6U, 12U);
    assert(LineFollower_Step(&line, &imu, false, 12U, &request, &status));
    line = sample(0x06U, 7U, 14U);
    assert(LineFollower_Step(&line, &imu, false, 14U, &request, &status));
    assert(status.mode == LINE_FOLLOWER_FOLLOW);

    imu.gyro_rad_s[1] = 10.0f / 57.2957795f;
    line = sample(0x06U, 8U, 16U);
    assert(LineFollower_Step(&line, &imu, true, 16U, &request, &status));
    assert(status.imu_state == LINE_IMU_USED);
    assert(status.imu_correction == -2); /* First-order gyro filter. */
    assert(status.yaw_rate_dps > 0.0f);

    imu.gyro_rad_s[1] = 1.0f / 57.2957795f;
    for (uint16_t sequence = 9U; sequence < 17U; sequence++) {
        line = sample(0x06U, sequence,
                      (uint32_t)(18U + 2U * (sequence - 9U)));
        assert(LineFollower_Step(&line, &imu, true, line.timestamp_ms,
                                 &request, &status));
    }
    assert(status.imu_state == LINE_IMU_OK);
    assert(status.imu_correction == 0);

    /* A stable centered line enables the straight-segment heading hold. */
    LinePosition_Reset();
    LineFollower_Init();
    imu.yaw_angle_deg = 0.0f;
    for (uint16_t sequence = 30U; sequence < 75U; sequence++) {
        line = sample(0x06U, sequence,
                      (uint32_t)(100U + 2U * (sequence - 30U)));
        (void)LineFollower_Step(&line, &imu, true, line.timestamp_ms,
                                &request, &status);
    }
    assert(status.heading_hold);
    imu.yaw_angle_deg = 10.0f;
    line = sample(0x06U, 75U, 190U);
    (void)LineFollower_Step(&line, &imu, true, 190U, &request, &status);
    assert(status.heading_error < 0.0f);

    LinePosition_Reset();
    LineFollower_Init();
    line = sample(0x07U, 10U, 20U); /* Three left sensors: negative side. */
    assert(LineFollower_Step(&line, &imu, false, 20U, &request, &status));
    assert(status.position < 0 && status.direction < 0);
    assert(request.left_speed > request.right_speed);

    line = sample(0x0EU, 11U, 22U); /* Three right sensors: positive side. */
    assert(LineFollower_Step(&line, &imu, false, 22U, &request, &status));
    assert(status.position > 0 && status.direction > 0);
    assert(request.left_speed < request.right_speed);

    /* A monotonic left-to-right sweep produces a positive trend and
     * predicts the next error instead of waiting for a single-frame jump. */
    LinePosition_Reset();
    LineFollower_Init();
    line = sample(0x01U, 20U, 40U);
    (void)LineFollower_Step(&line, &imu, false, 40U, &request, &status);
    line = sample(0x03U, 21U, 42U);
    (void)LineFollower_Step(&line, &imu, false, 42U, &request, &status);
    line = sample(0x06U, 22U, 44U);
    (void)LineFollower_Step(&line, &imu, false, 44U, &request, &status);
    line = sample(0x0CU, 23U, 46U);
    (void)LineFollower_Step(&line, &imu, false, 46U, &request, &status);
    line = sample(0x08U, 24U, 48U);
    (void)LineFollower_Step(&line, &imu, false, 48U, &request, &status);
    assert(status.trend > 0.0f && status.predicted_error > status.error);

    /* Separated runs are held briefly, then search follows the last side. */
    held = request;
    line = sample(0x05U, 25U, 50U);
    (void)LineFollower_Step(&line, &imu, false, 50U, &request, &status);
    assert(request.left_speed == held.left_speed &&
           request.right_speed == held.right_speed);
    line = sample(0x05U, 26U, 70U);
    (void)LineFollower_Step(&line, &imu, false, 70U, &request, &status);
    assert(status.mode == LINE_FOLLOWER_SEEK_RIGHT);

    return 0;
}
