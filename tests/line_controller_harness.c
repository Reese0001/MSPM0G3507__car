#include <assert.h>
#include <stdint.h>

#include "modules/line_tracking/line_controller.h"

static LineEstimate left_curve(void)
{
    LineEstimate line = {0};

    line.status.timestamp_ms = 1U;
    line.status.sequence = 1U;
    line.status.valid = true;
    line.status.health = MODULE_HEALTH_OK;
    line.error = 2.0f;
    line.predicted_error = 2.0f;
    line.confidence = 100U;
    line.event = LINE_EVENT_NONE;
    return line;
}

static LineTrendResult normal_trend(void)
{
    LineTrendResult trend = {0};

    trend.status.timestamp_ms = 1U;
    trend.status.sequence = 1U;
    trend.status.valid = true;
    trend.status.health = MODULE_HEALTH_OK;
    trend.type = LINE_TREND_NORMAL;
    return trend;
}

static LineControlOutput run_case(float yaw_rate_dps, bool yaw_fresh)
{
    LineEstimate line = left_curve();
    LineTrendResult trend = normal_trend();
    LineControlOutput output = {0};

    LineController_Reset();
    assert(LineController_Step(
        &line, &trend, yaw_rate_dps, yaw_fresh, 1U, &output));
    return output;
}

int main(void)
{
    LineControlConfig config = {
        300, 200, 150, 100, 100, 100, 50, 150,
        100, 100, 50, 100, 80U, 300, 300, 300,
        20.0f, 0.0f, 1.0f, 1.0f, 3.0f, 6.0f, 100.0f,
        1.0f, 0.5f, 20,
        20U, 40U, 3U, 20U
    };
    LineControlOutput no_imu;
    LineControlOutput stale_imu;
    LineControlOutput slow_left_yaw;
    LineControlOutput matched_left_yaw;
    LineControlOutput fast_left_yaw;

    config.yaw_rate_per_command = 1.0f;
    config.yaw_rate_kp = 0.5f;
    config.yaw_assist_limit = 20;
    assert(LineController_Init(&config));

    no_imu = run_case(0.0f, false);
    stale_imu = run_case(100.0f, false);
    slow_left_yaw = run_case(0.0f, true);
    matched_left_yaw = run_case(40.0f, true);
    fast_left_yaw = run_case(100.0f, true);

    assert(no_imu.turn == 40);
    assert(stale_imu.turn == no_imu.turn);
    assert(slow_left_yaw.turn > matched_left_yaw.turn);
    assert(matched_left_yaw.turn == no_imu.turn);
    assert(fast_left_yaw.turn < matched_left_yaw.turn);
    assert(no_imu.turn - fast_left_yaw.turn <= config.yaw_assist_limit);
    return 0;
}
