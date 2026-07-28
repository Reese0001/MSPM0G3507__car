#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "modules/line_tracking/controller/line_cascade_control.h"

static LineEstimate make_estimate(uint32_t now_ms, float error)
{
    LineEstimate estimate = {0};

    estimate.status.timestamp_ms = now_ms;
    estimate.status.sequence = (uint16_t)(now_ms + 1U);
    estimate.status.valid = true;
    estimate.status.health = MODULE_HEALTH_OK;
    estimate.error = error;
    estimate.predicted_error = error;
    estimate.confidence = 80U;
    estimate.event = LINE_EVENT_NONE;
    return estimate;
}

static Mpu6050Snapshot make_snapshot(uint32_t now_ms, float rate_dps)
{
    Mpu6050Snapshot snapshot = {0};

    snapshot.status.timestamp_ms = now_ms;
    snapshot.status.sequence = (uint16_t)(now_ms + 2U);
    snapshot.status.valid = true;
    snapshot.status.health = MODULE_HEALTH_OK;
    snapshot.yaw_rate_dps = rate_dps;
    return snapshot;
}

static LineLookupCommand make_feedforward(int16_t base, int16_t diff)
{
    LineLookupCommand command = {0};

    command.base = base;
    command.diff = diff;
    command.valid = true;
    return command;
}

static LineControlOutput run_case(float error,
                                  const LineLookupCommand *feedforward,
                                  bool imu_fresh,
                                  float yaw_rate_dps)
{
    LineControlOutput output = {0};
    uint32_t now_ms;

    LineCascadeControl_Init(0U);
    for (now_ms = 20U; now_ms <= 800U; now_ms += 20U) {
        LineEstimate estimate = make_estimate(now_ms, error);
        Mpu6050Snapshot imu = make_snapshot(now_ms, yaw_rate_dps);

        assert(LineCascadeControl_Step(&estimate, feedforward,
                                       imu_fresh ? &imu : 0, imu_fresh,
                                       now_ms, &output));
    }
    return output;
}

static void expect_missing_or_invalid_feedforward_fails(void)
{
    LineEstimate estimate = make_estimate(20U, 0.0f);
    LineLookupCommand invalid = {0};
    LineControlOutput output = {0};

    LineCascadeControl_Init(0U);
    assert(!LineCascadeControl_Step(&estimate, 0, 0, false, 20U, &output));
    assert(!LineCascadeControl_Step(&estimate, &invalid, 0, false,
                                    20U, &output));
}

static void expect_feedforward_turn_survives_centered_line(void)
{
    LineLookupCommand feedforward = make_feedforward(115, 28);
    LineControlOutput output = run_case(0.0f, &feedforward, false, 0.0f);

    assert(output.turn > 0);
}

static void expect_position_feedback_changes_turn(void)
{
    LineLookupCommand feedforward = make_feedforward(115, 12);
    LineControlOutput centered = run_case(0.0f, &feedforward, false, 0.0f);
    LineControlOutput offset = run_case(-3.0f, &feedforward, false, 0.0f);

    assert(offset.turn > centered.turn);
}

static void expect_fresh_imu_changes_angle_loop_result(void)
{
    LineLookupCommand feedforward = make_feedforward(80, 28);
    LineControlOutput stale = run_case(0.0f, &feedforward, false, 0.0f);
    LineControlOutput fresh = run_case(0.0f, &feedforward, true, 20.0f);

    assert(stale.turn != fresh.turn);
}

static void expect_center_deadzone_ignores_minus_one_zero_plus_one(void)
{
    LineLookupCommand feedforward = make_feedforward(115, 12);
    LineControlOutput left = run_case(-1.0f, &feedforward, false, 0.0f);
    LineControlOutput center = run_case(0.0f, &feedforward, false, 0.0f);
    LineControlOutput right = run_case(1.0f, &feedforward, false, 0.0f);

    assert(left.turn == center.turn);
    assert(right.turn == center.turn);
}

static void expect_wheel_limits_hold(void)
{
    LineLookupCommand feedforward = make_feedforward(140, 140);
    LineControlOutput output = run_case(-7.0f, &feedforward, true, 0.0f);
    int16_t left = (int16_t)(output.forward - output.turn);
    int16_t right = (int16_t)(output.forward + output.turn);

    assert(output.forward >= 0);
    assert(output.turn <= output.forward && -output.turn <= output.forward);
    assert(left >= 0 && left <= 140);
    assert(right >= 0 && right <= 140);
}

int main(void)
{
    expect_missing_or_invalid_feedforward_fails();
    expect_feedforward_turn_survives_centered_line();
    expect_position_feedback_changes_turn();
    expect_fresh_imu_changes_angle_loop_result();
    expect_center_deadzone_ignores_minus_one_zero_plus_one();
    expect_wheel_limits_hold();
    return 0;
}
