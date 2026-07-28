#include "line_cascade_control.h"

#include "../../mpu6050/mpu6050_config.h"
#include "../../../config/line_cascade_config.h"

typedef struct {
    float target_yaw_deg;
    float previous_error;
    float filtered_derivative;
    uint32_t last_step_ms;
    int16_t forward;
    int16_t turn;
    bool yaw_target_valid;
    bool error_valid;
    bool imu_used;
} LineCascadeState;

static LineCascadeState controller;

static float absolute_float(float value)
{
    return value < 0.0f ? -value : value;
}

static int16_t absolute_int16(int16_t value)
{
    return value < 0 ? (int16_t)-value : value;
}

static float clamp_float(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static int16_t round_command(float value)
{
    value += value >= 0.0f ? 0.5f : -0.5f;
    return (int16_t)clamp_float(value, (float)LINE_CASCADE_MAX_COMMAND);
}

static uint32_t bounded_elapsed_ms(uint32_t now_ms)
{
    uint32_t elapsed = (uint32_t)(now_ms - controller.last_step_ms);

    controller.last_step_ms = now_ms;
    if (elapsed > LINE_CASCADE_MAX_DT_MS) {
        return LINE_CASCADE_MAX_DT_MS;
    }
    return elapsed;
}

static float position_command(const LineEstimate *estimate, uint32_t elapsed_ms)
{
    float derivative = 0.0f;
    float effective_error = estimate->error;
    float command;

    if (absolute_float(effective_error) <= 1.0f) {
        controller.filtered_derivative = 0.0f;
        controller.previous_error = 0.0f;
        controller.error_valid = true;
        return 0.0f;
    }
    if (controller.error_valid && elapsed_ms > 0U) {
        derivative = (effective_error - controller.previous_error) *
                     (1000.0f / (float)elapsed_ms);
    }
    controller.filtered_derivative +=
        LINE_CASCADE_DERIVATIVE_ALPHA *
        (derivative - controller.filtered_derivative);
    controller.previous_error = effective_error;
    controller.error_valid = true;

    command = (-effective_error * LINE_CASCADE_POSITION_KP) -
              (controller.filtered_derivative * LINE_CASCADE_POSITION_KD);
    return clamp_float(command, LINE_CASCADE_MAX_POSITION_COMMAND);
}

static bool imu_is_fresh(const Mpu6050Snapshot *imu,
                         bool imu_fresh,
                         uint32_t now_ms)
{
    return imu_fresh && imu != 0 &&
           ModuleStatus_IsFresh(&imu->status, now_ms, MPU6050_STALE_MS);
}

static float angle_loop(float outer_command,
                        const Mpu6050Snapshot *imu,
                        bool fresh_imu,
                        uint32_t elapsed_ms)
{
    float angle_error;
    float dt_s;

    if (!fresh_imu) {
        controller.yaw_target_valid = false;
        return outer_command;
    }
    if (!controller.yaw_target_valid) {
        controller.target_yaw_deg = imu->yaw_angle_deg;
        controller.yaw_target_valid = true;
    }

    dt_s = (float)elapsed_ms / 1000.0f;
    controller.target_yaw_deg +=
        outer_command * LINE_CASCADE_YAW_RATE_PER_COMMAND * dt_s;
    angle_error = controller.target_yaw_deg - imu->yaw_angle_deg;
    angle_error = clamp_float(angle_error, LINE_CASCADE_MAX_YAW_ERROR_DEG);
    controller.target_yaw_deg = imu->yaw_angle_deg + angle_error;
    return outer_command +
           (angle_error * LINE_CASCADE_ANGLE_KP) -
           (imu->yaw_rate_dps * LINE_CASCADE_ANGLE_KD);
}

static int16_t slew(int16_t current,
                    int16_t target,
                    int16_t rising_step,
                    int16_t falling_step)
{
    if (target > current) {
        int32_t next = (int32_t)current + rising_step;
        return next < target ? (int16_t)next : target;
    }
    if (target < current) {
        int32_t next = (int32_t)current - falling_step;
        return next > target ? (int16_t)next : target;
    }
    return current;
}

static void limit_wheels(int16_t *forward, int16_t *turn)
{
    int16_t turn_limit;

    if (*forward < 0) {
        *forward = 0;
    }
    if (*forward > LINE_CASCADE_MAX_COMMAND) {
        *forward = LINE_CASCADE_MAX_COMMAND;
    }
    turn_limit = *forward;
    if (turn_limit > LINE_CASCADE_MAX_COMMAND - *forward) {
        turn_limit = (int16_t)(LINE_CASCADE_MAX_COMMAND - *forward);
    }
    if (absolute_int16(*turn) > turn_limit) {
        *turn = *turn < 0 ? (int16_t)-turn_limit : turn_limit;
    }
}

void LineCascadeControl_Init(uint32_t now_ms)
{
    controller = (LineCascadeState){0};
    controller.last_step_ms = now_ms;
}

bool LineCascadeControl_IsImuUsed(void)
{
    return controller.imu_used;
}

static void invalidate_control(uint32_t now_ms)
{
    controller = (LineCascadeState){0};
    controller.last_step_ms = now_ms;
}

bool LineCascadeControl_Step(const LineEstimate *estimate,
                             const LineLookupCommand *feedforward,
                             const Mpu6050Snapshot *imu,
                             bool imu_fresh,
                             uint32_t now_ms,
                             LineControlOutput *output)
{
    uint32_t elapsed_ms;
    float position_feedback;
    float desired_turn;
    float combined_turn;

    if (output == 0) {
        return false;
    }
    *output = (LineControlOutput){0};
    if (estimate == 0 || feedforward == 0 || !feedforward->valid ||
        !ModuleStatus_IsFresh(&estimate->status, now_ms,
                              LINE_CASCADE_ESTIMATE_STALE_MS) ||
        estimate->event == LINE_EVENT_LOST) {
        invalidate_control(now_ms);
        return false;
    }

    elapsed_ms = bounded_elapsed_ms(now_ms);
    position_feedback = position_command(estimate, elapsed_ms);
    desired_turn = (float)feedforward->diff + position_feedback;
    imu_fresh = imu_is_fresh(imu, imu_fresh, now_ms);
    controller.imu_used = imu_fresh;
    combined_turn = angle_loop(desired_turn, imu, imu_fresh, elapsed_ms);
    controller.forward = slew(controller.forward,
                              feedforward->base,
                              LINE_CASCADE_ACCEL_STEP,
                              LINE_CASCADE_DECEL_STEP);
    controller.turn = slew(controller.turn, round_command(combined_turn),
                           LINE_CASCADE_TURN_SLEW_STEP,
                           LINE_CASCADE_TURN_SLEW_STEP);
    limit_wheels(&controller.forward, &controller.turn);

    output->forward = controller.forward;
    output->turn = controller.turn;
    output->valid = true;
    return true;
}
