#include "line_controller.h"

static int16_t previous_forward = 0;
static int16_t previous_turn = 0;
static LineControlConfig control_config = {0};
static bool config_valid = false;

static float absolute_value(float value)
{
    return value < 0.0f ? -value : value;
}

static int16_t absolute_int16(int16_t value)
{
    return value < 0 ? (int16_t)-value : value;
}

static int16_t minimum(int16_t left, int16_t right)
{
    return left < right ? left : right;
}

static int16_t plan_target_speed(const LineEstimate *estimate,
                                 float yaw_rate_dps,
                                 bool yaw_fresh)
{
    float curve = absolute_value(estimate->predicted_error);
    int16_t target = control_config.max_forward;

    if (estimate->event == LINE_EVENT_HARD_LEFT ||
        estimate->event == LINE_EVENT_HARD_RIGHT) {
        target = control_config.hard_turn_forward;
    } else if (curve >= control_config.hard_curve_error_threshold) {
        target = control_config.hard_curve_forward;
    } else if (curve >= control_config.curve_error_threshold) {
        target = control_config.curve_forward;
    }
    if (estimate->confidence < control_config.low_confidence) {
        target = minimum(target, control_config.low_confidence_forward);
    } else if (estimate->confidence < control_config.medium_confidence) {
        target = minimum(target, control_config.curve_forward);
    }
    if (yaw_fresh &&
        absolute_value(yaw_rate_dps) >= control_config.high_yaw_rate_dps) {
        target = minimum(target, control_config.hard_curve_forward);
    }
    if (estimate->event == LINE_EVENT_HARD_LEFT ||
        estimate->event == LINE_EVENT_HARD_RIGHT) {
        target = minimum(target, control_config.hard_curve_forward);
    } else if (estimate->event == LINE_EVENT_WIDE_BLACK) {
        target = minimum(target, control_config.wide_black_forward);
    }
    return target;
}

static int16_t slew_forward(int16_t target)
{
    if (target > previous_forward) {
        int16_t next = (int16_t)(previous_forward + control_config.accel_step);
        previous_forward = next < target ? next : target;
    } else if (target < previous_forward) {
        int16_t next = (int16_t)(previous_forward - control_config.decel_step);
        previous_forward = next > target ? next : target;
    }
    return previous_forward;
}

static int16_t slew_turn(int16_t target, int16_t limit)
{
    if (previous_turn > limit) {
        previous_turn = limit;
    } else if (previous_turn < -limit) {
        previous_turn = (int16_t)-limit;
    }

    if (target > previous_turn) {
        int16_t next =
            (int16_t)(previous_turn + control_config.turn_slew_step);
        previous_turn = next < target ? next : target;
    } else if (target < previous_turn) {
        int16_t next =
            (int16_t)(previous_turn - control_config.turn_slew_step);
        previous_turn = next > target ? next : target;
    }
    return previous_turn;
}

bool LineController_Init(const LineControlConfig *settings)
{
    if (settings == 0 || settings->max_forward <= 0 ||
        settings->max_forward > 450 || settings->curve_forward <= 0 ||
        settings->hard_curve_forward <= 0 ||
        settings->wide_black_forward <= 0 ||
        settings->low_confidence_forward <= 0 ||
        settings->hard_turn_forward < 0 ||
        settings->hard_turn_command <= settings->hard_turn_forward ||
        settings->hard_turn_command > settings->max_forward ||
        settings->curve_forward > settings->max_forward ||
        settings->hard_curve_forward > settings->max_forward ||
        settings->wide_black_forward > settings->max_forward ||
        settings->low_confidence_forward > settings->max_forward ||
        settings->turn_limit_percent > 80U || settings->accel_step <= 0 ||
        settings->decel_step <= 0 || settings->turn_slew_step <= 0 ||
        settings->low_confidence > 100U ||
        settings->medium_confidence > 100U ||
        settings->low_confidence > settings->medium_confidence ||
        settings->estimate_stale_ms == 0U) {
        config_valid = false;
        previous_forward = 0;
        previous_turn = 0;
        return false;
    }
    control_config = *settings;
    config_valid = true;
    previous_forward = 0;
    previous_turn = 0;
    return true;
}

void LineController_Reset(void)
{
    previous_forward = 0;
    previous_turn = 0;
}

bool LineController_Step(const LineEstimate *estimate,
                         float yaw_rate_dps,
                         bool yaw_fresh,
                         uint32_t now_ms,
                         LineControlOutput *output)
{
    int16_t forward;
    int16_t turn;
    int16_t turn_limit;
    float raw_turn;

    if (output == 0) {
        return false;
    }
    output->forward = 0;
    output->turn = 0;
    output->valid = false;
    if (!config_valid || estimate == 0 ||
        !ModuleStatus_IsFresh(&estimate->status, now_ms,
                              control_config.estimate_stale_ms) ||
        estimate->event == LINE_EVENT_LOST) {
        previous_forward = 0;
        previous_turn = 0;
        return false;
    }

    forward = slew_forward(plan_target_speed(estimate, yaw_rate_dps,
                                              yaw_fresh));
    raw_turn = control_config.steering_polarity *
               (control_config.kp * estimate->error +
                control_config.kd * estimate->derivative);
    if (estimate->event == LINE_EVENT_HARD_LEFT ||
        estimate->event == LINE_EVENT_HARD_RIGHT) {
        turn_limit = control_config.hard_turn_command;
        raw_turn = raw_turn < 0.0f ? (float)-turn_limit :
                                      (float)turn_limit;
    } else {
        turn_limit = (int16_t)(((int32_t)forward *
                                control_config.turn_limit_percent) / 100);
    }
    if (raw_turn > (float)turn_limit) {
        turn = turn_limit;
    } else if (raw_turn < (float)-turn_limit) {
        turn = (int16_t)-turn_limit;
    } else {
        turn = (int16_t)raw_turn;
    }
    turn = slew_turn(turn, turn_limit);
    {
        int16_t turn_magnitude = absolute_int16(turn);
        if (forward + turn_magnitude > control_config.max_forward) {
            forward = (int16_t)(control_config.max_forward - turn_magnitude);
            previous_forward = forward;
        }
    }

    output->forward = forward;
    output->turn = turn;
    output->valid = true;
    return true;
}
