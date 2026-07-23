#include "line_recovery.h"

#include "config/line_recovery_config.h"

static LineRecoveryState recovery_state = LINE_RECOVERY_FOLLOW;
static uint8_t loss_count = 0U;
static uint8_t reacquire_count = 0U;
static uint16_t last_line_sequence = 0U;
static float last_seen_error = 0.0f;
static float pivot_start_yaw_deg = 0.0f;
static uint32_t state_started_ms = 0U;

static float absolute_value(float value)
{
    return value < 0.0f ? -value : value;
}

static float relative_yaw(float current, float start)
{
    float delta = current - start;
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }
    return delta;
}

static void invalidate_request(MotionRequest *request, uint32_t now_ms)
{
    request->left_speed = 0;
    request->right_speed = 0;
    request->timestamp_ms = now_ms;
    request->valid = false;
}

static bool line_is_fresh(const LineEstimate *line, uint32_t now_ms)
{
    return line != 0 &&
           ModuleStatus_IsFresh(&line->status, now_ms,
                                LINE_RECOVERY_ESTIMATE_STALE_MS);
}

static bool line_is_lost(const LineEstimate *line, uint32_t now_ms)
{
    return !line_is_fresh(line, now_ms) || line->event == LINE_EVENT_LOST;
}

static bool take_new_line(const LineEstimate *line)
{
    if (line == 0 || line->status.sequence == last_line_sequence) {
        return false;
    }
    last_line_sequence = line->status.sequence;
    return true;
}

static bool set_follow_request(const LineControlOutput *follow,
                               uint32_t now_ms,
                               MotionRequest *request)
{
    int16_t left;
    int16_t right;

    if (follow == 0 || !follow->valid) {
        invalidate_request(request, now_ms);
        return false;
    }
    left = (int16_t)(follow->forward - follow->turn);
    right = (int16_t)(follow->forward + follow->turn);
    request->left_speed = left;
    request->right_speed = right;
    request->timestamp_ms = now_ms;
    request->valid = true;
    return true;
}

static bool sharp_search_required(void)
{
    return absolute_value(last_seen_error) >= LINE_SHARP_SEARCH_ERROR;
}

static void set_pivot_request(LineRecoveryState state,
                              bool sharp_search,
                              uint32_t now_ms,
                              MotionRequest *request)
{
    int16_t inner_speed = sharp_search ?
        -LINE_SHARP_INNER_REVERSE_COMMAND : LINE_SEARCH_INNER_COMMAND;

    if (state == LINE_RECOVERY_PIVOT_LEFT) {
        request->left_speed = inner_speed;
        request->right_speed = LINE_PIVOT_FORWARD_COMMAND;
    } else {
        request->left_speed = LINE_PIVOT_FORWARD_COMMAND;
        request->right_speed = inner_speed;
    }
    request->timestamp_ms = now_ms;
    request->valid = true;
}

static void enter_fault(uint32_t now_ms, MotionRequest *request)
{
    recovery_state = LINE_RECOVERY_FAULT;
    invalidate_request(request, now_ms);
}

void LineRecovery_Init(void)
{
    recovery_state = LINE_RECOVERY_FOLLOW;
    loss_count = 0U;
    reacquire_count = 0U;
    last_line_sequence = 0U;
    last_seen_error = 0.0f;
    pivot_start_yaw_deg = 0.0f;
    state_started_ms = 0U;
}

void LineRecovery_Reset(void)
{
    LineRecovery_Init();
}

LineRecoveryState LineRecovery_GetState(void)
{
    return recovery_state;
}

bool LineRecovery_Step(const LineEstimate *line,
                       const LineControlOutput *follow,
                       float yaw_deg,
                       bool yaw_fresh,
                       bool emergency_stop,
                       uint32_t now_ms,
                       MotionRequest *request)
{
    bool new_line;
    bool lost;

    if (request == 0) {
        return false;
    }
    invalidate_request(request, now_ms);
    if (recovery_state == LINE_RECOVERY_FAULT) {
        return false;
    }
    if (emergency_stop) {
        enter_fault(now_ms, request);
        return false;
    }

    new_line = take_new_line(line);
    lost = line_is_lost(line, now_ms);

    if (recovery_state == LINE_RECOVERY_FOLLOW) {
        if (lost) {
            if (new_line) {
                loss_count = 1U;
                recovery_state = LINE_RECOVERY_LOSS_CONFIRM;
                state_started_ms = now_ms;
            }
            return false;
        }
        if (new_line) {
            last_seen_error = line->predicted_error;
        }
        return set_follow_request(follow, now_ms, request);
    }

    if (!line_is_fresh(line, now_ms)) {
        enter_fault(now_ms, request);
        return false;
    }

    if (recovery_state == LINE_RECOVERY_LOSS_CONFIRM) {
        if (!lost) {
            recovery_state = LINE_RECOVERY_FOLLOW;
            loss_count = 0U;
            last_seen_error = line->predicted_error;
            return set_follow_request(follow, now_ms, request);
        }
        if (new_line && loss_count < UINT8_MAX) {
            loss_count++;
        }
        if (loss_count >= LINE_LOSS_CONFIRM_COUNT) {
            recovery_state = last_seen_error <= 0.0f ?
                LINE_RECOVERY_PIVOT_LEFT : LINE_RECOVERY_PIVOT_RIGHT;
            pivot_start_yaw_deg = yaw_deg;
            state_started_ms = now_ms;
            reacquire_count = 0U;
            set_pivot_request(recovery_state, sharp_search_required(),
                              now_ms, request);
            return true;
        }
        return false;
    }

    if (recovery_state == LINE_RECOVERY_PIVOT_LEFT ||
        recovery_state == LINE_RECOVERY_PIVOT_RIGHT) {
        if ((uint32_t)(now_ms - state_started_ms) >=
                LINE_RECOVERY_TIMEOUT_MS ||
            (yaw_fresh &&
             absolute_value(relative_yaw(yaw_deg, pivot_start_yaw_deg)) >=
                 LINE_RECOVERY_MAX_YAW_DEG)) {
            enter_fault(now_ms, request);
            return false;
        }
        if (new_line) {
            if (!lost && line->confidence >= 40U) {
                if (reacquire_count < UINT8_MAX) {
                    reacquire_count++;
                }
            } else {
                reacquire_count = 0U;
            }
        }
        if (reacquire_count >= LINE_REACQUIRE_COUNT) {
            recovery_state = LINE_RECOVERY_ALIGN;
            state_started_ms = now_ms;
        }
        set_pivot_request(
            recovery_state == LINE_RECOVERY_ALIGN ?
                (last_seen_error <= 0.0f ?
                 LINE_RECOVERY_PIVOT_LEFT :
                 LINE_RECOVERY_PIVOT_RIGHT) : recovery_state,
            recovery_state != LINE_RECOVERY_ALIGN &&
                sharp_search_required(),
            now_ms, request);
        return true;
    }

    if (recovery_state == LINE_RECOVERY_ALIGN) {
        if (lost) {
            recovery_state = LINE_RECOVERY_LOSS_CONFIRM;
            loss_count = new_line ? 1U : 0U;
            state_started_ms = now_ms;
            return false;
        }
        if ((uint32_t)(now_ms - state_started_ms) >=
            LINE_ALIGN_DURATION_MS) {
            recovery_state = LINE_RECOVERY_FOLLOW;
            loss_count = 0U;
            reacquire_count = 0U;
        }
        return set_follow_request(follow, now_ms, request);
    }

    enter_fault(now_ms, request);
    return false;
}
