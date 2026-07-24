#include "line_recovery.h"

#include "config/line_recovery_config.h"

static LineRecoveryState recovery_state = LINE_RECOVERY_FOLLOW;
static int8_t recovery_direction = 0;
static uint8_t loss_count = 0U;
static uint8_t reacquire_count = 0U;
static uint16_t last_line_sequence = 0U;
static uint32_t recovery_started_ms = 0U;
static uint32_t state_started_ms = 0U;

static float absolute_value(float value)
{
    return value < 0.0f ? -value : value;
}

static void invalidate_request(MotionRequest *request, uint32_t now_ms)
{
    request->left_speed = 0;
    request->right_speed = 0;
    request->timestamp_ms = now_ms;
    request->valid = false;
}

static void publish_request(int16_t left,
                            int16_t right,
                            uint32_t now_ms,
                            MotionRequest *request)
{
    request->left_speed = left;
    request->right_speed = right;
    request->timestamp_ms = now_ms;
    request->valid = true;
}

static bool line_is_fresh(const LineEstimate *line, uint32_t now_ms)
{
    return line != 0 &&
           ModuleStatus_IsFresh(&line->status, now_ms,
                                LINE_RECOVERY_ESTIMATE_STALE_MS);
}

static bool trend_is_fresh(const LineTrendResult *trend, uint32_t now_ms)
{
    return trend != 0 &&
           ModuleStatus_IsFresh(&trend->status, now_ms,
                                LINE_RECOVERY_ESTIMATE_STALE_MS);
}

static bool line_is_trustworthy(const LineEstimate *line)
{
    return line->event != LINE_EVENT_LOST &&
           line->confidence >= LINE_RECOVERY_MIN_CONFIDENCE &&
           absolute_value(line->error) <= LINE_RECOVERY_CENTER_ERROR;
}

static bool take_new_line(const LineEstimate *line)
{
    if (line->status.sequence == last_line_sequence) {
        return false;
    }
    last_line_sequence = line->status.sequence;
    return true;
}

static bool set_follow_request(const LineControlOutput *follow,
                               uint32_t now_ms,
                               MotionRequest *request)
{
    if (follow == 0 || !follow->valid) {
        invalidate_request(request, now_ms);
        return false;
    }
    publish_request((int16_t)(follow->forward - follow->turn),
                    (int16_t)(follow->forward + follow->turn),
                    now_ms, request);
    return true;
}

static void set_forward_search(uint32_t now_ms, MotionRequest *request)
{
    if (recovery_direction < 0) {
        publish_request(LINE_SEARCH_INNER_COMMAND,
                        LINE_SEARCH_OUTER_COMMAND, now_ms, request);
    } else if (recovery_direction > 0) {
        publish_request(LINE_SEARCH_OUTER_COMMAND,
                        LINE_SEARCH_INNER_COMMAND, now_ms, request);
    } else {
        publish_request(LINE_SEARCH_STRAIGHT_COMMAND,
                        LINE_SEARCH_STRAIGHT_COMMAND, now_ms, request);
    }
}

static void set_pause_request(uint32_t now_ms, MotionRequest *request)
{
    publish_request(0, 0, now_ms, request);
}

static void set_rotate_search(uint32_t now_ms, MotionRequest *request)
{
    if (recovery_direction < 0) {
        publish_request(LINE_ROTATE_INNER_COMMAND,
                        LINE_ROTATE_OUTER_COMMAND, now_ms, request);
    } else if (recovery_direction > 0) {
        publish_request(LINE_ROTATE_OUTER_COMMAND,
                        LINE_ROTATE_INNER_COMMAND, now_ms, request);
    } else {
        invalidate_request(request, now_ms);
    }
}

static void enter_state(LineRecoveryState state, uint32_t now_ms)
{
    recovery_state = state;
    state_started_ms = now_ms;
}

static void enter_fault(uint32_t now_ms, MotionRequest *request)
{
    enter_state(LINE_RECOVERY_FAULT, now_ms);
    invalidate_request(request, now_ms);
}

static void update_direction(const LineEstimate *line,
                             const LineTrendResult *trend,
                             uint32_t now_ms)
{
    if (trend_is_fresh(trend, now_ms) && trend->direction != 0) {
        recovery_direction = trend->direction;
    } else if (line->predicted_error < 0.0f) {
        recovery_direction = -1;
    } else if (line->predicted_error > 0.0f) {
        recovery_direction = 1;
    }
}

static bool update_reacquisition(const LineEstimate *line, bool new_line)
{
    if (!new_line) {
        return false;
    }
    if (line_is_trustworthy(line)) {
        if (reacquire_count < UINT8_MAX) {
            reacquire_count++;
        }
    } else {
        reacquire_count = 0U;
    }
    return reacquire_count >= LINE_REACQUIRE_COUNT;
}

void LineRecovery_Init(void)
{
    recovery_state = LINE_RECOVERY_FOLLOW;
    recovery_direction = 0;
    loss_count = 0U;
    reacquire_count = 0U;
    last_line_sequence = 0U;
    recovery_started_ms = 0U;
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
                       const LineTrendResult *trend,
                       const LineControlOutput *follow,
                       float yaw_deg,
                       bool yaw_fresh,
                       bool emergency_stop,
                       uint32_t now_ms,
                       MotionRequest *request)
{
    bool new_line;
    bool lost;

    (void)yaw_deg;
    (void)yaw_fresh;
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
    if (!line_is_fresh(line, now_ms)) {
        enter_fault(now_ms, request);
        return false;
    }

    new_line = take_new_line(line);
    lost = line->event == LINE_EVENT_LOST;
    if (recovery_state == LINE_RECOVERY_FOLLOW ||
        recovery_state == LINE_RECOVERY_LOSS_CONFIRM) {
        update_direction(line, trend, now_ms);
    }

    if (recovery_state == LINE_RECOVERY_FOLLOW) {
        if (lost && new_line) {
            loss_count = 1U;
            recovery_started_ms = now_ms;
            enter_state(LINE_RECOVERY_LOSS_CONFIRM, now_ms);
        }
        if (lost) {
            return false;
        }
        return set_follow_request(follow, now_ms, request);
    }

    if ((uint32_t)(now_ms - recovery_started_ms) >=
        LINE_RECOVERY_TOTAL_TIMEOUT_MS) {
        enter_fault(now_ms, request);
        return false;
    }

    if (recovery_state == LINE_RECOVERY_LOSS_CONFIRM) {
        if (!lost) {
            recovery_state = LINE_RECOVERY_FOLLOW;
            loss_count = 0U;
            return set_follow_request(follow, now_ms, request);
        }
        if (new_line && loss_count < UINT8_MAX) {
            loss_count++;
        }
        if (loss_count >= LINE_LOSS_CONFIRM_COUNT) {
            reacquire_count = 0U;
            enter_state(LINE_RECOVERY_FORWARD_SEARCH, now_ms);
            set_forward_search(now_ms, request);
            return true;
        }
        return false;
    }

    if (recovery_state != LINE_RECOVERY_ALIGN &&
        update_reacquisition(line, new_line)) {
        enter_state(LINE_RECOVERY_ALIGN, now_ms);
        return set_follow_request(follow, now_ms, request);
    }

    if (recovery_state == LINE_RECOVERY_FORWARD_SEARCH) {
        if ((uint32_t)(now_ms - state_started_ms) >=
            LINE_FORWARD_SEARCH_MS) {
            enter_state(LINE_RECOVERY_ROTATION_PAUSE, now_ms);
            set_pause_request(now_ms, request);
        } else {
            set_forward_search(now_ms, request);
        }
        return true;
    }

    if (recovery_state == LINE_RECOVERY_ROTATION_PAUSE) {
        if ((uint32_t)(now_ms - state_started_ms) >=
            LINE_ROTATION_PAUSE_MS) {
            if (recovery_direction == 0) {
                enter_fault(now_ms, request);
                return false;
            }
            enter_state(LINE_RECOVERY_ROTATE_SEARCH, now_ms);
            set_rotate_search(now_ms, request);
        } else {
            set_pause_request(now_ms, request);
        }
        return true;
    }

    if (recovery_state == LINE_RECOVERY_ROTATE_SEARCH) {
        if ((uint32_t)(now_ms - state_started_ms) >=
            LINE_ROTATE_SEARCH_MS) {
            enter_fault(now_ms, request);
            return false;
        }
        set_rotate_search(now_ms, request);
        return true;
    }

    if (recovery_state == LINE_RECOVERY_ALIGN) {
        if (lost) {
            loss_count = new_line ? 1U : 0U;
            recovery_started_ms = now_ms;
            enter_state(LINE_RECOVERY_LOSS_CONFIRM, now_ms);
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
