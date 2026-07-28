#include "line_recovery.h"

#include "config/line_recovery_config.h"

#define LINE_REQUEST_COMMAND_LIMIT (140)

static LineRecoveryDiagnostics recovery_diagnostics = {
    LINE_RECOVERY_FOLLOW, 0, 0.0f, false
};
static uint8_t reacquire_count = 0U;
static uint16_t last_line_sequence = 0U;
static bool has_last_line_sequence = false;
static uint32_t state_started_ms = 0U;
static float loss_yaw_deg = 0.0f;

static float absolute_value(float value)
{
    return value < 0.0f ? -value : value;
}

static int16_t clamp_command(int32_t command, int16_t limit)
{
    if (command < 0) {
        return 0;
    }
    if (command > limit) {
        return limit;
    }
    return (int16_t)command;
}

static void invalidate_request(MotionRequest *request, uint32_t now_ms)
{
    request->left_speed = 0;
    request->right_speed = 0;
    request->timestamp_ms = now_ms;
    request->valid = false;
}

static void publish_request(int32_t left,
                            int32_t right,
                            int16_t limit,
                            uint32_t now_ms,
                            MotionRequest *request)
{
    request->left_speed = clamp_command(left, limit);
    request->right_speed = clamp_command(right, limit);
    request->timestamp_ms = now_ms;
    request->valid = true;
}

static bool line_is_fresh(const LineEstimate *line, uint32_t now_ms)
{
    return line != 0 &&
           ModuleStatus_IsFresh(&line->status, now_ms,
                                LINE_RECOVERY_ESTIMATE_STALE_MS);
}

static bool line_is_trustworthy(const LineEstimate *line)
{
    return line->event == LINE_EVENT_NONE &&
           line->confidence >= LINE_RECOVERY_MIN_CONFIDENCE;
}

static bool take_new_line(const LineEstimate *line)
{
    if (has_last_line_sequence &&
        line->status.sequence == last_line_sequence) {
        return false;
    }
    last_line_sequence = line->status.sequence;
    has_last_line_sequence = true;
    return true;
}

static bool set_follow_request(const LineControlOutput *follow,
                               int16_t limit,
                               uint32_t now_ms,
                               MotionRequest *request)
{
    if (follow == 0 || !follow->valid) {
        invalidate_request(request, now_ms);
        return false;
    }
    publish_request((int32_t)follow->forward - follow->turn,
                    (int32_t)follow->forward + follow->turn,
                    limit, now_ms, request);
    return true;
}

static void set_seek_request(float yaw_rate_dps,
                             bool yaw_fresh,
                             uint32_t now_ms,
                             MotionRequest *request)
{
    int16_t speed = LINE_SEEK_COMMAND;

    if (yaw_fresh &&
        absolute_value(yaw_rate_dps) >= LINE_SEEK_HIGH_YAW_DPS) {
        speed = LINE_SEEK_LIMITED_COMMAND;
    }
    if (recovery_diagnostics.direction < 0) {
        publish_request(0, speed, LINE_REQUEST_COMMAND_LIMIT,
                        now_ms, request);
    } else {
        publish_request(speed, 0, LINE_REQUEST_COMMAND_LIMIT,
                        now_ms, request);
    }
}

static void enter_state(LineRecoveryState state, uint32_t now_ms)
{
    recovery_diagnostics.state = state;
    state_started_ms = now_ms;
}

static void begin_seek(int8_t predicted_direction,
                       bool lock_new_direction,
                       float yaw_deg,
                       uint32_t now_ms)
{
    if (lock_new_direction) {
        recovery_diagnostics.direction =
            predicted_direction < 0 ? (int8_t)-1 : (int8_t)1;
    } else if (recovery_diagnostics.direction == 0) {
        recovery_diagnostics.direction = 1;
    }
    loss_yaw_deg = yaw_deg;
    recovery_diagnostics.yaw_delta_deg = 0.0f;
    reacquire_count = 0U;
    enter_state(recovery_diagnostics.direction < 0
                    ? LINE_RECOVERY_SEEK_LEFT
                    : LINE_RECOVERY_SEEK_RIGHT,
                now_ms);
}

static bool update_reacquisition(const LineEstimate *line, bool new_line)
{
    if (!line_is_trustworthy(line)) {
        reacquire_count = 0U;
        return false;
    }
    if (!new_line) {
        return false;
    }
    if (reacquire_count < UINT8_MAX) {
        reacquire_count++;
    }
    return reacquire_count >= LINE_REACQUIRE_COUNT;
}

void LineRecovery_Init(void)
{
    recovery_diagnostics.state = LINE_RECOVERY_FOLLOW;
    recovery_diagnostics.direction = 0;
    recovery_diagnostics.yaw_delta_deg = 0.0f;
    recovery_diagnostics.yaw_fresh = false;
    reacquire_count = 0U;
    last_line_sequence = 0U;
    has_last_line_sequence = false;
    state_started_ms = 0U;
    loss_yaw_deg = 0.0f;
}

void LineRecovery_Reset(void)
{
    LineRecovery_Init();
}

LineRecoveryState LineRecovery_GetState(void)
{
    return recovery_diagnostics.state;
}

void LineRecovery_GetDiagnostics(LineRecoveryDiagnostics *out)
{
    if (out != 0) {
        *out = recovery_diagnostics;
    }
}

bool LineRecovery_Step(const LineEstimate *line,
                       int8_t predicted_direction,
                       const LineControlOutput *follow,
                       float yaw_deg,
                       float yaw_rate_dps,
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
    recovery_diagnostics.yaw_delta_deg = yaw_deg - loss_yaw_deg;
    recovery_diagnostics.yaw_fresh = yaw_fresh;
    invalidate_request(request, now_ms);

    if (emergency_stop) {
        reacquire_count = 0U;
        enter_state(LINE_RECOVERY_STOPPED, now_ms);
        return false;
    }
    if (!line_is_fresh(line, now_ms)) {
        reacquire_count = 0U;
        enter_state(LINE_RECOVERY_STOPPED, now_ms);
        return false;
    }

    new_line = take_new_line(line);
    lost = line->event == LINE_EVENT_LOST;

    switch (recovery_diagnostics.state) {
        case LINE_RECOVERY_FOLLOW:
            if (lost) {
                begin_seek(predicted_direction, true, yaw_deg, now_ms);
                set_seek_request(yaw_rate_dps, yaw_fresh, now_ms, request);
                return true;
            }
            return set_follow_request(follow, LINE_REQUEST_COMMAND_LIMIT,
                                      now_ms, request);

        case LINE_RECOVERY_SEEK_LEFT:
        case LINE_RECOVERY_SEEK_RIGHT:
            if (update_reacquisition(line, new_line)) {
                enter_state(LINE_RECOVERY_ALIGN, now_ms);
                return set_follow_request(follow, LINE_ALIGN_COMMAND_LIMIT,
                                          now_ms, request);
            }
            set_seek_request(yaw_rate_dps, yaw_fresh, now_ms, request);
            return true;

        case LINE_RECOVERY_ALIGN:
            if (lost) {
                begin_seek(predicted_direction, false, yaw_deg, now_ms);
                set_seek_request(yaw_rate_dps, yaw_fresh, now_ms, request);
                return true;
            }
            if ((uint32_t)(now_ms - state_started_ms) >=
                LINE_ALIGN_DURATION_MS) {
                reacquire_count = 0U;
                enter_state(LINE_RECOVERY_FOLLOW, now_ms);
                return set_follow_request(follow, LINE_REQUEST_COMMAND_LIMIT,
                                          now_ms, request);
            }
            return set_follow_request(follow, LINE_ALIGN_COMMAND_LIMIT,
                                      now_ms, request);

        case LINE_RECOVERY_STOPPED:
            if (lost) {
                begin_seek(predicted_direction, true, yaw_deg, now_ms);
                set_seek_request(yaw_rate_dps, yaw_fresh, now_ms, request);
                return true;
            }
            if (update_reacquisition(line, new_line)) {
                enter_state(LINE_RECOVERY_ALIGN, now_ms);
                return set_follow_request(follow, LINE_ALIGN_COMMAND_LIMIT,
                                          now_ms, request);
            }
            return false;

        default:
            reacquire_count = 0U;
            enter_state(LINE_RECOVERY_STOPPED, now_ms);
            return false;
    }
}
