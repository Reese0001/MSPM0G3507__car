#include "corner_maneuver.h"

#include "config/corner_maneuver_config.h"

static CornerManeuverState state;
static int8_t corner_direction;
static uint8_t reacquire_frames;
static uint16_t last_feature_sequence;
static uint32_t maneuver_started_ms;
static uint32_t state_started_ms;
static bool event_rearm_required;

static int16_t limit_command(int16_t value)
{
    if (value > 450) {
        return 450;
    }
    if (value < -450) {
        return -450;
    }
    return value;
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
    request->left_speed = limit_command(left);
    request->right_speed = limit_command(right);
    request->timestamp_ms = now_ms;
    request->valid = true;
}

static bool features_are_fresh(const LineFeatures *features, uint32_t now_ms)
{
    return features != 0 &&
           ModuleStatus_IsFresh(&features->status, now_ms,
                                CORNER_FEATURE_STALE_MS);
}

static bool path_event_is_fresh(const LinePathEvent *path_event,
                                uint32_t now_ms)
{
    return path_event != 0 &&
           ModuleStatus_IsFresh(&path_event->status, now_ms,
                                CORNER_FEATURE_STALE_MS);
}

static bool set_follow_request(const LineControlOutput *follow,
                               uint32_t now_ms,
                               MotionRequest *request)
{
    int32_t left;
    int32_t right;

    if (follow == 0 || !follow->valid) {
        invalidate_request(request, now_ms);
        return false;
    }
    left = (int32_t)follow->forward - (int32_t)follow->turn;
    right = (int32_t)follow->forward + (int32_t)follow->turn;
    if (left > 450) {
        left = 450;
    } else if (left < -450) {
        left = -450;
    }
    if (right > 450) {
        right = 450;
    } else if (right < -450) {
        right = -450;
    }
    if (left < 0 && right < 0) {
        invalidate_request(request, now_ms);
        return false;
    }
    publish_request((int16_t)left, (int16_t)right, now_ms, request);
    return true;
}

static void set_brake(uint32_t now_ms, MotionRequest *request)
{
    publish_request(0, 0, now_ms, request);
}

static void set_pivot(uint32_t now_ms, MotionRequest *request)
{
    if (corner_direction < 0) {
        publish_request(CORNER_INNER_COMMAND,
                        CORNER_OUTER_COMMAND, now_ms, request);
    } else {
        publish_request(CORNER_OUTER_COMMAND,
                        CORNER_INNER_COMMAND, now_ms, request);
    }
}

static void enter_state(CornerManeuverState next_state, uint32_t now_ms)
{
    state = next_state;
    state_started_ms = now_ms;
}

static void enter_fault(uint32_t now_ms, CornerManeuverOutput *out)
{
    enter_state(CORNER_MANEUVER_FAULT, now_ms);
    invalidate_request(&out->request, now_ms);
    out->owns_motion = true;
    out->completed = false;
    out->fault = true;
}

static bool confirmed_direction(const LinePathEvent *path_event,
                                int8_t *direction)
{
    if (path_event->type == LINE_PATH_RIGHT_ANGLE_LEFT) {
        *direction = -1;
        return true;
    }
    if (path_event->type == LINE_PATH_RIGHT_ANGLE_RIGHT) {
        *direction = 1;
        return true;
    }
    return false;
}

static bool feature_has_reliable_line(const LineFeatures *features)
{
    return features->active_count >= 1U &&
           features->active_count <= 3U &&
           features->confidence >= CORNER_MIN_CONFIDENCE;
}

static bool update_reacquisition(const LineFeatures *features)
{
    if (features->status.sequence == last_feature_sequence) {
        return false;
    }
    last_feature_sequence = features->status.sequence;
    if (feature_has_reliable_line(features)) {
        if (reacquire_frames < CORNER_REACQUIRE_FRAMES) {
            reacquire_frames++;
        }
    } else {
        reacquire_frames = 0U;
    }
    return reacquire_frames >= CORNER_REACQUIRE_FRAMES;
}

static void start_maneuver(int8_t direction, uint32_t now_ms,
                           const LineFeatures *features)
{
    corner_direction = direction;
    reacquire_frames = 0U;
    last_feature_sequence = features->status.sequence;
    maneuver_started_ms = now_ms;
}

void CornerManeuver_Init(void)
{
    state = CORNER_MANEUVER_FOLLOW;
    corner_direction = 0;
    reacquire_frames = 0U;
    last_feature_sequence = 0U;
    maneuver_started_ms = 0U;
    state_started_ms = 0U;
    event_rearm_required = false;
}

void CornerManeuver_Reset(void)
{
    CornerManeuver_Init();
}

CornerManeuverState CornerManeuver_GetState(void)
{
    return state;
}

bool CornerManeuver_Step(const LineFeatures *features,
                         const LinePathEvent *path_event,
                         const LineControlOutput *follow,
                         bool emergency_stop,
                         uint32_t now_ms,
                         CornerManeuverOutput *out)
{
    int8_t detected_direction;

    if (out == 0) {
        return false;
    }
    invalidate_request(&out->request, now_ms);
    out->owns_motion = false;
    out->completed = false;
    out->fault = false;

    if (state == CORNER_MANEUVER_FAULT) {
        enter_fault(now_ms, out);
        return false;
    }
    if (emergency_stop || !features_are_fresh(features, now_ms) ||
        !path_event_is_fresh(path_event, now_ms)) {
        enter_fault(now_ms, out);
        return false;
    }

    if (state != CORNER_MANEUVER_FOLLOW &&
        (uint32_t)(now_ms - maneuver_started_ms) >=
            CORNER_TOTAL_TIMEOUT_MS) {
        enter_fault(now_ms, out);
        return false;
    }

    if (state == CORNER_MANEUVER_FOLLOW) {
        /* A fresh ordinary loss belongs to LineRecovery, not corner control. */
        if (path_event->type == LINE_PATH_LOST) {
            return true;
        }
        if (event_rearm_required) {
            if (path_event->type != LINE_PATH_NORMAL) {
                /* Do not replay a latched corner event or steal a later loss. */
                return true;
            }
            event_rearm_required = false;
        }
        if (confirmed_direction(path_event, &detected_direction)) {
            start_maneuver(detected_direction, now_ms, features);
            enter_state(CORNER_MANEUVER_BRAKE, now_ms);
            out->owns_motion = true;
            set_brake(now_ms, &out->request);
            return true;
        }
        if (path_event->type == LINE_PATH_WIDE_PENDING) {
            start_maneuver(0, now_ms, features);
            enter_state(CORNER_MANEUVER_FORWARD_PROBE, now_ms);
            out->owns_motion = true;
            publish_request(CORNER_PROBE_COMMAND,
                            CORNER_PROBE_COMMAND, now_ms, &out->request);
            return true;
        }
        if (!set_follow_request(follow, now_ms, &out->request)) {
            enter_fault(now_ms, out);
            return false;
        }
        return true;
    }

    out->owns_motion = true;
    if (state == CORNER_MANEUVER_FORWARD_PROBE) {
        if (confirmed_direction(path_event, &detected_direction)) {
            corner_direction = detected_direction;
            enter_state(CORNER_MANEUVER_BRAKE, now_ms);
            set_brake(now_ms, &out->request);
            return true;
        }
        if ((uint32_t)(now_ms - state_started_ms) >= CORNER_PROBE_MAX_MS) {
            enter_fault(now_ms, out);
            return false;
        }
        publish_request(CORNER_PROBE_COMMAND,
                        CORNER_PROBE_COMMAND, now_ms, &out->request);
        return true;
    }

    if (state == CORNER_MANEUVER_BRAKE) {
        if ((uint32_t)(now_ms - state_started_ms) >= CORNER_BRAKE_MS) {
            enter_state(CORNER_MANEUVER_COMMIT, now_ms);
            set_pivot(now_ms, &out->request);
        } else {
            set_brake(now_ms, &out->request);
        }
        return true;
    }

    if (state == CORNER_MANEUVER_COMMIT) {
        if ((uint32_t)(now_ms - state_started_ms) >= CORNER_COMMIT_MS) {
            enter_state(CORNER_MANEUVER_SEEK, now_ms);
        }
        set_pivot(now_ms, &out->request);
        return true;
    }

    if (state == CORNER_MANEUVER_SEEK) {
        if ((uint32_t)(now_ms - state_started_ms) >= CORNER_SEEK_MAX_MS) {
            enter_fault(now_ms, out);
            return false;
        }
        if (update_reacquisition(features)) {
            enter_state(CORNER_MANEUVER_SETTLE, now_ms);
            if (!set_follow_request(follow, now_ms, &out->request)) {
                enter_fault(now_ms, out);
                return false;
            }
            return true;
        }
        set_pivot(now_ms, &out->request);
        return true;
    }

    if (state == CORNER_MANEUVER_SETTLE) {
        /* Abort PD settlement and let normal three-frame loss recovery run. */
        if (path_event->type == LINE_PATH_LOST) {
            enter_state(CORNER_MANEUVER_FOLLOW, now_ms);
            corner_direction = 0;
            reacquire_frames = 0U;
            event_rearm_required = false;
            out->owns_motion = false;
            return true;
        }
        if ((uint32_t)(now_ms - state_started_ms) >= CORNER_SETTLE_MS) {
            if (!set_follow_request(follow, now_ms, &out->request)) {
                enter_fault(now_ms, out);
                return false;
            }
            enter_state(CORNER_MANEUVER_FOLLOW, now_ms);
            corner_direction = 0;
            reacquire_frames = 0U;
            event_rearm_required = true;
            out->completed = true;
            return true;
        }
        if (!set_follow_request(follow, now_ms, &out->request)) {
            enter_fault(now_ms, out);
            return false;
        }
        return true;
    }

    enter_fault(now_ms, out);
    return false;
}
