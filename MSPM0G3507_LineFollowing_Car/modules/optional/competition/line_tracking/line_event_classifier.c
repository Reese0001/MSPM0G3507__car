#include "line_event_classifier.h"

#include "line_event_config.h"

static uint8_t stable_single_frames;
static uint8_t wide_frames;
static uint16_t previous_sequence;
static int16_t direction_score;
static bool corner_candidate;
static LinePathEventType latched_corner;
static LinePathEvent previous_event;
static bool has_previous_event;

static int16_t frame_direction_score(const LineFeatures *features)
{
    int16_t score =
        (int16_t)(features->right_count - features->left_count) *
        LINE_EVENT_SIDE_COUNT_WEIGHT;

    if (features->right_edge) {
        score += LINE_EVENT_EDGE_WEIGHT;
    }
    if (features->left_edge) {
        score -= LINE_EVENT_EDGE_WEIGHT;
    }
    return score;
}

static uint8_t direction_confidence(void)
{
    uint16_t magnitude = direction_score < 0 ?
        (uint16_t)(-direction_score) : (uint16_t)direction_score;

    return magnitude >= 100U ? 100U : (uint8_t)magnitude;
}

static int8_t score_direction(void)
{
    if (direction_score < 0) {
        return -1;
    }
    if (direction_score > 0) {
        return 1;
    }
    return 0;
}

static void publish_invalid(LinePathEvent *out, uint32_t now_ms)
{
    if (out == 0) {
        return;
    }

    out->status.timestamp_ms = now_ms;
    out->status.sequence = 0U;
    out->status.valid = false;
    out->status.health = MODULE_HEALTH_FAULT;
    out->type = LINE_PATH_INVALID;
    out->direction = 0;
    out->direction_confidence = 0U;
}

static void publish_event(const LineFeatures *features,
                          LinePathEventType type,
                          LinePathEvent *out)
{
    out->status = features->status;
    out->type = type;
    out->direction = (type == LINE_PATH_RIGHT_ANGLE_LEFT) ? -1 :
                     (type == LINE_PATH_RIGHT_ANGLE_RIGHT) ? 1 :
                     (type == LINE_PATH_WIDE_PENDING) ? score_direction() : 0;
    out->direction_confidence = direction_confidence();
    previous_event = *out;
    has_previous_event = true;
}

void LineEventClassifier_Reset(void)
{
    stable_single_frames = 0U;
    wide_frames = 0U;
    previous_sequence = 0U;
    direction_score = 0;
    corner_candidate = false;
    latched_corner = LINE_PATH_NORMAL;
    previous_event = (LinePathEvent){0};
    has_previous_event = false;
}

void LineEventClassifier_Init(void)
{
    LineEventClassifier_Reset();
}

bool LineEventClassifier_Update(const LineFeatures *features,
                                const LineEstimate *estimate,
                                const LineTrendResult *trend,
                                uint32_t now_ms,
                                LinePathEvent *out)
{
    LinePathEventType type = LINE_PATH_NORMAL;

    if (features == 0 || estimate == 0 || trend == 0 || out == 0 ||
        !ModuleStatus_IsFresh(&features->status, now_ms,
                              LINE_EVENT_STALE_MS) ||
        !ModuleStatus_IsFresh(&estimate->status, now_ms,
                              LINE_EVENT_STALE_MS) ||
        !ModuleStatus_IsFresh(&trend->status, now_ms,
                              LINE_EVENT_STALE_MS)) {
        publish_invalid(out, now_ms);
        return false;
    }

    if (has_previous_event &&
        features->status.sequence == previous_sequence) {
        *out = previous_event;
        return true;
    }
    previous_sequence = features->status.sequence;

    if (latched_corner != LINE_PATH_NORMAL) {
        publish_event(features, latched_corner, out);
        return true;
    }

    if (corner_candidate) {
        direction_score += frame_direction_score(features);
        if (direction_score <= -LINE_EVENT_DIRECTION_THRESHOLD) {
            latched_corner = LINE_PATH_RIGHT_ANGLE_LEFT;
            type = latched_corner;
        } else if (direction_score >= LINE_EVENT_DIRECTION_THRESHOLD) {
            latched_corner = LINE_PATH_RIGHT_ANGLE_RIGHT;
            type = latched_corner;
        } else {
            type = LINE_PATH_WIDE_PENDING;
        }
        publish_event(features, type, out);
        return true;
    }

    if (features->active_count >= 1U && features->active_count <= 3U &&
        features->confidence >= LINE_EVENT_MIN_CONFIDENCE) {
        if (stable_single_frames < UINT8_MAX) {
            stable_single_frames++;
        }
        wide_frames = 0U;
    } else if (features->active_count >= LINE_EVENT_WIDE_ACTIVE_COUNT &&
               features->span >= LINE_EVENT_WIDE_MIN_SPAN &&
               stable_single_frames >= LINE_EVENT_STABLE_SINGLE_FRAMES) {
        if (wide_frames < UINT8_MAX) {
            wide_frames++;
        }
        if (wide_frames >= LINE_EVENT_WIDE_CONFIRM_FRAMES) {
            corner_candidate = true;
            type = LINE_PATH_WIDE_PENDING;
        }
    } else {
        stable_single_frames = 0U;
        wide_frames = 0U;
    }

    if (type == LINE_PATH_NORMAL && features->active_count == 0U) {
        type = LINE_PATH_LOST;
    }
    publish_event(features, type, out);
    return true;
}
