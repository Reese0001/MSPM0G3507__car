#include "line_trend_detector.h"

#include "../../../line_tracking/line_tracking_config.h"
#include "line_trend_config.h"

static int8_t trend_direction;
static uint8_t same_direction_frames;
static uint8_t outward_steps;
static uint8_t hairpin_frames;
static uint8_t stable_reacquire_frames;
static float previous_error;
static float maximum_absolute_error;
static uint16_t last_sequence;
static bool sequence_seen;
static LineTrendType last_type;

static float absolute_error(float value)
{
    return value < 0.0f ? -value : value;
}

static uint8_t count_active_bits(uint8_t bits)
{
    uint8_t count = 0U;

    while (bits != 0U) {
        count += (uint8_t)(bits & 1U);
        bits >>= 1U;
    }
    return count;
}

static int8_t direction_from_error(float error)
{
    if (error < 0.0f) {
        return -1;
    }
    if (error > 0.0f) {
        return 1;
    }
    return 0;
}

static void clear_direction_evidence(void)
{
    same_direction_frames = 0U;
    outward_steps = 0U;
    hairpin_frames = 0U;
    stable_reacquire_frames = 0U;
    maximum_absolute_error = 0.0f;
}

static void publish_invalid(LineTrendResult *result, uint32_t now_ms)
{
    if (result == 0) {
        return;
    }

    result->status.timestamp_ms = now_ms;
    result->status.sequence = 0U;
    result->status.valid = false;
    result->status.health = MODULE_HEALTH_FAULT;
    result->type = LINE_TREND_NORMAL;
    result->direction = 0;
}

static void publish_result(LineTrendResult *result,
                           const LineEstimate *estimate,
                           uint32_t now_ms,
                           LineTrendType type)
{
    result->status.timestamp_ms = now_ms;
    result->status.sequence = estimate->status.sequence;
    result->status.valid = true;
    result->status.health = MODULE_HEALTH_OK;
    result->type = type;
    result->direction = trend_direction;
}

void LineTrendDetector_Init(void)
{
    LineTrendDetector_Reset();
}

void LineTrendDetector_Reset(void)
{
    trend_direction = 0;
    previous_error = 0.0f;
    last_sequence = 0U;
    sequence_seen = false;
    last_type = LINE_TREND_NORMAL;
    clear_direction_evidence();
}

bool LineTrendDetector_Update(const LineEstimate *estimate,
                              const LineSensorSnapshot *snapshot,
                              uint32_t now_ms,
                              LineTrendResult *result)
{
    int8_t sample_direction;
    float current_absolute_error;
    LineTrendType type = LINE_TREND_NORMAL;
    bool completion_event;
    uint8_t active_count;

    if (estimate == 0 || snapshot == 0 || result == 0 ||
        !ModuleStatus_IsFresh(&estimate->status, now_ms, LINE_SENSOR_STALE_MS) ||
        !ModuleStatus_IsFresh(&snapshot->status, now_ms, LINE_SENSOR_STALE_MS)) {
        publish_invalid(result, now_ms);
        return false;
    }

    active_count = count_active_bits(snapshot->black_bits);
    completion_event =
        estimate->event == LINE_EVENT_LOST ||
        estimate->event == LINE_EVENT_WIDE_BLACK ||
        active_count >= LINE_TREND_CROSSLINE_ACTIVE_COUNT;
    if (estimate->confidence < LINE_TREND_MIN_CONFIDENCE &&
        !completion_event) {
        publish_invalid(result, now_ms);
        return false;
    }
    if (sequence_seen && estimate->status.sequence == last_sequence) {
        publish_result(result, estimate, now_ms, last_type);
        return true;
    }

    sample_direction = direction_from_error(estimate->error);
    current_absolute_error = absolute_error(estimate->error);
    if (sample_direction != 0) {
        if (trend_direction != 0 && sample_direction != trend_direction) {
            clear_direction_evidence();
        }

        if (sample_direction != trend_direction) {
            trend_direction = sample_direction;
            same_direction_frames = 1U;
        } else {
            if (same_direction_frames < UINT8_MAX) {
                same_direction_frames++;
            }
            if (current_absolute_error > absolute_error(previous_error)) {
                if (outward_steps < UINT8_MAX) {
                    outward_steps++;
                }
            } else if (current_absolute_error < absolute_error(previous_error)) {
                outward_steps = 0U;
                hairpin_frames = 0U;
            }
        }

        if (current_absolute_error > maximum_absolute_error) {
            maximum_absolute_error = current_absolute_error;
        }
    }

    if (trend_direction != 0 && outward_steps >= LINE_TREND_OUTWARD_STEPS) {
        if (estimate->event == LINE_EVENT_HARD_LEFT ||
            estimate->event == LINE_EVENT_HARD_RIGHT) {
            if (current_absolute_error >= LINE_TREND_HAIRPIN_ERROR) {
                if (hairpin_frames < UINT8_MAX) {
                    hairpin_frames++;
                }
                if (hairpin_frames >= LINE_TREND_HAIRPIN_FRAMES) {
                    type = trend_direction < 0 ? LINE_TREND_HAIRPIN_LEFT :
                                                 LINE_TREND_HAIRPIN_RIGHT;
                }
            }
        } else {
            hairpin_frames = 0U;
        }

        if (type == LINE_TREND_NORMAL &&
            current_absolute_error >= LINE_TREND_TIGHT_ERROR) {
            type = trend_direction < 0 ? LINE_TREND_TIGHT_LEFT :
                                         LINE_TREND_TIGHT_RIGHT;
        }
    }

    if (type == LINE_TREND_NORMAL && !completion_event &&
        estimate->confidence >= LINE_TREND_MIN_CONFIDENCE &&
        current_absolute_error <= LINE_TREND_TIGHT_ERROR) {
        if (stable_reacquire_frames < UINT8_MAX) {
            stable_reacquire_frames++;
        }
    } else {
        stable_reacquire_frames = 0U;
    }
    if (stable_reacquire_frames >= LINE_TREND_REACQUIRE_FRAMES) {
        clear_direction_evidence();
        trend_direction = 0;
    }

    previous_error = estimate->error;
    last_sequence = estimate->status.sequence;
    sequence_seen = true;
    last_type = type;
    publish_result(result, estimate, now_ms, type);
    return true;
}
