#include "line_features.h"

#include "line_feature_config.h"

static const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
static float previous_error;
static uint32_t previous_timestamp_ms;
static uint16_t previous_sequence;
static bool has_history;

static float absolute_value(float value)
{
    return value < 0.0f ? -value : value;
}

static uint8_t clamp_confidence(int16_t value)
{
    if (value < 0) {
        return 0U;
    }
    if (value > 100) {
        return 100U;
    }
    return (uint8_t)value;
}

void LineFeatureExtractor_Reset(void)
{
    previous_error = 0.0f;
    previous_timestamp_ms = 0U;
    previous_sequence = 0U;
    has_history = false;
}

void LineFeatureExtractor_Init(void)
{
    LineFeatureExtractor_Reset();
}

bool LineFeatureExtractor_Update(const LineSensorSnapshot *snapshot,
                                 uint32_t now_ms,
                                 LineFeatures *out)
{
    int16_t weighted_sum = 0;
    int16_t confidence = 100;
    uint8_t first_index = 0U;
    uint8_t last_index = 0U;
    bool previous_active = false;
    uint8_t index;

    if (snapshot == 0 || out == 0 ||
        !ModuleStatus_IsFresh(&snapshot->status, now_ms,
                              LINE_FEATURE_STALE_MS)) {
        return false;
    }

    out->status = snapshot->status;
    out->black_bits = snapshot->black_bits;
    out->active_count = 0U;
    out->left_count = 0U;
    out->right_count = 0U;
    out->span = 0U;
    out->segment_count = 0U;
    out->left_edge = false;
    out->right_edge = false;

    for (index = 0U; index < 8U; index++) {
        bool active = (snapshot->black_bits & (uint8_t)(1U << index)) != 0U;

        if (active) {
            if (!previous_active) {
                out->segment_count++;
            }
            if (out->active_count == 0U) {
                first_index = index;
            }
            last_index = index;
            out->active_count++;
            weighted_sum += weights[index];
            if (index < 4U) {
                out->left_count++;
            } else {
                out->right_count++;
            }
        }
        if (index == 0U) {
            out->left_edge = active;
        } else if (index == 7U) {
            out->right_edge = active;
        }
        previous_active = active;
    }

    if (out->active_count != 0U) {
        out->span = last_index - first_index + 1U;
        out->centroid_error =
            (float)weighted_sum / (float)out->active_count;
    } else {
        out->centroid_error = has_history ? previous_error : 0.0f;
    }
    /* 序号和时间都推进才计算导数，避免重复帧放大瞬时误差。 */
    out->error_rate = 0.0f;
    if (has_history &&
        snapshot->status.sequence != previous_sequence &&
        snapshot->status.timestamp_ms != previous_timestamp_ms) {
        uint32_t delta_ms =
            snapshot->status.timestamp_ms - previous_timestamp_ms;
        out->error_rate =
            (out->centroid_error - previous_error) * 1000.0f /
            (float)delta_ms;
    }

    /* 多段、过宽或质心突变均降低置信度，并限制在 0 到 100。 */
    if (out->segment_count > 1U) {
        confidence -= (int16_t)(out->segment_count - 1U) *
                      LINE_FEATURE_GROUP_PENALTY;
    }
    if (out->active_count > 3U) {
        confidence -= (int16_t)(out->active_count - 3U) *
                      LINE_FEATURE_WIDE_PENALTY;
    }
    if (has_history &&
        absolute_value(out->centroid_error - previous_error) >
            LINE_FEATURE_JUMP_ERROR) {
        confidence -= LINE_FEATURE_JUMP_PENALTY;
    }
    out->confidence = clamp_confidence(confidence);

    /* 仅记录新的非空帧；丢线与重复帧保留最近可靠质心。 */
    if (out->active_count != 0U &&
        (!has_history ||
         (snapshot->status.sequence != previous_sequence &&
          snapshot->status.timestamp_ms != previous_timestamp_ms))) {
        previous_error = out->centroid_error;
        previous_timestamp_ms = snapshot->status.timestamp_ms;
        previous_sequence = snapshot->status.sequence;
        has_history = true;
    }
    return true;
}
