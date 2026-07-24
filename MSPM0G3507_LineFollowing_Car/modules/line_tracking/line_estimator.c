#include "line_estimator.h"

#include "line_feature_config.h"
#include "line_tracking_config.h"

static LineEstimate latest_estimate = {0};

static float clamp_error(float value)
{
    if (value < -7.0f) {
        return -7.0f;
    }
    if (value > 7.0f) {
        return 7.0f;
    }
    return value;
}

static void publish_fault(uint32_t now_ms)
{
    latest_estimate.status.timestamp_ms = now_ms;
    latest_estimate.status.sequence++;
    latest_estimate.status.valid = false;
    latest_estimate.status.health = MODULE_HEALTH_FAULT;
    latest_estimate.confidence = 0U;
    latest_estimate.event = LINE_EVENT_LOST;
}

void LineEstimator_Init(void)
{
    latest_estimate.status.timestamp_ms = 0U;
    latest_estimate.status.sequence = 0U;
    latest_estimate.status.valid = false;
    latest_estimate.status.health = MODULE_HEALTH_UNKNOWN;
    latest_estimate.error = 0.0f;
    latest_estimate.derivative = 0.0f;
    latest_estimate.predicted_error = 0.0f;
    latest_estimate.confidence = 0U;
    latest_estimate.event = LINE_EVENT_LOST;
}

bool LineEstimator_Update(const LineFeatures *features, uint32_t now_ms)
{
    if (features == 0 ||
        !ModuleStatus_IsFresh(&features->status, now_ms,
                              LINE_FEATURE_STALE_MS)) {
        publish_fault(now_ms);
        return false;
    }

    latest_estimate.status = features->status;
    latest_estimate.status.sequence++;
    latest_estimate.error = features->centroid_error;
    latest_estimate.derivative = features->error_rate;
    latest_estimate.predicted_error = clamp_error(
        features->centroid_error +
        features->error_rate * LINE_PREDICTION_HORIZON_S);
    latest_estimate.confidence = features->confidence;

    if (features->active_count == 0U) {
        latest_estimate.status.valid = true;
        latest_estimate.status.health = MODULE_HEALTH_DEGRADED;
        latest_estimate.event = LINE_EVENT_LOST;
        return true;
    }

    latest_estimate.status.valid = true;
    latest_estimate.event = LINE_EVENT_NONE;
    if (features->active_count >= 6U) {
        latest_estimate.event = LINE_EVENT_WIDE_BLACK;
        if (latest_estimate.confidence > 30U) {
            latest_estimate.confidence = 30U;
        }
    } else if (features->centroid_error <= -4.0f) {
        latest_estimate.event = LINE_EVENT_HARD_LEFT;
    } else if (features->centroid_error >= 4.0f) {
        latest_estimate.event = LINE_EVENT_HARD_RIGHT;
    }
    return true;
}

bool LineEstimator_Get(LineEstimate *out)
{
    if (out == 0 || !latest_estimate.status.valid) {
        return false;
    }
    *out = latest_estimate;
    return true;
}
