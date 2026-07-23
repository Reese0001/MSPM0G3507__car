#include "line_estimator.h"

#include "line_tracking_config.h"

static const int8_t line_weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
static LineEstimate latest_estimate = {0};
static float previous_error = 0.0f;
static uint32_t previous_timestamp_ms = 0U;
static bool has_history = false;

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

static float absolute_value(float value)
{
    return value < 0.0f ? -value : value;
}

static uint8_t pattern_confidence(uint8_t bits, uint8_t active_count,
                                  float error)
{
    uint8_t groups = 0U;
    uint8_t previous_active = 0U;
    int16_t confidence = 100;
    uint8_t index;

    for (index = 0U; index < 8U; index++) {
        uint8_t active = (uint8_t)((bits >> index) & 0x01U);
        if (active != 0U && previous_active == 0U) {
            groups++;
        }
        previous_active = active;
    }
    if (groups > 1U) {
        confidence -= (int16_t)(groups - 1U) * 20;
    }
    if (active_count > 3U) {
        confidence -= (int16_t)(active_count - 3U) * 10;
    }
    if (has_history && absolute_value(error - previous_error) > 3.0f) {
        confidence -= 15;
    }
    if (confidence < 0) {
        return 0U;
    }
    return (uint8_t)confidence;
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
    previous_error = 0.0f;
    previous_timestamp_ms = 0U;
    has_history = false;
}

bool LineEstimator_Update(const LineSensorSnapshot *snapshot, uint32_t now_ms)
{
    int16_t weighted_sum = 0;
    uint8_t active_count = 0U;
    uint8_t index;
    float error;
    float derivative = 0.0f;

    if (snapshot == 0 ||
        !ModuleStatus_IsFresh(&snapshot->status, now_ms,
                              LINE_SENSOR_STALE_MS)) {
        publish_fault(now_ms);
        return false;
    }
    if (snapshot->black_bits == 0U) {
        latest_estimate.status.timestamp_ms = snapshot->status.timestamp_ms;
        latest_estimate.status.sequence++;
        latest_estimate.status.valid = true;
        latest_estimate.status.health = MODULE_HEALTH_DEGRADED;
        latest_estimate.error = has_history ? previous_error : 0.0f;
        latest_estimate.derivative = 0.0f;
        latest_estimate.predicted_error = latest_estimate.error;
        latest_estimate.confidence = 0U;
        latest_estimate.event = LINE_EVENT_LOST;
        return true;
    }

    for (index = 0U; index < 8U; index++) {
        if ((snapshot->black_bits & (uint8_t)(1U << index)) != 0U) {
            weighted_sum += line_weights[index];
            active_count++;
        }
    }
    error = (float)weighted_sum / (float)active_count;
    if (has_history && snapshot->status.timestamp_ms != previous_timestamp_ms) {
        uint32_t delta_ms = snapshot->status.timestamp_ms - previous_timestamp_ms;
        derivative = (error - previous_error) * 1000.0f / (float)delta_ms;
    }

    latest_estimate.status.timestamp_ms = snapshot->status.timestamp_ms;
    latest_estimate.status.sequence++;
    latest_estimate.status.valid = true;
    latest_estimate.status.health = MODULE_HEALTH_OK;
    latest_estimate.error = error;
    latest_estimate.derivative = derivative;
    latest_estimate.predicted_error = clamp_error(
        error + derivative * LINE_PREDICTION_HORIZON_S);
    latest_estimate.confidence = pattern_confidence(
        snapshot->black_bits, active_count, error);
    latest_estimate.event = LINE_EVENT_NONE;
    if (active_count >= 6U) {
        latest_estimate.event = LINE_EVENT_WIDE_BLACK;
        if (latest_estimate.confidence > 30U) {
            latest_estimate.confidence = 30U;
        }
    } else if (error <= -4.0f) {
        latest_estimate.event = LINE_EVENT_HARD_LEFT;
    } else if (error >= 4.0f) {
        latest_estimate.event = LINE_EVENT_HARD_RIGHT;
    }

    previous_error = error;
    previous_timestamp_ms = snapshot->status.timestamp_ms;
    has_history = true;
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
