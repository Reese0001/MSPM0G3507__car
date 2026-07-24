#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "modules/line_tracking/line_controller.h"
#include "modules/line_tracking/line_trend_detector.h"

static LineEstimate estimate(uint32_t sequence, float error, LineEvent event)
{
    LineEstimate value = {0};

    value.status.timestamp_ms = sequence;
    value.status.sequence = (uint16_t)sequence;
    value.status.valid = true;
    value.status.health = MODULE_HEALTH_OK;
    value.error = error;
    value.predicted_error = error;
    value.confidence = 100U;
    value.event = event;
    return value;
}

static LineSensorSnapshot snapshot(uint32_t sequence)
{
    LineSensorSnapshot value = {0};

    value.status.timestamp_ms = sequence;
    value.status.sequence = (uint16_t)sequence;
    value.status.valid = true;
    value.status.health = MODULE_HEALTH_OK;
    value.black_bits = 0x01U;
    return value;
}

static LineTrendResult normal_trend(uint32_t sequence)
{
    LineTrendResult value = {0};

    value.status.timestamp_ms = sequence;
    value.status.sequence = (uint16_t)sequence;
    value.status.valid = true;
    value.status.health = MODULE_HEALTH_OK;
    value.type = LINE_TREND_NORMAL;
    return value;
}

static void duplicate_frame_does_not_create_hairpin(void)
{
    LineTrendResult result = {0};
    LineEstimate line;
    LineSensorSnapshot raw;
    uint32_t sequence;

    LineTrendDetector_Init();
    for (sequence = 1U; sequence <= 3U; sequence++) {
        float error = sequence == 1U ? 1.0f :
                      sequence == 2U ? 3.0f : 7.0f;
        LineEvent event = sequence == 3U ?
                          LINE_EVENT_HARD_RIGHT : LINE_EVENT_NONE;
        line = estimate(sequence, error, event);
        raw = snapshot(sequence);
        assert(LineTrendDetector_Update(&line, &raw, sequence, &result));
    }
    assert(result.type == LINE_TREND_TIGHT_RIGHT);

    assert(LineTrendDetector_Update(&line, &raw, 4U, &result));
    assert(LineTrendDetector_Update(&line, &raw, 5U, &result));
    assert(result.type == LINE_TREND_TIGHT_RIGHT);
}

static void duplicate_frame_does_not_enable_straight_boost(void)
{
    const LineControlConfig config = {
        300, 180, 140, 100, 100, 100, 80, 200,
        100, 100, 50, 120, 60U, 300, 300, 300,
        1.0f, 0.0f, 1.0f, 1.0f, 3.0f, 6.0f, 100.0f,
        20U, 40U, 3U, 20U
    };
    LineControlOutput output = {0};
    LineEstimate line = estimate(1U, 0.0f, LINE_EVENT_NONE);
    LineTrendResult trend = normal_trend(1U);

    assert(LineController_Init(&config));
    assert(LineController_Step(&line, &trend, 0.0f, false, 1U, &output));
    assert(output.forward == config.cruise_forward);
    assert(LineController_Step(&line, &trend, 0.0f, false, 2U, &output));
    assert(LineController_Step(&line, &trend, 0.0f, false, 3U, &output));
    assert(output.forward == config.cruise_forward);

    line = estimate(2U, 0.0f, LINE_EVENT_NONE);
    trend = normal_trend(2U);
    assert(LineController_Step(&line, &trend, 0.0f, false, 4U, &output));
    assert(output.forward == config.cruise_forward);
    line = estimate(3U, 0.0f, LINE_EVENT_NONE);
    trend = normal_trend(3U);
    assert(LineController_Step(&line, &trend, 0.0f, false, 5U, &output));
    assert(output.forward == config.max_forward);
}

int main(void)
{
    duplicate_frame_does_not_create_hairpin();
    duplicate_frame_does_not_enable_straight_boost();
    return 0;
}
