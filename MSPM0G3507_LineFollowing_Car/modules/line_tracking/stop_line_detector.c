#include "stop_line_detector.h"

#include "line_tracking_config.h"

static bool started;
static bool departed;
static bool in_marker;
static uint8_t leave_frames;
static uint8_t marker_frames;
static uint32_t start_ms;
static uint32_t first_marker_distance;
static uint32_t last_marker_distance;

static uint8_t bit_count(uint8_t bits)
{
    uint8_t count = 0U;

    bits &= 0x0FU;
    while (bits != 0U) {
        count = (uint8_t)(count + (bits & 1U));
        bits >>= 1U;
    }
    return count;
}

static bool marker_bits(uint8_t bits)
{
    return bit_count(bits) >= STOP_LINE_MIN_BITS;
}

void StopLineDetector_Init(void)
{
    started = false;
    departed = false;
    in_marker = false;
    leave_frames = 0U;
    marker_frames = 0U;
    start_ms = 0U;
    first_marker_distance = 0U;
    last_marker_distance = 0U;
}

void StopLineDetector_Start(uint32_t now_ms, uint32_t distance_mm)
{
    started = true;
    departed = false;
    in_marker = false;
    leave_frames = 0U;
    marker_frames = 0U;
    start_ms = now_ms;
    first_marker_distance = distance_mm;
    last_marker_distance = distance_mm;
}

StopLineResult StopLineDetector_Update(uint8_t black_bits,
                                       uint32_t distance_mm,
                                       uint32_t now_ms)
{
    StopLineResult result = {0};
    bool marker = marker_bits(black_bits);

    result.black_bits = (uint8_t)(black_bits & 0x0FU);
    result.departed = departed;
    if (!started) {
        return result;
    }

    if (!departed) {
        if (marker) {
            leave_frames = 0U;
        } else if (++leave_frames >= STOP_LINE_LEAVE_FRAMES) {
            departed = true;
        }
        result.departed = departed;
        return result;
    }

    if ((uint32_t)(now_ms - start_ms) < STOP_LINE_MIN_ELAPSED_MS ||
        distance_mm < STOP_LINE_MIN_DISTANCE_MM) {
        return result;
    }

    if (marker) {
        if (!in_marker) {
            in_marker = true;
            marker_frames = 1U;
            first_marker_distance = distance_mm;
        } else if (marker_frames < 0xFFU) {
            marker_frames++;
        }
        last_marker_distance = distance_mm;
        result.candidate = true;
        result.candidate_frames = marker_frames;
        return result;
    }

    if (in_marker) {
        if (marker_frames >= STOP_LINE_MIN_FRAMES) {
            result.stop_event = true;
            result.center_distance_mm =
                first_marker_distance +
                (last_marker_distance - first_marker_distance) / 2U;
        }
        in_marker = false;
        marker_frames = 0U;
    }
    result.departed = departed;
    return result;
}
