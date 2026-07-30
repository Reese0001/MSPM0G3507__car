#include "line_position.h"

#include <stdbool.h>

#include "../line_tracking_config.h"

static const int8_t position_by_bits[16] = {
    [0x01] = -3, [0x03] = -2, [0x02] = -1,
    [0x06] = 0,  [0x04] = 1,  [0x0C] = 2,  [0x08] = 3
};
static const int8_t sensor_position[4] = {-3, -1, 1, 3};

static bool has_stable;
static int8_t stable_position;
static int8_t candidate_position;
static uint8_t candidate_frames;

void LinePosition_Reset(void)
{
    has_stable = false;
    stable_position = 0;
    candidate_position = 0;
    candidate_frames = 0U;
}

static LinePatternType classify_bits(uint8_t bits)
{
    uint8_t runs = 0U;
    uint8_t longest = 0U;
    uint8_t current = 0U;
    uint8_t index;

    if (bits == 0U) {
        return LINE_PATTERN_LOST;
    }
    for (index = 0U; index < 4U; index++) {
        if ((bits & (uint8_t)(1U << index)) != 0U) {
            if (current == 0U) {
                runs++;
            }
            current++;
            if (current > longest) {
                longest = current;
            }
        } else {
            current = 0U;
        }
    }
    if (runs > 1U) {
        return LINE_PATTERN_NOISE;
    }
    return longest >= 3U ? LINE_PATTERN_WIDE : LINE_PATTERN_POSITION;
}

static int8_t wide_position(uint8_t bits)
{
    int16_t sum = 0;
    uint8_t count = 0U;
    uint8_t index;

    for (index = 0U; index < 4U; index++) {
        if ((bits & (uint8_t)(1U << index)) != 0U) {
            sum += sensor_position[index];
            count++;
        }
    }
    return count == 0U ? 0 : (int8_t)(sum / (int16_t)count);
}

static float weighted_error(uint8_t bits)
{
    int16_t sum = 0;
    uint8_t count = 0U;
    uint8_t index;

    for (index = 0U; index < 4U; index++) {
        if ((bits & (uint8_t)(1U << index)) != 0U) {
            sum += sensor_position[index];
            count++;
        }
    }
    return count == 0U ? 0.0f : (float)sum / (float)count;
}

static uint8_t confidence_for(uint8_t bits, LinePatternType type)
{
    if (type == LINE_PATTERN_WIDE) {
        return 45U;
    }
    switch (bits) {
    case 0x06U:
        return 100U;
    case 0x02U:
    case 0x04U:
        return 90U;
    case 0x03U:
    case 0x0CU:
        return 80U;
    case 0x01U:
    case 0x08U:
        return 55U;
    default:
        return 0U;
    }
}

LinePositionResult LinePosition_Update(uint8_t black_bits)
{
    LinePositionResult result = {0};
    uint8_t bits = black_bits & 0x0FU;
    LinePatternType type = classify_bits(bits);

    if (type == LINE_PATTERN_WIDE) {
        candidate_position = wide_position(bits);
        candidate_frames = 1U;
    } else if (type != LINE_PATTERN_POSITION) {
        candidate_position = stable_position;
        candidate_frames = 0U;
    } else {
        int8_t decoded = position_by_bits[bits];
        int delta = (int)decoded - (int)stable_position;

        if (decoded == candidate_position && candidate_frames != 0U) {
            if (candidate_frames != 0xFFU) {
                candidate_frames++;
            }
        } else {
            candidate_position = decoded;
            candidate_frames = 1U;
        }
        if (delta < 0) {
            delta = -delta;
        }
        if (!has_stable || delta <= LINE_POSITION_ADJACENT_STEP ||
            candidate_frames >= LINE_POSITION_JUMP_ACCEPT_FRAMES) {
            stable_position = decoded;
            has_stable = true;
        }
    }

    result.type = type;
    result.stable_position = stable_position;
    result.candidate_position = candidate_position;
    result.candidate_frames = candidate_frames;
    result.black_bits = bits;
    result.weighted_error = type == LINE_PATTERN_POSITION ||
                            type == LINE_PATTERN_WIDE ?
                                weighted_error(bits) :
                                (float)stable_position;
    result.confidence = confidence_for(bits, type);
    result.reliable = type == LINE_PATTERN_POSITION ||
                      type == LINE_PATTERN_WIDE;
    return result;
}
