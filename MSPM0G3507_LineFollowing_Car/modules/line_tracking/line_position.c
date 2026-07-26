#include "line_position.h"

#include <stdbool.h>

#include "../../application/config/line_lookup_config.h"

/*
 * Signed position for each of the 15 legal sensor patterns.
 * Zero-initialized entries collide with the legal center value (0x18),
 * so legality is decided by pattern shape first, never by this table.
 */
static const int8_t position_by_bits[256] = {
    [0x01] = -7, [0x03] = -6, [0x02] = -5, [0x06] = -4,
    [0x04] = -3, [0x0C] = -2, [0x08] = -1, [0x18] = 0,
    [0x10] = 1,  [0x30] = 2,  [0x20] = 3,  [0x60] = 4,
    [0x40] = 5,  [0xC0] = 6,  [0x80] = 7
};

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

/* Classify the raw bitmap by run shape: 0 runs, 1 run of 1-2 (legal),
 * 1 run of 3+ (wide), or more than 1 separated run (noise). */
static LinePatternType ClassifyBits(uint8_t bits)
{
    uint8_t run_count = 0U;
    uint8_t longest_run = 0U;
    uint8_t current_run = 0U;
    uint8_t index;

    if (bits == 0x00U) {
        return LINE_PATTERN_LOST;
    }

    for (index = 0U; index < 8U; index++) {
        if ((bits & (uint8_t)(1U << index)) != 0U) {
            if (current_run == 0U) {
                run_count++;
            }
            current_run++;
            if (current_run > longest_run) {
                longest_run = current_run;
            }
        } else {
            current_run = 0U;
        }
    }

    if (run_count > 1U) {
        return LINE_PATTERN_NOISE;
    }
    if (longest_run >= 3U) {
        return LINE_PATTERN_WIDE;
    }
    return LINE_PATTERN_POSITION;
}

LinePositionResult LinePosition_Update(uint8_t black_bits)
{
    LinePositionResult result;
    LinePatternType type = ClassifyBits(black_bits);

    if (type != LINE_PATTERN_POSITION) {
        /* Illegal frame: hold the stable value and restart debouncing. */
        candidate_position = stable_position;
        candidate_frames = 0U;
    } else {
        int decoded = (int)position_by_bits[black_bits];
        int held = (int)stable_position;
        int delta = decoded - held;

        if (decoded == (int)candidate_position && candidate_frames > 0U) {
            if (candidate_frames < 0xFFU) {
                candidate_frames++;
            }
        } else {
            candidate_position = (int8_t)decoded;
            candidate_frames = 1U;
        }

        if (delta < 0) {
            delta = -delta;
        }

        if (!has_stable ||
            delta <= LINE_POSITION_ADJACENT_STEP ||
            candidate_frames >= LINE_POSITION_JUMP_ACCEPT_FRAMES) {
            stable_position = (int8_t)decoded;
            has_stable = true;
        }
    }

    result.type = type;
    result.stable_position = stable_position;
    result.candidate_position = candidate_position;
    result.candidate_frames = candidate_frames;
    result.black_bits = black_bits;
    return result;
}
