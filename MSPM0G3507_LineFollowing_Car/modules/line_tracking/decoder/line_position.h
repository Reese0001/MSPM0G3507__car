#ifndef LINE_POSITION_H
#define LINE_POSITION_H

#include <stdint.h>

/*
 * Decode the eight-channel black-line bitmap into one of the 15 legal
 * positions (-7..+7) and debounce large jumps. Bit 0 = sensor X1
 * (right side when looking forward), bit 7 = sensor X8 (left side).
 */

typedef enum {
    LINE_PATTERN_LOST = 0,   /* no sensor sees the line            */
    LINE_PATTERN_POSITION,   /* 1 sensor or 2 adjacent sensors     */
    LINE_PATTERN_WIDE,       /* one contiguous run of 3+ sensors   */
    LINE_PATTERN_NOISE       /* separated runs (illegal pattern)   */
} LinePatternType;

typedef struct {
    LinePatternType type;    /* classification of the current frame     */
    int8_t stable_position;  /* debounced position, -7..+7              */
    int8_t candidate_position; /* latest decoded position               */
    uint8_t candidate_frames;  /* consecutive frames with this candidate */
    uint8_t black_bits;      /* raw bitmap of the current frame         */
} LinePositionResult;

void LinePosition_Reset(void);
LinePositionResult LinePosition_Update(uint8_t black_bits);

#endif
