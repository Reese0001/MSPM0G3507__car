#ifndef LINE_POSITION_H
#define LINE_POSITION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LINE_PATTERN_LOST = 0,
    LINE_PATTERN_POSITION,
    LINE_PATTERN_WIDE,
    LINE_PATTERN_NOISE
} LinePatternType;

typedef struct {
    LinePatternType type;
    int8_t stable_position;
    int8_t candidate_position;
    uint8_t candidate_frames;
    uint8_t black_bits;
    float weighted_error;
    uint8_t confidence;
    bool reliable;
} LinePositionResult;

void LinePosition_Reset(void);
LinePositionResult LinePosition_Update(uint8_t black_bits);

#endif
