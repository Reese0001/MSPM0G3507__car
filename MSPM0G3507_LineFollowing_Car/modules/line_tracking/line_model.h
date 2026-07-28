#ifndef LINE_MODEL_H
#define LINE_MODEL_H

#include <stdbool.h>
#include <stdint.h>

#include "../../shared/module_status.h"

typedef enum {
    LINE_EVENT_NONE = 0,
    LINE_EVENT_HARD_LEFT,
    LINE_EVENT_HARD_RIGHT,
    LINE_EVENT_WIDE_BLACK,
    LINE_EVENT_LOST
} LineEvent;

typedef enum {
    LINE_TREND_NORMAL = 0,
    LINE_TREND_TIGHT_LEFT,
    LINE_TREND_TIGHT_RIGHT,
    LINE_TREND_HAIRPIN_LEFT,
    LINE_TREND_HAIRPIN_RIGHT,
    LINE_TREND_RIGHT_ANGLE_LEFT,
    LINE_TREND_RIGHT_ANGLE_RIGHT
} LineTrendType;

typedef struct {
    ModuleStatus status;
    float error;
    float derivative;
    float predicted_error;
    uint8_t confidence;
    LineEvent event;
} LineEstimate;

typedef struct {
    ModuleStatus status;
    LineTrendType type;
    int8_t direction;
} LineTrendResult;

typedef struct {
    int16_t forward;
    int16_t turn;
    bool valid;
} LineControlOutput;

#endif
