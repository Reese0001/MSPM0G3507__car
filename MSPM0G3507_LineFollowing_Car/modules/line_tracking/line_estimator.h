#ifndef LINE_ESTIMATOR_H
#define LINE_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "line_scanner.h"

typedef enum {
    LINE_EVENT_NONE = 0,
    LINE_EVENT_HARD_LEFT,
    LINE_EVENT_HARD_RIGHT,
    LINE_EVENT_WIDE_BLACK,
    LINE_EVENT_LOST
} LineEvent;

typedef struct {
    ModuleStatus status;
    float error;
    float derivative;
    float predicted_error;
    uint8_t confidence;
    LineEvent event;
} LineEstimate;

void LineEstimator_Init(void);
bool LineEstimator_Update(const LineSensorSnapshot *snapshot, uint32_t now_ms);
bool LineEstimator_Get(LineEstimate *out);

#endif
