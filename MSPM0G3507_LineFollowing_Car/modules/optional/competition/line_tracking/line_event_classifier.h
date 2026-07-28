#ifndef LINE_EVENT_CLASSIFIER_H
#define LINE_EVENT_CLASSIFIER_H

#include <stdbool.h>
#include <stdint.h>

#include "line_estimator.h"
#include "line_trend_detector.h"

typedef enum {
    LINE_PATH_NORMAL = 0,
    LINE_PATH_WIDE_PENDING,
    LINE_PATH_RIGHT_ANGLE_LEFT,
    LINE_PATH_RIGHT_ANGLE_RIGHT,
    LINE_PATH_LOST,
    LINE_PATH_INVALID
} LinePathEventType;

typedef struct {
    ModuleStatus status;
    LinePathEventType type;
    int8_t direction;
    uint8_t direction_confidence;
} LinePathEvent;

void LineEventClassifier_Init(void);
void LineEventClassifier_Reset(void);
bool LineEventClassifier_Update(const LineFeatures *features,
                                const LineEstimate *estimate,
                                const LineTrendResult *trend,
                                uint32_t now_ms,
                                LinePathEvent *out);

#endif
