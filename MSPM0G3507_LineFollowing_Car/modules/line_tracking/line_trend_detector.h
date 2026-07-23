#ifndef LINE_TREND_DETECTOR_H
#define LINE_TREND_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "line_estimator.h"
#include "line_scanner.h"

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
    LineTrendType type;
    int8_t direction;
} LineTrendResult;

void LineTrendDetector_Init(void);
void LineTrendDetector_Reset(void);
bool LineTrendDetector_Update(const LineEstimate *estimate,
                              const LineSensorSnapshot *snapshot,
                              uint32_t now_ms,
                              LineTrendResult *result);

#endif
