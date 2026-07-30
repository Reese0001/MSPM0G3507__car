#ifndef LINE_TREND_DETECTOR_H
#define LINE_TREND_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "../../../line_tracking/line_model.h"
#include "../../../line_tracking/scanner/four_line_scanner.h"

void LineTrendDetector_Init(void);
void LineTrendDetector_Reset(void);
bool LineTrendDetector_Update(const LineEstimate *estimate,
                              const LineSensorSnapshot *snapshot,
                              uint32_t now_ms,
                              LineTrendResult *result);

#endif
