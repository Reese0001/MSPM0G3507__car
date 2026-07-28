#ifndef LINE_FEATURES_H
#define LINE_FEATURES_H

#include <stdbool.h>
#include <stdint.h>

#include "../../../line_tracking/scanner/line_scanner.h"

typedef struct {
    ModuleStatus status;
    uint8_t black_bits;
    uint8_t active_count;
    uint8_t left_count;
    uint8_t right_count;
    uint8_t span;
    uint8_t segment_count;
    bool left_edge;
    bool right_edge;
    float centroid_error;
    float error_rate;
    uint8_t confidence;
} LineFeatures;

void LineFeatureExtractor_Init(void);
void LineFeatureExtractor_Reset(void);
bool LineFeatureExtractor_Update(const LineSensorSnapshot *snapshot,
                                 uint32_t now_ms,
                                 LineFeatures *out);

#endif
