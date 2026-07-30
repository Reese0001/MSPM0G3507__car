#ifndef APP_LINE_SAMPLE_H
#define APP_LINE_SAMPLE_H

#include <stdint.h>

#include "../modules/line_tracking/decoder/line_position.h"

typedef struct {
    LinePositionResult position;
    uint16_t sequence;
    uint32_t timestamp_ms;
} AppLineSample;

#endif
