#ifndef STOP_LINE_DETECTOR_H
#define STOP_LINE_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool departed;
    bool candidate;
    bool stop_event;
    uint8_t candidate_frames;
    uint8_t black_bits;
    uint32_t center_distance_mm;
} StopLineResult;

void StopLineDetector_Init(void);
void StopLineDetector_Start(uint32_t now_ms, uint32_t distance_mm);
StopLineResult StopLineDetector_Update(uint8_t black_bits,
                                       uint32_t distance_mm,
                                       uint32_t now_ms);

#endif
