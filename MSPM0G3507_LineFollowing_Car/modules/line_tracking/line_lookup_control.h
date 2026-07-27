#ifndef LINE_LOOKUP_CONTROL_H
#define LINE_LOOKUP_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Open-loop lookup controller: converts a stable 15-position line
 * estimate (-7..7, negative = line left of center) into bounded
 * left/right motor commands. Positive differential slows the left
 * wheel and turns left. No PID state is kept; the fresh MPU yaw rate
 * only limits excessive differential in sharp corners.
 */

typedef struct {
    int16_t left;
    int16_t right;
    int16_t base;
    int16_t diff;
    bool valid;
} LineLookupCommand;

LineLookupCommand LineLookupControl_Step(int8_t position,
                                         float yaw_rate_dps,
                                         bool yaw_fresh);

#endif
