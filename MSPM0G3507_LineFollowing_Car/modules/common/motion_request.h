#ifndef MOTION_REQUEST_H
#define MOTION_REQUEST_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t left_speed;
    int16_t right_speed;
    uint32_t timestamp_ms;
    bool valid;
} MotionRequest;

#endif
