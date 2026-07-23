#ifndef SAFETY_DECISION_H
#define SAFETY_DECISION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SAFETY_BOOT_SAFE = 0,
    SAFETY_READY,
    SAFETY_RUNNING,
    SAFETY_LIMITED,
    SAFETY_STOP_LATCHED,
    SAFETY_FAULT
} SafetySupervisorState;

typedef enum {
    SAFETY_REASON_NONE = 0U,
    SAFETY_REASON_BOOT_GATE = 1U << 0,
    SAFETY_REASON_MOTOR_FAULT = 1U << 1,
    SAFETY_REASON_OBSTACLE = 1U << 2,
    SAFETY_REASON_SENSOR_STALE = 1U << 3,
    SAFETY_REASON_REQUEST_INVALID = 1U << 4,
    SAFETY_REASON_POWER = 1U << 5
} SafetyReason;

typedef struct {
    bool approved;
    int16_t left_speed;
    int16_t right_speed;
    uint16_t reason;
    SafetySupervisorState state;
} SafetyDecision;

#endif
