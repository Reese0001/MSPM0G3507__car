#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

#include "module_status.h"

typedef struct {
    ModuleStatus status;
    uint32_t pulse_us;
    uint16_t distance_mm;
} UltrasonicSnapshot;

bool Ultrasonic_PulseUsToMm(uint32_t pulse_us, uint16_t *distance_mm);
bool Ultrasonic_GetSnapshot(UltrasonicSnapshot *out);

#endif
