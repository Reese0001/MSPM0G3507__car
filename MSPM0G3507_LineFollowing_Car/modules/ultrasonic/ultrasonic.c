#include "ultrasonic.h"

#include "ultrasonic_config.h"

static UltrasonicSnapshot snapshot = {0};

bool Ultrasonic_PulseUsToMm(uint32_t pulse_us, uint16_t *distance_mm)
{
    if (distance_mm == 0 ||
        pulse_us < ULTRASONIC_MIN_PULSE_US ||
        pulse_us > ULTRASONIC_MAX_PULSE_US) {
        return false;
    }

    *distance_mm = (uint16_t)((pulse_us * 343U + 1000U) / 2000U);
    return true;
}

bool Ultrasonic_GetSnapshot(UltrasonicSnapshot *out)
{
    if (out == 0) {
        return false;
    }

    *out = snapshot;
    return true;
}
