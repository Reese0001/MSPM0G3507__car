#include "modules/optional/foc/foc_math.h"

#include <math.h>

#define FOC_ONE_OVER_SQRT_THREE (0.57735026919f)

FocAlphaBeta FocMath_Clarke(float phase_a, float phase_b)
{
    FocAlphaBeta result;

    result.alpha = phase_a;
    result.beta = (phase_a + (2.0f * phase_b)) * FOC_ONE_OVER_SQRT_THREE;
    return result;
}

FocDq FocMath_Park(FocAlphaBeta stationary,
                   float sin_electrical_angle,
                   float cos_electrical_angle)
{
    FocDq result;

    result.d = (stationary.alpha * cos_electrical_angle) +
               (stationary.beta * sin_electrical_angle);
    result.q = (-stationary.alpha * sin_electrical_angle) +
               (stationary.beta * cos_electrical_angle);
    return result;
}

FocAlphaBeta FocMath_InversePark(FocDq rotating,
                                 float sin_electrical_angle,
                                 float cos_electrical_angle)
{
    FocAlphaBeta result;

    result.alpha = (rotating.d * cos_electrical_angle) -
                   (rotating.q * sin_electrical_angle);
    result.beta = (rotating.d * sin_electrical_angle) +
                  (rotating.q * cos_electrical_angle);
    return result;
}

FocDq FocMath_LimitVoltage(FocDq command, float maximum_magnitude)
{
    FocDq stopped = {0.0f, 0.0f};
    float magnitude_squared;
    float maximum_squared;
    float scale;

    if (maximum_magnitude <= 0.0f) {
        return stopped;
    }

    magnitude_squared = (command.d * command.d) + (command.q * command.q);
    maximum_squared = maximum_magnitude * maximum_magnitude;
    if (magnitude_squared <= maximum_squared) {
        return command;
    }

    scale = maximum_magnitude / sqrtf(magnitude_squared);
    command.d *= scale;
    command.q *= scale;
    return command;
}
