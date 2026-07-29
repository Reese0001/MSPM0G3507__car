#include <assert.h>
#include <math.h>

#include "modules/optional/foc/foc_math.h"

static int nearly_equal(float lhs, float rhs)
{
    return fabsf(lhs - rhs) < 0.001f;
}

static void expect_clarke_park_round_trip(void)
{
    FocAlphaBeta stationary = FocMath_Clarke(2.0f, -1.0f);
    FocDq rotating = FocMath_Park(stationary, 1.0f, 0.0f);
    FocAlphaBeta restored = FocMath_InversePark(rotating, 1.0f, 0.0f);

    assert(nearly_equal(stationary.alpha, 2.0f));
    assert(nearly_equal(stationary.beta, 0.0f));
    assert(nearly_equal(rotating.d, 0.0f));
    assert(nearly_equal(rotating.q, -2.0f));
    assert(nearly_equal(restored.alpha, stationary.alpha));
    assert(nearly_equal(restored.beta, stationary.beta));
}

static void expect_voltage_vector_is_limited(void)
{
    FocDq command = {6.0f, 8.0f};
    FocDq limited = FocMath_LimitVoltage(command, 5.0f);

    assert(nearly_equal(limited.d, 3.0f));
    assert(nearly_equal(limited.q, 4.0f));
}

static void expect_invalid_limit_stops_output(void)
{
    FocDq command = {3.0f, 4.0f};
    FocDq limited = FocMath_LimitVoltage(command, 0.0f);

    assert(nearly_equal(limited.d, 0.0f));
    assert(nearly_equal(limited.q, 0.0f));
}

int main(void)
{
    expect_clarke_park_round_trip();
    expect_voltage_vector_is_limited();
    expect_invalid_limit_stops_output();
    return 0;
}
