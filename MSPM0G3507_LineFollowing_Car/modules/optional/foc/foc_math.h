#ifndef MODULES_OPTIONAL_FOC_FOC_MATH_H
#define MODULES_OPTIONAL_FOC_FOC_MATH_H

typedef struct {
    float alpha;
    float beta;
} FocAlphaBeta;

typedef struct {
    float d;
    float q;
} FocDq;

FocAlphaBeta FocMath_Clarke(float phase_a, float phase_b);
FocDq FocMath_Park(FocAlphaBeta stationary,
                   float sin_electrical_angle,
                   float cos_electrical_angle);
FocAlphaBeta FocMath_InversePark(FocDq rotating,
                                 float sin_electrical_angle,
                                 float cos_electrical_angle);
FocDq FocMath_LimitVoltage(FocDq command, float maximum_magnitude);

#endif
