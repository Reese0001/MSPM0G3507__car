#ifndef DIFFERENTIAL_CONTROLLER_H
#define DIFFERENTIAL_CONTROLLER_H

#include <stdint.h>

typedef struct {
    float average_integral;
    float difference_integral;
} DifferentialController;

void DifferentialController_Init(DifferentialController *controller);
void DifferentialController_Update(DifferentialController *controller,
                                   float target_left,
                                   float target_right,
                                   float actual_left,
                                   float actual_right,
                                   float dt_s,
                                   float *output_left,
                                   float *output_right);

#endif
