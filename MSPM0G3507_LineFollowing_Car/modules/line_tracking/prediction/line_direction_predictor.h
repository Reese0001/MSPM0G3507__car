#ifndef LINE_DIRECTION_PREDICTOR_H
#define LINE_DIRECTION_PREDICTOR_H

#include <stdint.h>

void LineDirectionPredictor_Reset(void);
void LineDirectionPredictor_Record(int8_t position);
int8_t LineDirectionPredictor_Predict(void);

#endif
