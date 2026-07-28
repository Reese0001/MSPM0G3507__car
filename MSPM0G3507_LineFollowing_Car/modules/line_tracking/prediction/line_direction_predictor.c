#include "line_direction_predictor.h"

static int8_t positions[3];
static int8_t count;
static int8_t last_nonzero;

void LineDirectionPredictor_Reset(void)
{
    positions[0] = 0;
    positions[1] = 0;
    positions[2] = 0;
    count = 0;
    last_nonzero = 0;
}

void LineDirectionPredictor_Record(int8_t position)
{
    if ((position <= -7) || (position >= 7)) {
        return;
    }

    if (count < 3) {
        positions[count] = position;
        count++;
    } else {
        positions[0] = positions[1];
        positions[1] = positions[2];
        positions[2] = position;
    }

    if (position < 0) {
        last_nonzero = -1;
    } else if (position > 0) {
        last_nonzero = 1;
    }
}

int8_t LineDirectionPredictor_Predict(void)
{
    int8_t prediction;

    if (count < 3) {
        return last_nonzero == 0 ? 1 : last_nonzero;
    }

    prediction = (int8_t)(positions[2] + (positions[2] - positions[1]) +
        (positions[1] - positions[0]));
    if (prediction < 0) {
        return -1;
    }
    if (prediction > 0) {
        return 1;
    }
    return last_nonzero == 0 ? 1 : last_nonzero;
}
