#include <stdint.h>
#include "../MSPM0G3507_LineFollowing_Car/modules/line_tracking/prediction/line_direction_predictor.h"

static int expect_prediction(int8_t expected)
{
    int8_t actual = LineDirectionPredictor_Predict();

    return actual == expected ? 0 : 1;
}

int main(void)
{
    int failures = 0;

    LineDirectionPredictor_Reset();
    failures += expect_prediction(1);

    LineDirectionPredictor_Reset();
    LineDirectionPredictor_Record(-2);
    failures += expect_prediction(-1);
    LineDirectionPredictor_Record(3);
    failures += expect_prediction(1);

    LineDirectionPredictor_Reset();
    LineDirectionPredictor_Record(-1);
    LineDirectionPredictor_Record(-3);
    LineDirectionPredictor_Record(-5);
    failures += expect_prediction(-1);

    LineDirectionPredictor_Reset();
    LineDirectionPredictor_Record(1);
    LineDirectionPredictor_Record(3);
    LineDirectionPredictor_Record(5);
    failures += expect_prediction(1);

    LineDirectionPredictor_Reset();
    LineDirectionPredictor_Record(-4);
    LineDirectionPredictor_Record(-2);
    LineDirectionPredictor_Record(0);
    failures += expect_prediction(1);

    LineDirectionPredictor_Reset();
    LineDirectionPredictor_Record(2);
    LineDirectionPredictor_Record(0);
    LineDirectionPredictor_Record(1);
    failures += expect_prediction(1);

    LineDirectionPredictor_Reset();
    LineDirectionPredictor_Record(-7);
    failures += expect_prediction(-1);

    LineDirectionPredictor_Reset();
    LineDirectionPredictor_Record(7);
    failures += expect_prediction(1);

    LineDirectionPredictor_Reset();
    LineDirectionPredictor_Record(-1);
    LineDirectionPredictor_Record(-3);
    LineDirectionPredictor_Record(-5);
    LineDirectionPredictor_Record(-8);
    LineDirectionPredictor_Record(8);
    failures += expect_prediction(-1);
    failures += expect_prediction(-1);

    return failures == 0 ? 0 : 1;
}
