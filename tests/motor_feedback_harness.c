#include <assert.h>
#include <math.h>

#include "modules/motor/feedback/motor_feedback.h"

int main(void)
{
    MotorFeedbackSnapshot snapshot;

    MotorFeedback_Init();
    assert(MotorFeedback_ParseSpeedFrame("MSPD:100.0,110.0,0.0,90.0", 0U));
    assert(MotorFeedback_GetSnapshot(&snapshot, 0U));
    assert(fabsf(snapshot.left_speed_mm_s - 90.0f) < 0.01f);
    assert(fabsf(snapshot.right_speed_mm_s - 110.0f) < 0.01f);

    MotorFeedback_UpdateOdometry(0U);
    MotorFeedback_UpdateOdometry(40U);
    assert(MotorFeedback_GetDistanceMm() > 3.0f);
    assert(MotorFeedback_GetDistanceMm() < 5.0f);

    assert(!MotorFeedback_IsFresh(151U));
    assert(!MotorFeedback_GetSnapshot(&snapshot, 151U));
    return 0;
}
