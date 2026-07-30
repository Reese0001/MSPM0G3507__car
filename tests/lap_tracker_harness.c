#include <assert.h>
#include <stdint.h>

#include "modules/line_tracking/lap_tracker.h"

int main(void)
{
    LapStatus status;

    LapTracker_Init();
    LapTracker_Start(0U);
    (void)LapTracker_Update(0x0FU, 0U, 0U);
    (void)LapTracker_Update(0x06U, 10U, 2U);
    (void)LapTracker_Update(0x06U, 20U, 4U);
    (void)LapTracker_Update(0x06U, 30U, 6U);
    LapTracker_GetStatus(6U, &status);
    assert(status.state == LAP_RUNNING);

    assert(!LapTracker_Update(0x06U, 3000U, 3000U));
    assert(!LapTracker_Update(0x07U, 4500U, 5000U));
    assert(!LapTracker_Update(0x07U, 4510U, 5010U));
    assert(LapTracker_Update(0x06U, 4520U, 5020U));
    LapTracker_GetStatus(5020U, &status);
    assert(status.state == LAP_STOPPING);
    assert(LapTracker_GetStopTargetMm() == 4555U);
    LapTracker_NotifyStopped(5070U);
    LapTracker_GetStatus(5070U, &status);
    assert(status.state == LAP_FINISHED);
    return 0;
}
