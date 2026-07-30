#include <assert.h>
#include <stdint.h>

#include "modules/line_tracking/stop_line_detector.h"

int main(void)
{
    StopLineResult result;
    const uint8_t marker_patterns[] = {0x07U, 0x0EU, 0x0FU};
    uint8_t index;

    StopLineDetector_Init();
    StopLineDetector_Start(0U, 0U);

    /* The car must leave the initial marker before a return can stop it. */
    (void)StopLineDetector_Update(0x0FU, 0U, 0U);
    (void)StopLineDetector_Update(0x06U, 10U, 2U);
    (void)StopLineDetector_Update(0x06U, 20U, 4U);
    result = StopLineDetector_Update(0x06U, 30U, 6U);
    assert(result.departed);
    assert(!result.stop_event);

    /* A normal center line before the distance gate is not a stop line. */
    result = StopLineDetector_Update(0x06U, 3000U, 3000U);
    assert(!result.stop_event);
    result = StopLineDetector_Update(0x07U, 4500U, 5000U);
    assert(!result.stop_event);
    result = StopLineDetector_Update(0x06U, 4510U, 5010U);
    assert(!result.stop_event);

    /* The real stop bar may be 3-of-4 or 4-of-4, not only 0x0f. */
    for (index = 0U; index < 3U; index++) {
        StopLineDetector_Init();
        StopLineDetector_Start(0U, 0U);
        (void)StopLineDetector_Update(0x06U, 0U, 0U);
        (void)StopLineDetector_Update(0x06U, 10U, 2U);
        (void)StopLineDetector_Update(0x06U, 20U, 4U);
        (void)StopLineDetector_Update(0x06U, 30U, 6U);
        (void)StopLineDetector_Update(marker_patterns[index], 4500U, 5000U);
        (void)StopLineDetector_Update(marker_patterns[index], 4510U, 5010U);
        result = StopLineDetector_Update(0x06U, 4520U, 5020U);
        assert(result.stop_event);
        assert(result.center_distance_mm >= 4500U);
        assert(result.center_distance_mm <= 4520U);
    }

    return 0;
}
