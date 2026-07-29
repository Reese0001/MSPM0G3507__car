#include <stdio.h>
#include <stdlib.h>

#include "modules/line_tracking/controller/line_official_control.h"

#define CHECK(expr)                                                     \
    do {                                                                \
        if (!(expr)) {                                                  \
            (void)fprintf(stderr, "CHECK failed at line %d: %s\n",      \
                          __LINE__, #expr);                             \
            exit(1);                                                    \
        }                                                               \
    } while (0)

static LinePositionResult position(LinePatternType type, int8_t value)
{
    LinePositionResult result = {0};
    result.type = type;
    result.stable_position = value;
    result.candidate_position = value;
    return result;
}

int main(void)
{
    LineOfficialControlResult result;
    LineOfficialControlDiagnostics diagnostics;
    LinePositionResult frame;
    int16_t undamped_turn;

    LineOfficialControl_Init();

    frame = position(LINE_PATTERN_POSITION, 0);
    CHECK(LineOfficialControl_Step(&frame, 1U, 0U, 0.0f, false, &result));
    CHECK(result.follow.valid);
    CHECK(result.follow.forward == 140 && result.follow.turn == 0);
    CHECK(!result.imu_used);

    frame = position(LINE_PATTERN_POSITION, -7);
    CHECK(LineOfficialControl_Step(&frame, 2U, 10U, 0.0f, false, &result));
    CHECK(result.recovery_direction == -1);
    CHECK(result.follow.turn > 0);
    undamped_turn = result.follow.turn;

    /* Positive body-Z rate opposes the existing left command. */
    CHECK(LineOfficialControl_Step(&frame, 3U, 20U, 50.0f, true, &result));
    CHECK(result.imu_used);
    CHECK(result.follow.turn < undamped_turn);

    /* Deadband and stale snapshots must contribute exactly zero correction. */
    CHECK(LineOfficialControl_Step(&frame, 4U, 30U, 2.0f, true, &result));
    CHECK(!result.imu_used);
    CHECK(result.follow.turn == undamped_turn);
    CHECK(LineOfficialControl_Step(&frame, 5U, 40U, 500.0f, false, &result));
    CHECK(!result.imu_used);
    CHECK(result.follow.turn == undamped_turn);

    CHECK(LineOfficialControl_Step(&frame, 6U, 50U, 1000.0f, true, &result));
    LineOfficialControl_GetDiagnostics(&diagnostics);
    CHECK(diagnostics.damping_command == 24);

    /* A centered wide frame must not erase the last reliable left direction. */
    frame = position(LINE_PATTERN_WIDE, 0);
    CHECK(LineOfficialControl_Step(&frame, 7U, 60U, 0.0f, false, &result));
    CHECK(result.recovery_direction == -1);

    /* Separated noise holds the last command briefly, then becomes lost. */
    frame = position(LINE_PATTERN_NOISE, 0);
    CHECK(LineOfficialControl_Step(&frame, 8U, 70U, 0.0f, false, &result));
    CHECK(result.follow.valid && result.estimate.event != LINE_EVENT_LOST);
    CHECK(LineOfficialControl_Step(&frame, 9U, 90U, 0.0f, false, &result));
    CHECK(result.follow.valid && result.estimate.event != LINE_EVENT_LOST);
    CHECK(LineOfficialControl_Step(&frame, 10U, 91U, 0.0f, false, &result));
    CHECK(!result.follow.valid && result.estimate.event == LINE_EVENT_LOST);
    CHECK(result.recovery_direction == -1);

    frame = position(LINE_PATTERN_LOST, 0);
    CHECK(LineOfficialControl_Step(&frame, 11U, 100U, 0.0f, false, &result));
    CHECK(result.estimate.event == LINE_EVENT_LOST);
    CHECK(result.recovery_direction == -1);

    return 0;
}
