#include <stdio.h>
#include <stdlib.h>

#include "modules/line_tracking/controller/line_lookup_control.h"

#define CHECK(expr)                                                     \
    do {                                                                \
        if (!(expr)) {                                                  \
            (void)fprintf(stderr, "CHECK failed at line %d: %s\n",      \
                          __LINE__, #expr);                             \
            exit(1);                                                    \
        }                                                               \
    } while (0)

static int abs16(int16_t value)
{
    return value < 0 ? -(int)value : (int)value;
}

int main(void)
{
    LineLookupCommand center;
    LineLookupCommand left_edge;
    LineLookupCommand right_edge;
    LineLookupCommand limited;
    LineLookupCommand stale;
    int8_t position;

    /* Center drives straight inside the agreed base window. */
    center = LineLookupControl_Step(0, 0.0f, false);
    CHECK(center.valid);
    CHECK(center.left == center.right);
    CHECK(center.diff == 0);
    CHECK(center.base >= 400 && center.base <= 430);

    /* The table is mirror symmetric between the two edges. */
    left_edge = LineLookupControl_Step(-7, 0.0f, false);
    right_edge = LineLookupControl_Step(7, 0.0f, false);
    CHECK(left_edge.valid && right_edge.valid);
    CHECK(left_edge.left == right_edge.right);
    CHECK(left_edge.right == right_edge.left);
    CHECK(left_edge.base == right_edge.base);

    /* Every position stays inside the safety command limit. */
    for (position = -7; position <= 7; position++) {
        LineLookupCommand command =
            LineLookupControl_Step(position, 0.0f, false);
        CHECK(command.valid);
        CHECK(abs16(command.left) <= 450 && abs16(command.right) <= 450);
    }

    /* Negative position means line on the left: left wheel slower. */
    CHECK(left_edge.left < left_edge.right);
    /* Positive position means line on the right: right wheel slower. */
    CHECK(right_edge.right < right_edge.left);

    /* High fresh yaw near the edge shrinks the differential ... */
    limited = LineLookupControl_Step(-7, 150.0f, true);
    CHECK(abs16(limited.diff) < abs16(left_edge.diff));
    /* ... but never reverses the requested turn direction. */
    CHECK(limited.diff > 0);
    CHECK(limited.left < limited.right);
    /* Opposite-direction yaw also never reverses the turn. */
    limited = LineLookupControl_Step(-7, -150.0f, true);
    CHECK(limited.diff > 0);
    CHECK(limited.left < limited.right);

    /* Low fresh yaw never modifies the fresh table output. */
    stale = LineLookupControl_Step(-7, 10.0f, true);
    limited = LineLookupControl_Step(-7, 0.0f, true);
    CHECK(stale.left == limited.left && stale.right == limited.right);

    /* Yaw limiting only applies near the edges (|position| >= 5). */
    stale = LineLookupControl_Step(-4, 150.0f, true);
    limited = LineLookupControl_Step(-4, 0.0f, true);
    CHECK(stale.left == limited.left && stale.right == limited.right);

    /* A stale IMU degrades every wheel command to the 280 cap. */
    for (position = -7; position <= 7; position++) {
        LineLookupCommand degraded =
            LineLookupControl_Step(position, 0.0f, false);
        CHECK(degraded.valid);
        CHECK(abs16(degraded.left) <= 280 && abs16(degraded.right) <= 280);
    }
    /* Degradation caps magnitude but keeps the steering direction. */
    stale = LineLookupControl_Step(-7, 0.0f, false);
    CHECK(stale.left < stale.right);
    /* A fresh IMU allows the full table speed again. */
    limited = LineLookupControl_Step(0, 0.0f, true);
    CHECK(limited.left > 280 && limited.right > 280);

    /* Out-of-range positions produce an invalid zero command. */
    limited = LineLookupControl_Step(8, 0.0f, false);
    CHECK(!limited.valid);
    CHECK(limited.left == 0 && limited.right == 0);
    limited = LineLookupControl_Step(-8, 0.0f, false);
    CHECK(!limited.valid);
    CHECK(limited.left == 0 && limited.right == 0);

    return 0;
}
