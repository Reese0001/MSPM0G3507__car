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

int main(void)
{
    LineLookupCommand center;
    LineLookupCommand left_near;
    LineLookupCommand right_near;
    LineLookupCommand left_edge;
    LineLookupCommand right_edge;
    LineLookupCommand invalid;
    int8_t position;

    /* Center and the three near-center positions drive straight. */
    center = LineLookupControl_Step(0);
    CHECK(center.valid);
    CHECK(center.base == 140);
    CHECK(center.diff == 0);
    CHECK(center.left == 140 && center.right == 140);
    left_near = LineLookupControl_Step(-1);
    right_near = LineLookupControl_Step(1);
    CHECK(left_near.valid && right_near.valid);
    CHECK(left_near.diff == 0 && right_near.diff == 0);

    /* The table is mirror symmetric between the two edges. */
    left_edge = LineLookupControl_Step(-7);
    right_edge = LineLookupControl_Step(7);
    CHECK(left_edge.valid && right_edge.valid);
    CHECK(left_edge.left == right_edge.right);
    CHECK(left_edge.right == right_edge.left);
    CHECK(left_edge.base == right_edge.base);

    /* Every valid position remains a non-negative bounded command. */
    for (position = -7; position <= 7; position++) {
        LineLookupCommand command = LineLookupControl_Step(position);
        CHECK(command.valid);
        CHECK(command.base >= 0 && command.base <= 140);
        CHECK(command.left >= 0 && command.left <= 140);
        CHECK(command.right >= 0 && command.right <= 140);
    }

    /* Negative position means line on the left: left wheel slower. */
    CHECK(left_edge.left < left_edge.right);
    /* Positive position means line on the right: right wheel slower. */
    CHECK(right_edge.right < right_edge.left);

    /* Out-of-range positions produce an invalid zero command. */
    invalid = LineLookupControl_Step(8);
    CHECK(!invalid.valid);
    CHECK(invalid.base == 0 && invalid.diff == 0);
    CHECK(invalid.left == 0 && invalid.right == 0);
    invalid = LineLookupControl_Step(-8);
    CHECK(!invalid.valid);
    CHECK(invalid.base == 0 && invalid.diff == 0);
    CHECK(invalid.left == 0 && invalid.right == 0);

    return 0;
}
