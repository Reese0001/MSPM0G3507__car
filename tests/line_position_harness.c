#include <stdio.h>
#include <stdlib.h>

#include "modules/line_tracking/decoder/line_position.h"

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
    LinePositionResult result;

    /* Each of the 15 legal patterns decodes on the first frame after reset. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x01U).stable_position == 7);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x03U).stable_position == 6);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x02U).stable_position == 5);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x06U).stable_position == 4);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x04U).stable_position == 3);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x0CU).stable_position == 2);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x08U).stable_position == 1);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x18U).stable_position == 0);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x10U).stable_position == -1);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x30U).stable_position == -2);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x20U).stable_position == -3);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x60U).stable_position == -4);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x40U).stable_position == -5);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0xC0U).stable_position == -6);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x80U).stable_position == -7);

    /* Legal frames report LINE_PATTERN_POSITION. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x18U).type == LINE_PATTERN_POSITION);

    /* A contiguous run of three or more sensors is WIDE. */
    LinePosition_Reset();
    result = LinePosition_Update(0x07U);
    CHECK(result.type == LINE_PATTERN_WIDE);
    CHECK(result.candidate_position > 0);
    LinePosition_Reset();
    result = LinePosition_Update(0xE0U);
    CHECK(result.type == LINE_PATTERN_WIDE);
    CHECK(result.candidate_position < 0);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0xFFU).type == LINE_PATTERN_WIDE);

    /* Separated runs are NOISE. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x11U).type == LINE_PATTERN_NOISE);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x81U).type == LINE_PATTERN_NOISE);

    /* No black sensor is LOST. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x00U).type == LINE_PATTERN_LOST);

    /* +7 -> +6 (adjacent) is accepted immediately. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x01U).stable_position == 7);
    result = LinePosition_Update(0x03U);
    CHECK(result.type == LINE_PATTERN_POSITION);
    CHECK(result.stable_position == 6);

    /* +7 -> -7 is rejected on the first frame ... */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x01U).stable_position == 7);
    result = LinePosition_Update(0x80U);
    CHECK(result.stable_position == 7);
    CHECK(result.candidate_position == -7);
    CHECK(result.candidate_frames == 1U);
    /* ... and accepted on the second identical frame. */
    result = LinePosition_Update(0x80U);
    CHECK(result.stable_position == -7);
    CHECK(result.candidate_frames == 2U);

    /* A changing far candidate never gets accepted. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x01U).stable_position == 7);
    CHECK(LinePosition_Update(0x80U).stable_position == 7);
    CHECK(LinePosition_Update(0x40U).stable_position == 7);
    CHECK(LinePosition_Update(0x80U).stable_position == 7);

    /* Illegal frames hold the stable value and restart debouncing. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x01U).stable_position == 7);
    result = LinePosition_Update(0x00U);
    CHECK(result.type == LINE_PATTERN_LOST);
    CHECK(result.stable_position == 7);
    result = LinePosition_Update(0x80U);
    CHECK(result.stable_position == 7);
    result = LinePosition_Update(0x00U);
    CHECK(result.stable_position == 7);
    CHECK(result.candidate_frames == 0U);
    result = LinePosition_Update(0x80U);
    CHECK(result.stable_position == 7);
    result = LinePosition_Update(0x80U);
    CHECK(result.stable_position == -7);

    /* Raw bits are echoed back for diagnostics. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x18U).black_bits == 0x18U);

    return 0;
}
