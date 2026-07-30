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

    /* The four physical channels decode in left-to-right bit order. */
    LinePosition_Reset();
    result = LinePosition_Update(0x01U);
    CHECK(result.stable_position == -3);
    CHECK(result.weighted_error == -3.0f && result.confidence == 55U &&
          result.reliable);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x03U).stable_position == -2);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x02U).stable_position == -1);
    LinePosition_Reset();
    result = LinePosition_Update(0x06U);
    CHECK(result.stable_position == 0 && result.weighted_error == 0.0f &&
          result.confidence == 100U && result.reliable);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x04U).stable_position == 1);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x0CU).stable_position == 2);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x08U).stable_position == 3);

    /* Legal frames report LINE_PATTERN_POSITION. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x06U).type == LINE_PATTERN_POSITION);

    /* A contiguous run of three or more sensors is WIDE. */
    LinePosition_Reset();
    result = LinePosition_Update(0x07U);
    CHECK(result.type == LINE_PATTERN_WIDE);
    CHECK(result.candidate_position < 0);
    CHECK(result.weighted_error < 0.0f && result.confidence == 45U &&
          result.reliable);
    LinePosition_Reset();
    result = LinePosition_Update(0x0EU);
    CHECK(result.type == LINE_PATTERN_WIDE);
    CHECK(result.candidate_position > 0);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x0FU).type == LINE_PATTERN_WIDE);

    /* Separated runs are NOISE. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x05U).type == LINE_PATTERN_NOISE);
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x09U).type == LINE_PATTERN_NOISE);

    /* No black sensor is LOST. */
    LinePosition_Reset();
    result = LinePosition_Update(0x00U);
    CHECK(result.type == LINE_PATTERN_LOST && !result.reliable &&
          result.confidence == 0U);

    /* A far left-to-right jump needs two matching samples. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x01U).stable_position == -3);
    result = LinePosition_Update(0x08U);
    CHECK(result.stable_position == -3);
    CHECK(result.candidate_position == 3);
    CHECK(result.candidate_frames == 1U);
    result = LinePosition_Update(0x08U);
    CHECK(result.stable_position == 3);
    CHECK(result.candidate_frames == 2U);

    /* A changing far candidate never gets accepted. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x01U).stable_position == -3);
    CHECK(LinePosition_Update(0x08U).stable_position == -3);
    CHECK(LinePosition_Update(0x04U).stable_position == -3);
    CHECK(LinePosition_Update(0x08U).stable_position == -3);

    /* Lost frames hold the stable value and restart debouncing. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x01U).stable_position == -3);
    result = LinePosition_Update(0x00U);
    CHECK(result.type == LINE_PATTERN_LOST);
    CHECK(result.stable_position == -3);
    result = LinePosition_Update(0x08U);
    CHECK(result.stable_position == -3);
    result = LinePosition_Update(0x00U);
    CHECK(result.stable_position == -3);
    CHECK(result.candidate_frames == 0U);
    result = LinePosition_Update(0x08U);
    CHECK(result.stable_position == -3);
    result = LinePosition_Update(0x08U);
    CHECK(result.stable_position == 3);

    /* Raw bits are echoed back for diagnostics. */
    LinePosition_Reset();
    CHECK(LinePosition_Update(0x06U).black_bits == 0x06U);

    return 0;
}
