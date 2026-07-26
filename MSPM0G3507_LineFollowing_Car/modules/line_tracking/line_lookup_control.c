#include "line_lookup_control.h"

#include <math.h>

#include "../../application/config/line_lookup_config.h"

typedef struct {
    int16_t base;
    int16_t diff;
} LineLookupEntry;

/* Entries for positions 0..7; negative positions use the same row
 * with the differential sign mirrored. */
static const LineLookupEntry table[8] = {LINE_LOOKUP_TABLE_ENTRIES};

static int16_t clamp_command(int32_t value)
{
    if (value > LINE_LOOKUP_COMMAND_LIMIT) {
        return (int16_t)LINE_LOOKUP_COMMAND_LIMIT;
    }
    if (value < -LINE_LOOKUP_COMMAND_LIMIT) {
        return (int16_t)-LINE_LOOKUP_COMMAND_LIMIT;
    }
    return (int16_t)value;
}

LineLookupCommand LineLookupControl_Step(int8_t position,
                                         float yaw_rate_dps,
                                         bool yaw_fresh)
{
    LineLookupCommand command = {0, 0, 0, 0, false};
    int8_t magnitude_index;
    int16_t diff;

    if (position < -7 || position > 7) {
        return command;
    }

    magnitude_index = (int8_t)(position < 0 ? -position : position);
    command.base = table[magnitude_index].base;

    /* Negative position means the line is left: slow the left wheel by
     * subtracting a positive differential. */
    diff = table[magnitude_index].diff;
    if (position > 0) {
        diff = (int16_t)-diff;
    }

    /* MPU acts only as a corner limiter: if the car already yaws fast
     * in a sharp corner, shrink the differential without reversing it. */
    if (yaw_fresh &&
        magnitude_index >= LINE_LOOKUP_YAW_LIMIT_MIN_POSITION &&
        fabsf(yaw_rate_dps) >= LINE_LOOKUP_HIGH_YAW_DPS) {
        diff = (int16_t)(diff * 3 / 4);
    }

    command.diff = diff;
    command.left = clamp_command((int32_t)command.base - diff);
    command.right = clamp_command((int32_t)command.base + diff);
    command.valid = true;
    return command;
}
