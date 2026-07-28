#include "line_lookup_control.h"

#include "../../../config/line_lookup_config.h"

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
    if (value < 0) {
        return 0;
    }
    return (int16_t)value;
}

LineLookupCommand LineLookupControl_Step(int8_t position)
{
    LineLookupCommand command = {0, 0, 0, 0, false};
    int8_t magnitude_index;
    int16_t diff;

    if (position < -7 || position > 7) {
        return command;
    }

    magnitude_index = (int8_t)(position < 0 ? -position : position);
    command.base = table[magnitude_index].base;

    /* Negative position means the line is left of the car. Positive
     * differential slows the left wheel and turns left. */
    diff = table[magnitude_index].diff;
    if (position > 0) {
        diff = (int16_t)-diff;
    }

    command.diff = diff;
    command.left = clamp_command((int32_t)command.base - diff);
    command.right = clamp_command((int32_t)command.base + diff);
    command.valid = true;
    return command;
}
