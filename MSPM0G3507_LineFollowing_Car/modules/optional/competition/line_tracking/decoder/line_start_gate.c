#include "line_start_gate.h"

static unsigned char valid_frames;
static bool started;

void LineStartGate_Reset(void)
{
    valid_frames = 0U;
    started = false;
}

bool LineStartGate_Update(LinePatternType type)
{
    if (started) {
        return true;
    }
    if (type != LINE_PATTERN_POSITION) {
        valid_frames = 0U;
        return false;
    }
    if (valid_frames < 2U) {
        valid_frames++;
    }
    started = valid_frames >= 2U;
    return started;
}
