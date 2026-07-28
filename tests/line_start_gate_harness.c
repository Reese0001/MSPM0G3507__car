#include <assert.h>

#include "modules/optional/competition/line_tracking/decoder/line_start_gate.h"

int main(void)
{
    LineStartGate_Reset();
    assert(!LineStartGate_Update(LINE_PATTERN_LOST));
    assert(!LineStartGate_Update(LINE_PATTERN_POSITION));
    assert(LineStartGate_Update(LINE_PATTERN_POSITION));
    assert(LineStartGate_Update(LINE_PATTERN_NOISE));

    LineStartGate_Reset();
    assert(!LineStartGate_Update(LINE_PATTERN_POSITION));
    assert(!LineStartGate_Update(LINE_PATTERN_WIDE));
    assert(!LineStartGate_Update(LINE_PATTERN_POSITION));
    assert(LineStartGate_Update(LINE_PATTERN_POSITION));
    return 0;
}
