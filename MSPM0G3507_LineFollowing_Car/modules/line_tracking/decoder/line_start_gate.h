#ifndef LINE_START_GATE_H
#define LINE_START_GATE_H

#include <stdbool.h>

#include "line_position.h"

void LineStartGate_Reset(void);
bool LineStartGate_Update(LinePatternType type);

#endif
