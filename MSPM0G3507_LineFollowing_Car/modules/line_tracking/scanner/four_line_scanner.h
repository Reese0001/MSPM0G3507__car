#ifndef FOUR_LINE_SCANNER_H
#define FOUR_LINE_SCANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "../../../shared/module_status.h"

typedef struct {
    ModuleStatus status;
    uint8_t black_bits;
} LineSensorSnapshot;

void FourLineScanner_Init(void);
void FourLineScanner_Sample(uint32_t now_ms);
bool FourLineScanner_GetSnapshot(LineSensorSnapshot *out);

#endif
