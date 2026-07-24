#ifndef LINE_SCANNER_H
#define LINE_SCANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "../common/module_status.h"

typedef struct {
    ModuleStatus status;
    uint8_t black_bits;
} LineSensorSnapshot;

void LineScanner_Init(void);
void LineScanner_Service(uint32_t now_us, uint32_t now_ms);
bool LineScanner_GetSnapshot(LineSensorSnapshot *out);

#endif
