#ifndef K230_LINK_H
#define K230_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "../../../shared/module_status.h"

typedef struct {
    ModuleStatus status;
    uint8_t event_id;
    uint8_t confidence;
    int16_t x;
    int16_t y;
    int16_t width;
    int16_t height;
    char text[48];
} K230VisionSnapshot;

typedef struct {
    uint32_t accepted_frames;
    uint32_t rejected_events;
    uint32_t rejected_frames;
    uint32_t rx_overflows;
} K230LinkDiagnostics;

void K230Link_Init(void);
void K230Link_OnRxByteFromISR(uint8_t byte);
void K230Link_Service(uint32_t now_ms);
bool K230Link_GetSnapshot(uint32_t now_ms, K230VisionSnapshot *out);
void K230Link_GetDiagnostics(K230LinkDiagnostics *out);

/* BSP ISR bridge: drain UART2 quickly and defer frame parsing to the scheduler. */
void K230_UART_IRQHandler(void);

#endif
