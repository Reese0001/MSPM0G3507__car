#ifndef LINE_LAP_TRACKER_H
#define LINE_LAP_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LAP_WAITING = 0,
    LAP_LEAVING_START,
    LAP_RUNNING,
    LAP_STOPPING,
    LAP_FINISHED
} LapState;

typedef struct {
    LapState state;
    uint32_t elapsed_ms;
} LapStatus;

void LapTracker_Reset(void);
void LapTracker_Init(void);
void LapTracker_Start(uint32_t now_ms);
bool LapTracker_Update(uint8_t black_bits,
                       uint32_t distance_mm,
                       uint32_t now_ms);
void LapTracker_NotifyStopped(uint32_t now_ms);
uint32_t LapTracker_GetStopTargetMm(void);
void LapTracker_GetStatus(uint32_t now_ms, LapStatus *status);
uint32_t LapTracker_GetElapsedMs(void);

#endif
