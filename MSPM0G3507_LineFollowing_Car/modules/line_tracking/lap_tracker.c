#include "lap_tracker.h"

#include "line_tracking_config.h"
#include "stop_line_detector.h"

static LapState state;
static uint32_t start_ms;
static uint32_t elapsed_ms;
static uint32_t stop_target_mm;
static StopLineResult last_stop_result;

void LapTracker_Reset(void)
{
    state = LAP_WAITING;
    start_ms = 0U;
    elapsed_ms = 0U;
    stop_target_mm = 0U;
    last_stop_result = (StopLineResult){0};
    StopLineDetector_Init();
}

void LapTracker_Init(void)
{
    LapTracker_Reset();
}

void LapTracker_Start(uint32_t now_ms)
{
    LapTracker_Reset();
    state = LAP_LEAVING_START;
    start_ms = now_ms;
    StopLineDetector_Start(now_ms, 0U);
}

bool LapTracker_Update(uint8_t black_bits,
                       uint32_t distance_mm,
                       uint32_t now_ms)
{
    if (state == LAP_FINISHED || state == LAP_STOPPING) {
        return false;
    }
    last_stop_result = StopLineDetector_Update(black_bits, distance_mm,
                                                now_ms);
    state = last_stop_result.departed ? LAP_RUNNING : LAP_LEAVING_START;
    if (!last_stop_result.stop_event) {
        return false;
    }
    stop_target_mm = last_stop_result.center_distance_mm +
                     (uint32_t)LINE_SENSOR_TO_TEST_POINT_MM;
    state = LAP_STOPPING;
    return true;
}

void LapTracker_NotifyStopped(uint32_t now_ms)
{
    if (state != LAP_STOPPING) {
        return;
    }
    elapsed_ms = (uint32_t)(now_ms - start_ms);
    state = LAP_FINISHED;
}

uint32_t LapTracker_GetStopTargetMm(void)
{
    return stop_target_mm;
}

void LapTracker_GetStatus(uint32_t now_ms, LapStatus *status)
{
    if (status == 0) {
        return;
    }
    status->state = state;
    if (state == LAP_WAITING) {
        status->elapsed_ms = 0U;
    } else if (state == LAP_FINISHED) {
        status->elapsed_ms = elapsed_ms;
    } else {
        status->elapsed_ms = (uint32_t)(now_ms - start_ms);
    }
}

uint32_t LapTracker_GetElapsedMs(void)
{
    return elapsed_ms;
}
