#include "line_scanner.h"

#include "../../bsp/bsp_line_mux.h"
#include "line_tracking_config.h"

typedef enum {
    LINE_SCAN_SELECT = 0,
    LINE_SCAN_SETTLE,
    LINE_SCAN_SAMPLE
} LineScanState;

static LineScanState scan_state = LINE_SCAN_SELECT;
static uint8_t scan_channel = 0U;
static uint8_t working_black_bits = 0U;
static uint32_t selected_at_us = 0U;
static LineSensorSnapshot published_snapshot = {0};

void LineScanner_Init(void)
{
    scan_state = LINE_SCAN_SELECT;
    scan_channel = 0U;
    working_black_bits = 0U;
    selected_at_us = 0U;
    published_snapshot.status.timestamp_ms = 0U;
    published_snapshot.status.sequence = 0U;
    published_snapshot.status.valid = false;
    published_snapshot.status.health = MODULE_HEALTH_UNKNOWN;
    published_snapshot.black_bits = 0U;
}

void LineScanner_Service(uint32_t now_us)
{
    if (scan_state == LINE_SCAN_SELECT) {
        BSP_LineMux_SelectChannel(scan_channel);
        selected_at_us = now_us;
        scan_state = LINE_SCAN_SETTLE;
        return;
    }

    if (scan_state == LINE_SCAN_SETTLE) {
        if ((uint32_t)(now_us - selected_at_us) >= LINE_MUX_SETTLE_US) {
            scan_state = LINE_SCAN_SAMPLE;
        }
        return;
    }

    if (BSP_LineMux_IsBlack()) {
        working_black_bits |= (uint8_t)(1U << scan_channel);
    }
    scan_channel++;
    if (scan_channel >= 8U) {
        published_snapshot.black_bits = working_black_bits;
        published_snapshot.status.timestamp_ms = now_us / 1000U;
        published_snapshot.status.sequence++;
        published_snapshot.status.valid = true;
        published_snapshot.status.health = MODULE_HEALTH_OK;
        scan_channel = 0U;
        working_black_bits = 0U;
    }
    scan_state = LINE_SCAN_SELECT;
}

bool LineScanner_GetSnapshot(LineSensorSnapshot *out)
{
    if (out == 0 || !published_snapshot.status.valid) {
        return false;
    }
    *out = published_snapshot;
    return true;
}
