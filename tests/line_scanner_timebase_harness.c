#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "modules/line_tracking/line_scanner.h"

static uint32_t fake_us;

void BSP_LineMux_SelectChannel(uint8_t channel)
{
    (void)channel;
}

bool BSP_LineMux_IsBlack(void)
{
    return true;
}

uint32_t BSP_Time_GetUs(void)
{
    /* Each observation advances one microsecond of fake time. */
    fake_us += 1U;
    return fake_us;
}

int main(void)
{
    LineSensorSnapshot snapshot = {0};
    uint32_t now_us = UINT32_MAX - 100U;
    const uint32_t now_ms = 5000000U;
    uint8_t channel;

    LineScanner_Init();
    for (channel = 0U; channel < 8U; channel++) {
        LineScanner_Service(now_us, now_ms);
        now_us += 50U;
        LineScanner_Service(now_us, now_ms);
        LineScanner_Service(now_us, now_ms);
    }

    assert(LineScanner_GetSnapshot(&snapshot));
    assert(snapshot.black_bits == 0xFFU);
    assert(snapshot.status.timestamp_ms == now_ms);
    assert(snapshot.status.sequence == 1U);

    /* A full-frame read completes inside the scan budget and publishes
     * a fresh sequence with the caller's millisecond timestamp. */
    snapshot = (LineSensorSnapshot){0};
    assert(LineScanner_ReadFrame(now_ms + 2U, &snapshot));
    assert(snapshot.black_bits == 0xFFU);
    assert(snapshot.status.timestamp_ms == now_ms + 2U);
    assert(snapshot.status.sequence == 2U);
    return 0;
}
