#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "modules/line_tracking/line_scanner.h"

void BSP_LineMux_SelectChannel(uint8_t channel)
{
    (void)channel;
}

bool BSP_LineMux_IsBlack(void)
{
    return true;
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
    return 0;
}
