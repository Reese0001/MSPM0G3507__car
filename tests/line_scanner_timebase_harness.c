#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "modules/line_tracking/scanner/four_line_scanner.h"

static uint32_t gpio_levels;
static uint8_t gpio_read_count;

uint32_t DL_GPIO_readPins(uintptr_t gpio, uint32_t pins)
{
    assert(gpio == 0U);
    assert(pins == ((1UL << 24) | (1UL << 25) |
                    (1UL << 26) | (1UL << 27)));
    gpio_read_count++;
    return gpio_levels;
}

int main(void)
{
    LineSensorSnapshot snapshot = {0};
    const uint32_t now_ms = 5000000U;

    /* This module drives OUT low over a black line.  Keep the raw GPIO
     * levels explicit so the scanner, not the harness, owns the inversion. */
    gpio_levels = (1UL << 24) | (1UL << 27); /* X2 + X3 black -> 0x05. */
    FourLineScanner_Init();
    FourLineScanner_Sample(now_ms);

    assert(FourLineScanner_GetSnapshot(&snapshot));
    assert(snapshot.black_bits == 0x05U);
    assert(snapshot.status.timestamp_ms == now_ms);
    assert(snapshot.status.sequence == 1U);
    assert(gpio_read_count == 1U);

    /* A second background frame publishes a fresh sequence with the
     * caller's millisecond timestamp. */
    snapshot = (LineSensorSnapshot){0};
    gpio_levels = (1UL << 25) | (1UL << 26); /* X1 + X4 black -> 0x0A. */
    FourLineScanner_Sample(now_ms + 2U);
    assert(FourLineScanner_GetSnapshot(&snapshot));
    assert(snapshot.black_bits == 0x0AU);
    assert(snapshot.status.timestamp_ms == now_ms + 2U);
    assert(snapshot.status.sequence == 2U);
    assert(gpio_read_count == 2U);
    return 0;
}
