#include "four_line_scanner.h"

#if defined(FOUR_LINE_SCANNER_HOST_TEST)
#include <stdint.h>

#define GPIOA ((uintptr_t)0U)
#define DL_GPIO_PIN_24 (1UL << 24)
#define DL_GPIO_PIN_25 (1UL << 25)
#define DL_GPIO_PIN_26 (1UL << 26)
#define DL_GPIO_PIN_27 (1UL << 27)
extern uint32_t DL_GPIO_readPins(uintptr_t gpio, uint32_t pins);
#else
#include <ti/devices/msp/m0p/mspm0g350x.h>
#include <ti/driverlib/dl_gpio.h>
#endif

#include "../line_tracking_config.h"

#define FOUR_LINE_X1_PIN DL_GPIO_PIN_24
#define FOUR_LINE_X2_PIN DL_GPIO_PIN_25
#define FOUR_LINE_X3_PIN DL_GPIO_PIN_26
#define FOUR_LINE_X4_PIN DL_GPIO_PIN_27

static LineSensorSnapshot published_snapshot;

void FourLineScanner_Init(void)
{
    published_snapshot = (LineSensorSnapshot){0};
    published_snapshot.status.health = MODULE_HEALTH_UNKNOWN;
}

void FourLineScanner_Sample(uint32_t now_ms)
{
    uint32_t levels = DL_GPIO_readPins(GPIOA,
        FOUR_LINE_X1_PIN | FOUR_LINE_X2_PIN |
        FOUR_LINE_X3_PIN | FOUR_LINE_X4_PIN);
    uint8_t raw =
        ((levels & FOUR_LINE_X2_PIN) != 0U ? 0x01U : 0U) |
        ((levels & FOUR_LINE_X1_PIN) != 0U ? 0x02U : 0U) |
        ((levels & FOUR_LINE_X3_PIN) != 0U ? 0x04U : 0U) |
        ((levels & FOUR_LINE_X4_PIN) != 0U ? 0x08U : 0U);

    published_snapshot.black_bits =
        FOUR_LINE_BLACK_ACTIVE_LEVEL != 0U ? raw : (uint8_t)(raw ^ 0x0FU);
    published_snapshot.status.timestamp_ms = now_ms;
    published_snapshot.status.sequence++;
    published_snapshot.status.valid = true;
    published_snapshot.status.health = MODULE_HEALTH_OK;
}

bool FourLineScanner_GetSnapshot(LineSensorSnapshot *out)
{
    if (out == 0 || !published_snapshot.status.valid) {
        return false;
    }
    *out = published_snapshot;
    return true;
}
