#include "ti_msp_dl_config.h"

#define LED_DIAGNOSTIC_HALF_PERIOD_CYCLES (40000000U)

int main(void)
{
    SYSCFG_DL_init();

    while (1) {
        DL_GPIO_setPins(LED_PORT, LED_D1_PIN);
        DL_GPIO_clearPins(LED_PORT, LED_D2_PIN);
        DL_Common_delayCycles(LED_DIAGNOSTIC_HALF_PERIOD_CYCLES);

        DL_GPIO_clearPins(LED_PORT, LED_D1_PIN);
        DL_GPIO_setPins(LED_PORT, LED_D2_PIN);
        DL_Common_delayCycles(LED_DIAGNOSTIC_HALF_PERIOD_CYCLES);
    }
}
