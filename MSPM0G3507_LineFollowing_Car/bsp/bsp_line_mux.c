#include "bsp_line_mux.h"

#include <ti/devices/msp/m0p/mspm0g350x.h>
#include <ti/driverlib/dl_gpio.h>

#include "../modules/line_tracking/line_tracking_config.h"

#define LINE_MUX_AD0_PIN DL_GPIO_PIN_15
#define LINE_MUX_AD1_PIN DL_GPIO_PIN_16
#define LINE_MUX_AD2_PIN DL_GPIO_PIN_17
#define LINE_MUX_OUT_PIN DL_GPIO_PIN_18

void BSP_LineMux_SelectChannel(uint8_t channel)
{
    const uint32_t address_mask =
        LINE_MUX_AD0_PIN | LINE_MUX_AD1_PIN | LINE_MUX_AD2_PIN;
    const uint32_t address_value = ((uint32_t)(channel & 0x07U)) << 15U;

    /* Update AD0..AD2 together so a service step performs one GPIO write. */
    DL_GPIO_writePinsVal(GPIOA, address_mask, address_value);
}

bool BSP_LineMux_IsBlack(void)
{
    const uint32_t level =
        DL_GPIO_readPins(GPIOA, LINE_MUX_OUT_PIN) == 0U ? 0U : 1U;
    return level == LINE_SENSOR_BLACK_ACTIVE_LEVEL;
}
