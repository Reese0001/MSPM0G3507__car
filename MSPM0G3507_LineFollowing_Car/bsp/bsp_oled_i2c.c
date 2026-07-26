#include "bsp_oled_i2c.h"

#include "ti_msp_dl_config.h"

#include "time/timer.h"

/* SysConfig requires globally unique pin names, so PA10/PA11 are generated as
   CLK/DAT. Alias them back to SCL/SDA for readability. */
#define OLED_I2C_SCL_PIN   OLED_I2C_CLK_PIN
#define OLED_I2C_SCL_IOMUX OLED_I2C_CLK_IOMUX
#define OLED_I2C_SDA_PIN   OLED_I2C_DAT_PIN
#define OLED_I2C_SDA_IOMUX OLED_I2C_DAT_IOMUX

#define OLED_I2C_HALF_PERIOD_US (3U)
/* Clock-stretch / arbitration guard for one bit. */
#define OLED_I2C_BIT_TIMEOUT_US (50U)

static void short_delay(void)
{
    uint32_t started_us = BSP_Time_GetUs();

    while ((uint32_t)(BSP_Time_GetUs() - started_us) <
           OLED_I2C_HALF_PERIOD_US) {
    }
}

static void scl_low(void)
{
    DL_GPIO_initDigitalOutput(OLED_I2C_SCL_IOMUX);
    DL_GPIO_clearPins(OLED_I2C_PORT, OLED_I2C_SCL_PIN);
    DL_GPIO_enableOutput(OLED_I2C_PORT, OLED_I2C_SCL_PIN);
}

static void scl_release(void)
{
    DL_GPIO_disableOutput(OLED_I2C_PORT, OLED_I2C_SCL_PIN);
    DL_GPIO_initDigitalInput(OLED_I2C_SCL_IOMUX);
}

static void sda_low(void)
{
    DL_GPIO_initDigitalOutput(OLED_I2C_SDA_IOMUX);
    DL_GPIO_clearPins(OLED_I2C_PORT, OLED_I2C_SDA_PIN);
    DL_GPIO_enableOutput(OLED_I2C_PORT, OLED_I2C_SDA_PIN);
}

static void sda_release(void)
{
    DL_GPIO_disableOutput(OLED_I2C_PORT, OLED_I2C_SDA_PIN);
    DL_GPIO_initDigitalInput(OLED_I2C_SDA_IOMUX);
}

static bool sda_is_high(void)
{
    return (DL_GPIO_readPins(OLED_I2C_PORT, OLED_I2C_SDA_PIN) &
            OLED_I2C_SDA_PIN) != 0U;
}

static void start_condition(void)
{
    sda_release();
    scl_release();
    short_delay();
    sda_low();
    short_delay();
    scl_low();
    short_delay();
}

static void stop_condition(void)
{
    sda_low();
    short_delay();
    scl_release();
    short_delay();
    sda_release();
    short_delay();
}

static bool write_byte(uint8_t value)
{
    uint8_t bit;
    bool acked;

    for (bit = 0U; bit < 8U; bit++) {
        if ((value & 0x80U) != 0U) {
            sda_release();
        } else {
            sda_low();
        }
        value = (uint8_t)(value << 1);
        short_delay();
        scl_release();
        short_delay();
        scl_low();
    }
    /* Ninth clock: release SDA and sample the ACK. */
    sda_release();
    short_delay();
    scl_release();
    short_delay();
    acked = !sda_is_high();
    scl_low();
    short_delay();
    return acked;
}

void BSP_OledI2C_Init(void)
{
    scl_release();
    sda_release();
}

bool BSP_OledI2C_Write(uint8_t address_7bit,
                       const uint8_t *data,
                       uint16_t length)
{
    uint16_t index;

    if (data == 0 && length != 0U) {
        return false;
    }
    start_condition();
    if (!write_byte((uint8_t)(address_7bit << 1))) {
        stop_condition();
        return false;
    }
    for (index = 0U; index < length; index++) {
        if (!write_byte(data[index])) {
            stop_condition();
            return false;
        }
    }
    stop_condition();
    return true;
}
