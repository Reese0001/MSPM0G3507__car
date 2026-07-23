#include "bsp_i2c.h"

bool BSP_I2C_Init(void)
{
    return false;
}

bool BSP_I2C_Read(uint8_t address,
                  uint8_t reg,
                  uint8_t *buffer,
                  uint8_t length)
{
    if (address == 0U || buffer == 0 || length == 0U ||
        length > BSP_I2C_MAX_TRANSFER) {
        return false;
    }

    (void)reg;
    return false;
}

bool BSP_I2C_Write(uint8_t address,
                   uint8_t reg,
                   const uint8_t *buffer,
                   uint8_t length)
{
    if (address == 0U || buffer == 0 || length == 0U ||
        length > BSP_I2C_MAX_TRANSFER) {
        return false;
    }

    (void)reg;
    return false;
}
