#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdbool.h>
#include <stdint.h>

#define BSP_I2C_MAX_TRANSFER (16U)

bool BSP_I2C_Init(void);
bool BSP_I2C_Read(uint8_t address,
                  uint8_t reg,
                  uint8_t *buffer,
                  uint8_t length);
bool BSP_I2C_Write(uint8_t address,
                   uint8_t reg,
                   const uint8_t *buffer,
                   uint8_t length);

#endif
