#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdbool.h>
#include <stdint.h>

#define BSP_I2C_MAX_TRANSFER (16U)
#define BSP_I2C_TRANSACTION_TIMEOUT_US (10000U)

typedef enum {
    BSP_I2C_STATUS_IDLE = 0,
    BSP_I2C_STATUS_BUSY,
    BSP_I2C_STATUS_DONE,
    BSP_I2C_STATUS_ERROR
} BSP_I2C_Status;

bool BSP_I2C_Init(void);
bool BSP_I2C_BeginRead(uint8_t address,
                       uint8_t reg,
                       uint8_t *buffer,
                       uint8_t length);
bool BSP_I2C_BeginWrite(uint8_t address,
                        uint8_t reg,
                        const uint8_t *buffer,
                        uint8_t length);
void BSP_I2C_Service(uint32_t now_us);
BSP_I2C_Status BSP_I2C_GetStatus(void);

#endif
