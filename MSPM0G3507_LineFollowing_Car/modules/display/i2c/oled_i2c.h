#ifndef BSP_OLED_I2C_H
#define BSP_OLED_I2C_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Blocking open-drain software I2C on PA10 (SCL) / PA11 (SDA) for the
 * SSD1306 diagnostic display. Runs only inside the lowest-priority
 * display task, so bit-banging may be preempted at any time. A stuck
 * or absent display only fails the transfer; it never affects motion.
 */

void BSP_OledI2C_Init(void);
bool BSP_OledI2C_Write(uint8_t address_7bit,
                       const uint8_t *data,
                       uint16_t length);

#endif
