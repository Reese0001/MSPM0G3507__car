#ifndef BSP_K230_UART_H
#define BSP_K230_UART_H

#include <stdbool.h>
#include <stdint.h>

/* Zero-hardware adapter. Replace only after PA21/PA22 are assigned in SysConfig. */
void BSP_K230_UartInit(void);
bool BSP_K230_UartRxEmpty(void);
uint8_t BSP_K230_UartReadByte(void);

#endif
