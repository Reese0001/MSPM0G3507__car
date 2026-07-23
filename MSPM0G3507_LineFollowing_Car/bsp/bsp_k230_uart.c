#include "bsp_k230_uart.h"

void BSP_K230_UartInit(void)
{
    /* Intentionally inert until the final SysConfig integration. */
}

bool BSP_K230_UartRxEmpty(void)
{
    return true;
}

uint8_t BSP_K230_UartReadByte(void)
{
    return 0U;
}
