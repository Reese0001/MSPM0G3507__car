#include "bsp_k230_uart.h"

#include "ti_msp_dl_config.h"

#include "../modules/optional/k230/k230_link.h"

void BSP_K230_UartInit(void)
{
    /*
     * SysConfig has already configured UART2 and its RX source.  Enable the
     * NVIC only after the software ring buffer has been reset by K230Link_Init.
     */
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);
}

bool BSP_K230_UartRxEmpty(void)
{
    return DL_UART_Main_isRXFIFOEmpty(K230_INST);
}

uint8_t BSP_K230_UartReadByte(void)
{
    return DL_UART_Main_receiveData(K230_INST);
}

void K230_INST_IRQHandler(void)
{
    K230_UART_IRQHandler();
}
