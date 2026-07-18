#include "bsp_motor_usart.h"

// 电机串口初始化
void Motor_Usart_init(void)
{
    // 使能UART1中断
    NVIC_ClearPendingIRQ(Motor_INST_INT_IRQN);
    NVIC_EnableIRQ(Motor_INST_INT_IRQN);
}

/************************************************
函数名称： Send_Motor_U8		Function name: Send_Motor_U8
功    能： USART1发送一个字符	Function: USART1 sends a character
参    数： Data --- 数据		Parameter: Data --- data
返 回 值： 无					Return value: None
*************************************************/
void Send_Motor_U8(uint8_t Data)
{
	while( DL_UART_isBusy(Motor_INST) == true );
	DL_UART_Main_transmitData(Motor_INST, Data);
}

/************************************************
函数名称： Send_Motor_ArrayU8	Function name: Send_Motor_ArrayU8
功    能： 串口1发送N个字符		Function: Serial port 1 sends N characters
参    数： pData ---- 字符串	Parameter: pData ---- string
            Length --- 长度		Length --- length
返 回 值： 无					Return value: None
*************************************************/
void Send_Motor_ArrayU8(uint8_t *pData, uint16_t Length)
{
	while (Length--)
	{
		Send_Motor_U8(*pData);
		pData++;
	}
}

/* 中断失控保护：固定零速帧，等待次数有上限，禁止格式化和无限阻塞。 */
void Motor_EmergencyStop_FromISR(void)
{
    static const uint8_t stop_frame[] = "$spd:0,0,0,0#";
    uint16_t index;
    for (index = 0U; index < (uint16_t)(sizeof(stop_frame) - 1U); index++) {
        uint32_t wait_count = 1000U;
        while ((DL_UART_isBusy(Motor_INST) == true) && (wait_count > 0U)) {
            wait_count--;
        }
        if (wait_count == 0U) return;
        DL_UART_Main_transmitData(Motor_INST, stop_frame[index]);
    }
}


/*  串口中断接收处理 */
/* Serial port interrupt reception processing */
void Motor_INST_IRQHandler(void)
{
	uint8_t Rx2_Temp = 0;

	switch( DL_UART_getPendingInterrupt(Motor_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断	If it is a receive interrupt
			// 接收发送过来的数据保存	Receive and save the data sent
			Rx2_Temp = DL_UART_Main_receiveData(Motor_INST);
			//处理	deal with
			Deal_Control_Rxtemp(Rx2_Temp);
			break;

		default://其他的串口中断	Other serial port interrupts
			break;
	}


}
