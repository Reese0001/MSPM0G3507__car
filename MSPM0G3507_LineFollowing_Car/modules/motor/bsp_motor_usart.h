#ifndef __BSP_MOTOR_USART_H_
#define __BSP_MOTOR_USART_H_

#include "ti_msp_dl_config.h"
#include "app_motor_usart.h"

#define MOTOR_UART_TX_WAIT_LIMIT (1000U)

void Motor_Usart_init (void);
void Send_Motor_U8(uint8_t Data);
void Send_Motor_ArrayU8(uint8_t *pData, uint16_t Length);
bool Motor_Usart_SendArrayBounded(const uint8_t *data, uint16_t length);
bool Motor_EmergencyStop_FromISR(void);



#endif

