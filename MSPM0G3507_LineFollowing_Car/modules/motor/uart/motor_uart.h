#ifndef __BSP_MOTOR_USART_H_
#define __BSP_MOTOR_USART_H_

#include <stdbool.h>

#include "ti_msp_dl_config.h"

#include <stdint.h>

#define MOTOR_UART_TX_TIMEOUT_US (5000U)

void Motor_Usart_init (void);
void Send_Motor_U8(uint8_t Data);
void Send_Motor_ArrayU8(uint8_t *pData, uint16_t Length);
bool Motor_Usart_SendArrayBounded(const uint8_t *data, uint16_t length);
bool Motor_EmergencyStop_FromISR(void);
void Motor_Usart_Service(uint32_t now_ms);



#endif

