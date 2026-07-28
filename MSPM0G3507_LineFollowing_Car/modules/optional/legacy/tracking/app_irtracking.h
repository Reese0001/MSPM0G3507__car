#ifndef _APP_IRTRACKING_H_
#define _APP_IRTRACKING_H_

#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "../../../motor/configuration/motor_configuration.h"
#include "debug_uart.h"

#define BLACK       1        //黑线black
#define WHITE       0        //白线white


#define u8 uint8_t
#define u16 uint16_t

// 八路灰度传感器引脚定义 (地址选择)
#define GRAY_AD0_PORT   GPIOA
#define GRAY_AD0_PIN    DL_GPIO_PIN_15

#define GRAY_AD1_PORT   GPIOA
#define GRAY_AD1_PIN    DL_GPIO_PIN_16

#define GRAY_AD2_PORT   GPIOA
#define GRAY_AD2_PIN    DL_GPIO_PIN_17

// 八路灰度传感器数据输出引脚
#define GRAY_OUT_PORT   GPIOA
#define GRAY_OUT_PIN    DL_GPIO_PIN_18

void Gray_SelectChannel(uint8_t channel);
uint8_t Gray_ReadChannel(uint8_t channel);
void Gray_ReadAll(uint8_t *x1, uint8_t *x2, uint8_t *x3, uint8_t *x4,
                  uint8_t *x5, uint8_t *x6, uint8_t *x7, uint8_t *x8);
bool Tracking_ComputeWeightedError(uint8_t sensor_bits, float *error);
void printf_gray_data(void);
void LineWalking(void);
int LineCheck(void);

#endif
