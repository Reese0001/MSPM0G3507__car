#ifndef __APP_MOTOR_USART_H_
#define __APP_MOTOR_USART_H_

#include <stdbool.h>

#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "../uart/motor_uart.h"

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t


//外部声明区	External declaration area
typedef enum _motor_type  //此类型用做判断死区	This type is used to determine the dead zone
{
	MOTOR_TYPE_NONE = 0x00,       // 保留	reserve
	MOTOR_520 ,       //520电机 包含L型	520 motor including L type
	MOTOR_310 ,       //310电机	310 motor
	MOTOR_TT_Encoder ,//tt电机,带编码器	tt motor with encoder
	MOTOR_TT , 		  // tt电机,不带编码器	tt motor, without encoder

	Motor_TYPE_MAX    // 最后一个电机类型，仅作为判断	The last motor type is for judgment only
} motor_type_t;



//引出编码器变量，供外部使用	Lead out encoder variables for external use
extern int Encoder_Offset[4];
extern int Encoder_Now[4];
extern float g_Speed[4];
extern volatile uint8_t g_recv_flag;
extern volatile bool g_recv_speed_frame;


bool send_motor_type(motor_type_t data);
bool send_motor_deadzone(uint16_t data);
bool send_pulse_line(uint16_t data);
bool send_pulse_phase(uint16_t data);
bool send_wheel_diameter(float data);
void send_motor_PID(float P,float I,float D);
void send_upload_data(bool ALLEncoder_Switch,bool TenEncoder_Switch,bool Speed_Switch);
void Deal_Control_Rxtemp(uint8_t rxtemp);
void Deal_data_real(void);

#endif

