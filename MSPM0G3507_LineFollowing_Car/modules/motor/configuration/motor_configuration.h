#ifndef __APP_MOTOR_H_
#define __APP_MOTOR_H_

#include "ti_msp_dl_config.h"
#include "../protocol/motor_protocol.h"

//底盘电机间距之和的一半    Half of the sum of the motor spacing between the chassis
#define Car_APB          				(188.0f)//  (228+148)/2

bool Set_Motor(int MOTOR_TYPE);
void Motion_Car_Control(int16_t V_x, int16_t V_y, int16_t V_z);
void Motion_Yaw_Calc(float offset_yaw);
void Get_Odometry(void);



#endif
