#ifndef	_APP_MPU6050_H_
#define _APP_MPU6050_H_

#include "ti_msp_dl_config.h"
#include "bsp_mpu6050.h"
#include "inv_mpu.h"
#include "delay.h"
#include "timer.h"

extern volatile float pitch,roll,yaw;   //??¤???? Euler Angles

extern volatile float yawBias,pitchBias,rollBias;
extern volatile float calibratedYaw, calibratedPitch, calibratedRoll;


typedef struct
{
	float Xoffset;
	float Yoffset;
	float Zoffset;
} Bias_t; 


void Get_EulerAngles(void);
float Dir_PID(float error);
float navigetion_0_360_limit(float angle);
float get_minor_arc(float azimuth,float headingAngle);
void AngleOffsetCalc(void);
void Get_CalibratedAngles(void);


#endif
