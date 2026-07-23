#include "app_mpu6050.h"

volatile float pitch=0,roll=0,yaw=0;   //欧拉角 Euler Angles

Bias_t Angle;

const int CALIB_SAMPLES = 500; // 采样次数  Number of samples

// 计算偏移量   Number of samples
volatile float yawBias = 0, pitchBias = 0, rollBias = 0;

volatile float calibratedYaw = 0, calibratedPitch = 0, calibratedRoll = 0;

void AngleOffsetCalc(void)
{
    float yawSum = 0, pitchSum = 0, rollSum = 0;
    delay_ms(7000);
    delay_ms(7000);
    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        Get_EulerAngles();
//        printf("yaw:%.2f, pitch:%.2f, roll:%.2f\r\n", yaw, pitch, roll);
        yawSum += yaw;
        pitchSum += pitch;
        rollSum += roll;
        
//        delay_ms(20);
    }
//    printf("yawsum:%d\r\n", (int)yawSum);
    // 计算偏移量   Calculate offset
    yawBias = yawSum / CALIB_SAMPLES;
    pitchBias = pitchSum / CALIB_SAMPLES;
    rollBias = rollSum / CALIB_SAMPLES;
    
//    printf("yawBias:%.2f, pitchBias:%.2f, rollBias:%.2f", yawBias, pitchBias, rollBias);
}
//获取已校准的角度  Get the calibrated angle
void Get_CalibratedAngles(void)
{
    if(yaw < 0)
    {
        calibratedYaw = -yaw;
    }
    else if(yaw >= 0)
    {
        calibratedYaw = 360 - yaw;
    }
    
//    calibratedYaw = local_yaw - local_yawBias;
//    calibratedPitch = local_pitch - local_pitchBias;
//    calibratedRoll = local_roll - local_rollBias;
//    printf("hanshu: %d,%d,%d\r\n", (int)local_yawBias, (int)local_pitchBias, (int)local_rollBias);

}

void Get_EulerAngles(void)
{
    //获取欧拉角 Get Euler angles
    float p = pitch;
    float r = roll;
    float y = yaw;

    if (mpu_dmp_get_data(&p, &r, &y) == 0U) {
        pitch = p;
        roll = r;
        yaw = y;
    }
}



//角度环PID控制 Angle ring PID control
float dir_kp = 5,dir_kd = 0;
float last_error = 0;

float Dir_PID(float error)
{
	float result = 0;
	result = dir_kp * error + dir_kd * (error - last_error);
	last_error = error;
	return result;
}

//将航向角限制为 0-360 度（防止因加减运算导致航向角范围超过 0-360 度）
//Limit the heading_angle to 0-360 degrees(to prevent the range of heading_angle over 0-360 degrees beacuse of Addition or subtraction operations )
float navigetion_0_360_limit(float angle)
{
		float temp = 0;
		if(angle < 0)
		{
			temp = angle + 360;
		}
		else if(angle > 360)
		{
			temp = angle - 360;
		}
		else
		{
			temp = angle;
		}
		return temp;
}


//计算 0-360 导航坐标系中的小圆弧偏差（逆时针方向的负圆弧角度为正，顺时针方向的为负）
//Calculate the minor arc deviation in the 0-360 Navigation Coordinate System (Counterclockwise negative arc Angle is Positive, Clockwise is Negative)  
float get_minor_arc(float azimuth,float headingAngle)
{
    float angle_err = 0.0;
    if(azimuth >= 180 + headingAngle)
    {
        angle_err = azimuth - headingAngle - 360;
    }
    else if(headingAngle > 180 + azimuth)
    {
        angle_err = azimuth - headingAngle + 360;
    }
    else
    {
        angle_err =  azimuth - headingAngle;
    }

    return -angle_err;
}
