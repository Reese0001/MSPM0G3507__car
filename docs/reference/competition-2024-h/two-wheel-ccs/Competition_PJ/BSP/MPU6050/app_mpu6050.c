#include "app_mpu6050.h"
#include "Filter.h"
#define Pi 3.14159265
volatile short angle[3];
volatile short accel[3];
volatile float pitch=0,roll=0,yaw=0;   //欧拉角 Euler Angles
uint8_t Way_Angle=2; 
Bias_t Angle;

const int CALIB_SAMPLES = 30; // 采样次数  Number of samples
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Acceleration_Z;
// 计算偏移量   Number of samples
volatile float yawBias = 0, pitchBias = 0, rollBias = 0;

volatile float calibratedYaw = 0, calibratedPitch = 0, calibratedRoll = 0;

float AngleOffsetCalc(void)
{
    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        Get_EulerAngles();
//        printf("yaw:%.2f, pitch:%.2f, roll:%.2f\r\n", yaw, pitch, roll);
        calibratedYaw += calibratedYaw;

       
//        delay_ms(20);
    }
//    printf("yawsum:%d\r\n", (int)yawSum);
    // 计算偏移量   Calculate offset
    calibratedYaw = calibratedYaw / CALIB_SAMPLES;

    
//    printf("yawBias:%.2f, pitchBias:%.2f, rollBias:%.2f", yawBias, pitchBias, rollBias);
}
//获取已校准的角度  Get the calibrated angle
void Get_CalibratedAngles(void)
{
    if(yaw < -3)
    {
        calibratedYaw = -yaw;
    }
    else if(yaw >= 3)
    {
        calibratedYaw = 360 - yaw; //车头朝前yaw顺时针为负数  逆时针为正数 yaw clockwise is negative
    }
    else
    {
        calibratedYaw = 0; 	//过滤掉0和360附近的数据  Filter out data near 0 and 360
			
    }
//    calibratedYaw = local_yaw - local_yawBias;
//    calibratedPitch = local_pitch - local_pitchBias;
//    calibratedRoll = local_roll - local_rollBias;
//    printf("hanshu: %d,%d,%d\r\n", (int)local_yawBias, (int)local_pitchBias, (int)local_rollBias);
	   	printf("%3.1f \r\n",calibratedYaw);
}

void Get_EulerAngles(void)
{
    //获取欧拉角 Get Euler angles
    mpu_dmp_get_data(&pitch,&roll,&yaw);

    //delay_ms(20);//根据设置的采样率，不可设置延时过大 According to the set sampling rate, the delay cannot be set too large
}

//角度环PID控制 Angle ring PID control
float dir_kp = 4,dir_ki=0.01,dir_kd = 0.5;
int Integral_Max = 300; //300
int pid_max = 3000; //3000
float Dir_PID(float error)
{ //5*20/1000*188
    float result = 0;
    static int16_t err_last = 0; //上次的误差初始为0  Last error
    static float Integral = 0; // 初始化积分项 Initialize integral term

//   if(error == 0)          
//   {
//       Integral = 0;          //积分清零   Integral cleared
//   }
    Integral += error;           // 更新积分项，并进行限幅 Update the integral term and limit it
    if (Integral > Integral_Max) Integral = Integral_Max;               //积分限幅 Integral limiting
    if (Integral < -Integral_Max) Integral = -Integral_Max;             //积分限幅 Integral limiting

    // 位置式 PID
    result = dir_kp * error +dir_ki*Integral+ dir_kd * (error - err_last);
  
	 err_last = error;       // 更新积分项，并进行限幅 Update the integral term and limit it

    // 对输出进行限幅Output limiting value
    if (result > Integral_Max) result = pid_max;  
    if (result < -Integral_Max) result = -pid_max;

    return -result;
}
//将航向角限制为 0-360 度（防止因加减运算导致航向角范围超过 0-360 度）
//Limit the heading_angle to 0-360 degrees(to prevent the range of heading_angle over 0-360 degrees beacuse of Addition or subtraction operations )
float navigetion_0_360_limit(float angle)
{
		float temp = 0;
		 if(angle > 360)
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


 /**************************************************************************
Function: Get angle
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none
函数功能：获取角度	
入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/	
void Get_Angle(uint8_t way)
{ 
	float gyro_x,gyro_y,gyro_z,accel_z;
	float Accel_Angle_X,Accel_Angle_Y;
	if(way==1)                           //DMP的读取在	数据采集中断读取，严格遵循时序要求
	{	
		if( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 )
        { 
          //  printf("\r\npitch =%d\r\n", (int)pitch);
           // printf("\r\nroll =%d\r\n", (int)roll);
            printf("\r\nyaw =%3.2f\r\n", yaw);
        }      
        delay_ms(10);//根据设置的采样率，不可设置延时过大
	}			
	else
	{
		
		MPU6050ReadGyro(angle);
		MPU6050ReadAcc(accel);
		Accel_Angle_X=atan2(accel[0],accel[2])*180/Pi;     //计算倾角，转换单位为度	
		Accel_Angle_Y=atan2(accel[1],accel[2])*180/Pi;     //计算倾角，转换单位为度
		accel_z=accel[2]*1.962/32768;
		gyro_z = angle[2] * 2000 / 32768; // 陀螺仪量程转换
		delay_ms(10);
		if(Way_Angle==2)		  	
		{
			pitch= -Kalman_Filter_x(Accel_Angle_X,gyro_x);//卡尔曼滤波
			roll = -Kalman_Filter_y(Accel_Angle_Y,gyro_y);
			 printf("\r\nyaw =%3.2f\r\n", yaw);
		}
		else if(Way_Angle==3) 
		{  
			 pitch = -Complementary_Filter_x(Accel_Angle_X,gyro_x);//互补滤波
			 roll = -Complementary_Filter_y(Accel_Angle_Y,gyro_y);
		}
		Angle_Balance=pitch;                              //更新平衡倾角
		Gyro_Turn=angle[2];                                 //更新转向角速度
		Acceleration_Z=accel[2];                           //更新Z轴加速度计
		
	}

} 


