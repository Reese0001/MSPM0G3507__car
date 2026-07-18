#include "app_motor.h"

static float speed_lr = 0;
static float speed_fb = 0;
static float speed_spin = 0;
static int speed_L1_setup = 0;
static int speed_L2_setup = 0;
static int speed_R1_setup = 0;
static int speed_R2_setup = 0;

// 返回当前小车轮子轴间距和的一半   Returns half of the current small wheel axle spacing
static float Motion_Get_APB(void)
{
    return Car_APB;
}

void Set_Motor(int MOTOR_TYPE)
{
    if(MOTOR_TYPE == 1)
    {
        send_motor_type(1);//配置电机类型	Configure motor type
        delay_ms(100);
        send_pulse_phase(30);//配置减速比 查电机手册得出	Configure the reduction ratio. Check the motor manual to find out
        delay_ms(100);
        send_pulse_line(11);//配置磁环线 查电机手册得出	Configure the magnetic ring wire. Check the motor manual to get the result.
        delay_ms(100);
        send_wheel_diameter(67.00);//配置轮子直径,测量得出		Configure the wheel diameter and measure it
        delay_ms(100);
        send_motor_deadzone(1900);//配置电机死区,实验得出	Configure the motor dead zone, and the experiment shows
        delay_ms(100);
    }
    
    else if(MOTOR_TYPE == 2)
    {
        send_motor_type(2);
        delay_ms(100);
        send_pulse_phase(20);
        delay_ms(100);
        send_pulse_line(13);
        delay_ms(100);
        send_wheel_diameter(48.00);
        delay_ms(100);
        send_motor_deadzone(1600);
        delay_ms(100);
    }
    
    else if(MOTOR_TYPE == 3)
    {
        send_motor_type(3);
        delay_ms(100);
        send_pulse_phase(45);
        delay_ms(100);
        send_pulse_line(13);
        delay_ms(100);
        send_wheel_diameter(68.00);
        delay_ms(100);
        send_motor_deadzone(1600);
        delay_ms(100);
    }
    
    else if(MOTOR_TYPE == 4)
    {
        send_motor_type(4);
        delay_ms(100);
        send_pulse_phase(48);
        delay_ms(100);
        send_motor_deadzone(1000);
        delay_ms(100);
    }
    
    else if(MOTOR_TYPE == 5)
    {
        send_motor_type(1);
        delay_ms(100);
        send_pulse_phase(40);
        delay_ms(100);
        send_pulse_line(11);               
        delay_ms(100);
        send_wheel_diameter(67.00);
        delay_ms(100);
        send_motor_deadzone(1900);
        delay_ms(100);
    }
}

//控制小车的运动    Control the movement of the car
void Motion_Car_Control(int16_t V_x, int16_t V_y, int16_t V_z)
{
	float robot_APB = Motion_Get_APB();
	speed_lr = 0;
    speed_fb = V_x;
    speed_spin = (V_z / 1000.0f) * robot_APB;
    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Contrl_Speed(0,0,0,0);
        return;
    }

   // speed_L1_setup = speed_fb + speed_spin;
    speed_L2_setup = speed_fb + speed_spin;
    //speed_R1_setup = speed_fb  - speed_spin;
    speed_R2_setup = speed_fb  - speed_spin;
		
//    if (speed_L1_setup > 1000) speed_L1_setup = 1000;
//    if (speed_L1_setup < -1000) speed_L1_setup = -1000;
    if (speed_L2_setup > 1000) speed_L2_setup = 1000;
    if (speed_L2_setup < -1000) speed_L2_setup = -1000;
    if (speed_R1_setup > 1000) speed_R1_setup = 1000;
    if (speed_R1_setup < -1000) speed_R1_setup = -1000;
    if (speed_R2_setup > 1000) speed_R2_setup = 1000;
    if (speed_R2_setup < -1000) speed_R2_setup = -1000;
    
    //printf("%d\t,%d\t,%d\t,%d\r\n",speed_L1_setup,speed_L2_setup,speed_R1_setup,speed_R2_setup);
    
    Contrl_Speed(0, speed_L2_setup, 0, speed_R2_setup);
		
}

// 通过偏航角计算当前的偏差值，校准小车运动方向。   Calculate the current deviation value by yaw angle and calibrate the direction of the carriage movement.
void Motion_Yaw_Calc(float offset_yaw)
{
    //int speed_L1 = speed_L1_setup - (int)offset_yaw;
    int speed_L2 = speed_L2_setup + (int)offset_yaw;  //yaw为负 右转
    //int speed_R1 = speed_R1_setup + (int)offset_yaw;
    int speed_R2 = speed_R2_setup - (int)offset_yaw;
        
    //if (speed_L1 > 1000) speed_L1 = 1000;
    //if (speed_L1 < -1000) speed_L1 = -1000;
    if (speed_L2 > 1000) speed_L2 = 1000;	
    if (speed_L2 < -1000) speed_L2 = -1000;
    //if (speed_R1 > 1000) speed_R1 = 1000;
    //if (speed_R1 < -1000) speed_R1 = -1000;
    if (speed_R2 > 1000) speed_R2 = 1000;
    if (speed_R2 < -1000) speed_R2 = -1000;
    Contrl_Speed(0, speed_L2, 0, speed_R2);
}

//获取两个个电机的平均的10ms的编码器数据，累计增加来获取里程值
//Get the average encoder data of four motors and add the cumulatively to get the mileage value
void Get_Odometry(void)
{
    if(encoder_odometry_flag)
    {
		float average_speed = 0;
     Deal_data_real();
       //odometry_sum += (Encoder_Offset[1] + Encoder_Offset[3])/2;
		average_speed = ((g_Speed[1] + g_Speed[3]) / 2);  //mm/s
		odometry_sum+=average_speed;     //mm
    //odometry_sum += ((Encoder_Offset[0] + Encoder_Offset[1] + Encoder_Offset[2] + Encoder_Offset[3]) / 4);
		//	printf("%f  ",odometry_sum);
    }
}


