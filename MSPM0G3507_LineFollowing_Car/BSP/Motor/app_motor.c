#include "app_motor.h"
#include "motor_safety.h"

static float speed_lr = 0;
static float speed_fb = 0;
static float speed_spin = 0;
static int speed_L1_setup = 0;
static int speed_L2_setup = 0;
static int speed_R1_setup = 0;
static int speed_R2_setup = 0;

// 返回当前小车轮距的一半   Returns half of the current small wheel axle spacing
static float Motion_Get_APB(void)
{
    return Car_APB;
}

void Set_Motor(int MOTOR_TYPE)
{
    if(MOTOR_TYPE == 1)
    {
        send_motor_type(1);//设置电机类型	Configure motor type
        delay_ms(100);
        send_pulse_phase(30);//设置减速比 查看电机手册得出	Configure the reduction ratio. Check the motor manual to find out
        delay_ms(100);
        send_pulse_line(11);//设置磁环线 查看电机手册得出	Configure the magnetic ring wire. Check the motor manual to get the result.
        delay_ms(100);
        send_wheel_diameter(67.00);//设置轮子直径,实际测出		Configure the wheel diameter and measure it
        delay_ms(100);
        send_motor_deadzone(1900);//设置电机死区,实验测出	Configure the motor dead zone, and the experiment shows
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

//控制小车的运动    Control the movement of the car (2WD version)
void Motion_Car_Control(int16_t V_x, int16_t V_y, int16_t V_z)
{
	float robot_APB = Motion_Get_APB();
	speed_lr = 0;
    speed_fb = V_x;
    speed_spin = (V_z / 1000.0f) * robot_APB;
    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Motor_Safety_RequestSpeed(0, 0, 0, 0);
        return;
    }

    // 2WD: M1=0(castor), M2=left rear(driven), M3=0(castor), M4=right rear(driven)
    speed_L1_setup = 0;  // M1 - castor wheel, no motor
    speed_L2_setup = speed_fb + speed_spin;  // M2 - left rear driven wheel
    speed_R1_setup = 0;  // M3 - castor wheel, no motor
    speed_R2_setup = speed_fb - speed_spin;  // M4 - right rear driven wheel

    if (speed_L1_setup > 1000) speed_L1_setup = 1000;
    if (speed_L1_setup < -1000) speed_L1_setup = -1000;
    if (speed_L2_setup > 1000) speed_L2_setup = 1000;
    if (speed_L2_setup < -1000) speed_L2_setup = -1000;
    if (speed_R1_setup > 1000) speed_R1_setup = 1000;
    if (speed_R1_setup < -1000) speed_R1_setup = -1000;
    if (speed_R2_setup > 1000) speed_R2_setup = 1000;
    if (speed_R2_setup < -1000) speed_R2_setup = -1000;

    //printf("%d\t,%d\t,%d\t,%d\r\n",speed_L1_setup,speed_L2_setup,speed_R1_setup,speed_R2_setup);

    Motor_Safety_RequestSpeed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);

}

// 通过偏航角计算当前的偏航值，校准小车运动方向   Calculate the current deviation value by yaw angle and calibrate the direction of the carriage movement.
void Motion_Yaw_Calc(float offset_yaw)
{
    int speed_L1 = 0;  // M1 - castor wheel, no motor
    int speed_L2 = speed_L2_setup - (int)offset_yaw;  // M2 - left rear driven wheel
    int speed_R1 = 0;  // M3 - castor wheel, no motor
    int speed_R2 = speed_R2_setup + (int)offset_yaw;  // M4 - right rear driven wheel

    if (speed_L1 > 1000) speed_L1 = 1000;
    if (speed_L1 < -1000) speed_L1 = -1000;
    if (speed_L2 > 1000) speed_L2 = 1000;
    if (speed_L2 < -1000) speed_L2 = -1000;
    if (speed_R1 > 1000) speed_R1 = 1000;
    if (speed_R1 < -1000) speed_R1 = -1000;
    if (speed_R2 > 1000) speed_R2 = 1000;
    if (speed_R2 < -1000) speed_R2 = -1000;
    Motor_Safety_RequestSpeed(speed_L1, speed_L2, speed_R1, speed_R2);
}

//获取四个编码器的平均10ms的编码器数据，累加编码值获取里程值
//Get the average encoder data of driven motors and add the cumulatively to get the mileage value (2WD version)
void Get_Odometry(void)
{
    if(encoder_odometry_flag)
    {
        Deal_data_real();
        // 2WD: Only use M2 (Encoder_Offset[1]) and M4 (Encoder_Offset[3]) for odometry
        odometry_sum += ((Encoder_Offset[1] + Encoder_Offset[3]) / 2);
    }
}


