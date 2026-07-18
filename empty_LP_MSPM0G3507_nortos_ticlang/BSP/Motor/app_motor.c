#include "app_motor.h"

static float speed_lr = 0;
static float speed_fb = 0;
static float speed_spin = 0;
static int speed_L1_setup = 0;
static int speed_L2_setup = 0;
static int speed_R1_setup = 0;
static int speed_R2_setup = 0;

// ���ص�ǰС����������͵�һ��   Returns half of the current small wheel axle spacing
static float Motion_Get_APB(void)
{
    return Car_APB;
}

void Set_Motor(int MOTOR_TYPE)
{
    if(MOTOR_TYPE == 1)
    {
        send_motor_type(1);//���õ������	Configure motor type
        delay_ms(100);
        send_pulse_phase(30);//���ü��ٱ� �����ֲ�ó�	Configure the reduction ratio. Check the motor manual to find out
        delay_ms(100);
        send_pulse_line(11);//���ôŻ��� �����ֲ�ó�	Configure the magnetic ring wire. Check the motor manual to get the result.
        delay_ms(100);
        send_wheel_diameter(67.00);//��������ֱ��,�����ó�		Configure the wheel diameter and measure it
        delay_ms(100);
        send_motor_deadzone(1900);//���õ������,ʵ��ó�	Configure the motor dead zone, and the experiment shows
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

//����С�����˶�    Control the movement of the car (2WD version)
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

    Contrl_Speed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);

}

// ͨ��ƫ���Ǽ��㵱ǰ��ƫ��ֵ��У׼С���˶�����   Calculate the current deviation value by yaw angle and calibrate the direction of the carriage movement.
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
    Contrl_Speed(speed_L1, speed_L2, speed_R1, speed_R2);
}

//��ȡĸ������ƽ����10ms�ı��������ݣ��ۼ���������ȡ���ֵ
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


