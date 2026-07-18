#ifndef	__QUESTIONS_H__
#define __QUESTIONS_H__

#include "ti_msp_dl_config.h"
#include "app_mpu6050.h"
#include "app_motor.h"
#include "app_irtracking.h"
#include "led.h"

extern int odometry_sum;
extern int encoder_odometry_flag;
extern float Servo_error;
extern float PID_Value;


//question state definition
#define  STOP_STATE   0
#define  QUESTION_1   1
#define  QUESTION_2   2
#define  QUESTION_3   3
#define  QUESTION_4   4

//State_Machine
struct state_machine
{
    int Main_State;
	int Q1_State;
	int Q2_State;
	int Q3_State;
	int Q4_State;
};

extern volatile struct state_machine State_Machine;
void State_Machine_init(void);
void Question_Task_1(void);
void Question_Task_2(void);
void Question_Task_3(void);
void Question_Task_4(void);


#endif
