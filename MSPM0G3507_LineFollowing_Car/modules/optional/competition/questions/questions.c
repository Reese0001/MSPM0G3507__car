#include "questions.h"

/* Optional legacy contest state machine. Not linked by the active firmware. */
#include "key.h"
#include "buzzer.h"
#include "motor_safety.h"

float object_angle = 0;                    //The target heading_Angle of the car movement(车辆运动的目标航向角)
float Angle_error = 0;
int current_angle = 0;
int odometry_sum = 0;                      //Odometer value(里程计数值)
float first_angle = 0;
int encoder_odometry_flag = 0;
float PID_Value = 0;

//The second level question state definition
#define  Q2_STATE_1   1
#define  Q2_STATE_2   2
#define  Q2_STATE_3   3
#define  Q2_STATE_4   4
#define  Q2_STATE_5   5
#define  Q3_STATE_1   1
#define  Q3_STATE_2   2
#define  Q3_STATE_3   3
#define  Q3_STATE_4   4
#define  Q3_STATE_5   5

volatile struct state_machine State_Machine;

int q1_first_flag = 0;
int q2_first_flag = 0;
int q3_first_flag = 0;
int q4_first_flag = 0;
int q4_circle_num = 0;


void State_Machine_init(void)
{
	State_Machine.Main_State = STOP_STATE;
	State_Machine.Q1_State = STOP_STATE;
	State_Machine.Q2_State = STOP_STATE;
	State_Machine.Q3_State = STOP_STATE;
	State_Machine.Q4_State = STOP_STATE;
}

void Legacy_Questions_HandleKey(void)
{
    static int task_flag = 1;

    switch (Key_PollEvent()) {
        case KEY_EVENT_SHORT:
            g_LinePortal_flag = 1;
            bee_time = 500;
            break;
        case KEY_EVENT_LONG:
            Motor_Safety_Disarm();
            g_LinePortal_flag = 0;
            switch (task_flag) {
                case 1:
                    State_Machine.Main_State = QUESTION_1;
                    task_flag += 1;
                    Buzzer_RequestBeeps(1U);
                    break;
                case 2:
                    State_Machine.Main_State = QUESTION_2;
                    task_flag += 1;
                    Buzzer_RequestBeeps(2U);
                    break;
                case 3:
                    State_Machine.Main_State = QUESTION_3;
                    task_flag += 1;
                    Buzzer_RequestBeeps(3U);
                    break;
                case 4:
                    State_Machine.Main_State = QUESTION_4;
                    task_flag = 1;
                    Buzzer_RequestBeeps(4U);
                    break;
                default:
                    task_flag = 1;
                    break;
            }
            break;
        default:
            break;
    }
}


void Question_Task_1(void)
{
	if(q1_first_flag == 0)
	{
		//记录目标角度	Record target angle
		object_angle = calibratedYaw;
	
		q1_first_flag = 1;
        //开始计算里程数	Start calculating mileage
		encoder_odometry_flag = 1;
	}
					
    Motion_Car_Control(300,0,0);
	//计算航向偏差	Calculate heading deviation
	Angle_error = get_minor_arc(object_angle,calibratedYaw);
    PID_Value = Dir_PID(Angle_error);
    Motion_Yaw_Calc(PID_Value);
    
	//检测到黑线，并且里程计数值大于等于1200时进入下一个状态	When a black line is detected and the mileage count value is greater than or equal to 1200, enter the next state
	if((LineCheck() == BLACK) && odometry_sum >= 1200)
	{
		bee_time = 400;
		State_Machine.Main_State = STOP_STATE;
                
		//问题1的首次运行标志位清零	The first run flag of question 1 is cleared
		q1_first_flag = 0;
        //停止计算里程数并清零	Stop counting mileage and reset to zero
		encoder_odometry_flag = 0;
        odometry_sum = 0;
	}
}



void Question_Task_2(void)
{
	if(q2_first_flag == 0)
	{
		//记录目标角度	Record target angle
		object_angle = calibratedYaw;
		//记录第一次发车的角度	Record the angle of the first start
		first_angle = calibratedYaw;
        //开始计算里程数	Start calculating mileage
		encoder_odometry_flag = 1;
		//开始问题2的小状态机	Small state machine to start problem 2
		State_Machine.Q2_State = Q2_STATE_1;
		
		q2_first_flag = 1;
	}
	if(State_Machine.Q2_State == Q2_STATE_1)  //问题2的状态1(跑直)	Problem 2, state 1 (running straight)
	{
		//跑直	running straight
		Angle_error = get_minor_arc(object_angle,calibratedYaw);
		Motion_Car_Control(400,0,Angle_error);
        

		//检测到黑线，并且里程计数值大于等于1200时进入下一个状态	When a black line is detected and the mileage count value is greater than or equal to 1200, enter the next state
		if((LineCheck() == BLACK) && odometry_sum >= 1200)
		{
			bee_time = 300;
            //停止计算里程数并清零	Stop counting mileage and reset to zero
            encoder_odometry_flag = 0;
            odometry_sum = 0;
			//进入问题2的下一个状态	Go to the next state of question 2
			State_Machine.Q2_State = Q2_STATE_2;
		}
	}
	else if(State_Machine.Q2_State == Q2_STATE_2) // 问题2的状态2(右转灰度循迹)	Problem 2, state 2 (right turn grayscale tracking)
	{
		//右转优先的灰度循迹	Grayscale tracking with right turn priority
		LineWalking();
				
		//检测到白线，同时角度在设定的范围内时，进入下一个状态	When the white line is detected and the angle is within the set range, enter the next state
		if((calibratedYaw > 140 && calibratedYaw < 200) && LineCheck() == WHITE)
		{
			bee_time = 300;
			//进入问题2的下一个状态	Go to the next state of question 2
			State_Machine.Q2_State = Q2_STATE_3;
		}
	}
	else if(State_Machine.Q2_State == Q2_STATE_3)  //问题2的状态3(跑直)	Problem 2, state 3 (running straight)
	{
		//开始计算里程数	Start calculating mileage
		encoder_odometry_flag = 1;
        Motion_Car_Control(300,0,0);
		//获取目标角度	Get the target angle
		object_angle = navigetion_0_360_limit(first_angle + 160);
        
		Angle_error = get_minor_arc(object_angle,calibratedYaw);
        PID_Value = Dir_PID(Angle_error);
		
		//检测到黑线，并且里程计数值大于等于1200时进入下一个状态	When a black line is detected and the mileage count value is greater than or equal to 1200, enter the next state
		if(LineCheck() == BLACK && odometry_sum >= 1200)
		{
			bee_time = 300;
            //停止计算里程数并清零	Stop calculating mileage and clear
            encoder_odometry_flag = 0;
            odometry_sum = 0;
			//进入问题2的下一个状态	Enter the next state of question 2
			State_Machine.Q2_State = Q2_STATE_4;
		}
	}
	else if(State_Machine.Q2_State == Q2_STATE_4)  //问题2的状态3(右转灰度循迹)	State 3 of question 2 (right turn grayscale tracking)
	{
		LineWalking();
		
		//检测到白线，同时角度在设定的范围内时，进入下一个状态	When the white line is detected and the angle is within the set range, it enters the next state
		if((calibratedYaw > 290 ||  calibratedYaw < 90) &&  LineCheck() == WHITE)
		{
			bee_time = 300;
			//进入问题2的下一个状态	Enter the next state of question 2
			State_Machine.Q2_State = Q2_STATE_5;
		}
	}
	else
	{
		//退出当前题目	Exit the current question
		State_Machine.Q2_State = STOP_STATE;
		State_Machine.Main_State = STOP_STATE;
		//问题2的运行标志位清零	The running flag bit of question 2 is cleared
		q2_first_flag = 0;
	}
}


void Question_Task_3(void)
{
	if(q3_first_flag == 0)
	{
		//记录第一次发车的角度	Record the angle of the first departure
		first_angle = calibratedYaw;
        //开始计算里程数	Start calculating mileage
		encoder_odometry_flag = 1;
		//声光提示	Sound and light prompts
		bee_time = 300;
		//开始问题3的小状态机	Start the small state machine of problem 3
		State_Machine.Q3_State = Q3_STATE_1;
		
		q3_first_flag = 1;
	}
	if(State_Machine.Q3_State == Q3_STATE_1)
	{		
		//设定斜切目标角度	Set the beveled target angle
		object_angle = navigetion_0_360_limit(first_angle + 32);

        Motion_Car_Control(300,0,0);
		Angle_error = get_minor_arc(object_angle,calibratedYaw);
        PID_Value = Dir_PID(Angle_error);
        Motion_Yaw_Calc(PID_Value);
        
		if(LineCheck() == BLACK && odometry_sum >= 1000)   
		{
			bee_time = 300;
            //停止计算里程数并清零	Stop calculating mileage and clear
            encoder_odometry_flag = 0;
            odometry_sum = 0;
			//进入问题3的下一个状态	Enter the next state of question 3
			State_Machine.Q3_State = Q3_STATE_2;
		
		 }
	}
	else if(State_Machine.Q3_State == Q3_STATE_2)   //问题3的状态2(左转灰度循迹)	State 2 of question 3 (left turn grayscale tracking)
	{
		LineWalking();

		//检测到白线，同时角度在设定的范围内时，进入下一个状态	When the white line is detected and the angle is within the set range, it enters the next state
		if((calibratedYaw < 220 &&  calibratedYaw > 150) &&  LineCheck() == WHITE)
		{
			bee_time = 300;
			State_Machine.Q3_State = Q3_STATE_3;
		}			
	}
	else if(State_Machine.Q3_State == Q3_STATE_3)
	{
		static int Q3_state3_first_flag = 0;
		if(Q3_state3_first_flag == 0)
		{
			//设定斜切目标角度	Set the beveled target angle
			object_angle =navigetion_0_360_limit(first_angle + 180) - 11;
			Q3_state3_first_flag = 1;
		}
		Motion_Car_Control(300,0,0);
        //开始计算里程数	Start calculating mileage
		encoder_odometry_flag = 1;
		Angle_error = get_minor_arc(object_angle,calibratedYaw);
        PID_Value = Dir_PID(Angle_error);
        Motion_Yaw_Calc(PID_Value);
        
		//检测到黑线，并且里程计数值大于等于1200时进入下一个状态	When the black line is detected and the mileage count value is greater than or equal to 1200, enters the next state
		if(odometry_sum >= 1200 && LineCheck() == BLACK)
		{
            //停止计算里程数并清零	Stop calculating mileage and clear
            encoder_odometry_flag = 0;
            odometry_sum = 0;
			bee_time = 300;
			Q3_state3_first_flag = 0;
			State_Machine.Q3_State = Q3_STATE_4;
		}
	}
	else if(State_Machine.Q3_State == Q3_STATE_4)
	{
		LineWalking();
		//检测到白线，同时角度在设定的范围内时，进入下一个状态	When the white line is detected and the angle is within the set range, it enters the next state
		if((calibratedYaw > 325 ||  calibratedYaw < 90) &&  LineCheck() == WHITE)
		{
			bee_time = 300;
			State_Machine.Q3_State = Q3_STATE_5;
		}
	}
	else
	{
		//退出当前题目	Exit the current question
		State_Machine.Q3_State = STOP_STATE;
		State_Machine.Main_State = STOP_STATE;
		//问题3的运行标志位清零	The running flag bit of question 3 is cleared
		q3_first_flag = 0;
	}
}


void Question_Task_4(void)
{
	if(q4_first_flag == 0)
	{		
		//记录第一次发车的角度	Record the angle of the first departure
		first_angle = calibratedYaw;
        //开始计算里程数	Start calculating mileage
		encoder_odometry_flag = 1;
		//声光提示	Sound and light prompts
		bee_time = 300;
		//开始问题3的小状态机	Start the small state machine of problem 3
		State_Machine.Q3_State = Q3_STATE_1;
		
		q4_first_flag = 1;
	}
	if(State_Machine.Q3_State == Q3_STATE_1)
	{
		//使用MPU6050跑圈时，每运行一圈，Yaw值可能都会有变化，可以自己实测自己的Yaw角的变化程序，来修改每圈所需要矫正的Yaw变化数值
		//When running laps with MPU6050, the Yaw value may change every time you run the lap. You can measure your own Yaw angle change program to modify the Yaw change value that needs to be corrected for each lap.
        if(q4_circle_num == 0)
		{
			object_angle = navigetion_0_360_limit(first_angle + 32);
		}
		else if(q4_circle_num == 1)
		{
			object_angle = navigetion_0_360_limit(first_angle + 24);
		}
        else if(q4_circle_num == 2)
		{
			object_angle = navigetion_0_360_limit(first_angle + 23);
		}
        else if(q4_circle_num == 3)
		{
			object_angle = navigetion_0_360_limit(first_angle + 20);
		}

        Motion_Car_Control(300,0,0);
		Angle_error = get_minor_arc(object_angle,calibratedYaw);
        PID_Value = Dir_PID(Angle_error);
        Motion_Yaw_Calc(PID_Value);
        //检测到黑线，并且里程计数值大于等于1200时进入下一个状态	When the black line is detected and the mileage count value is greater than or equal to 1200, enters the next state
		if(LineCheck() == BLACK && odometry_sum >= 1200)   
		{
			bee_time = 300;
            //停止计算里程数并清零
            encoder_odometry_flag = 0;
            odometry_sum = 0;
			//进入问题3的下一个状态
			State_Machine.Q3_State = Q3_STATE_2;
		 }
	}
	else if(State_Machine.Q3_State == Q3_STATE_2)   //问题3的状态2(左转灰度循迹)	State 2 of question 3 (left turn grayscale tracking)
	{
		LineWalking();

		//检测到白线，同时角度在设定的范围内时，进入下一个状态	When the white line is detected and the angle is within the set range, it enters the next state
		if((calibratedYaw < 220 &&  calibratedYaw > 150) &&  LineCheck() == WHITE)
		{
			bee_time = 300;
			State_Machine.Q3_State = Q3_STATE_3;
		}			
	}
	else if(State_Machine.Q3_State == Q3_STATE_3)
	{
		//使用MPU6050跑圈时，每运行一圈，Yaw值可能都会有变化，可以自己实测自己的Yaw角的变化程序，来修改每圈所需要矫正的Yaw变化数值
		//When running laps with MPU6050, the Yaw value may change every time you run the lap. You can measure your own Yaw angle change program to modify the Yaw change value that needs to be corrected for each lap.
        if(q4_circle_num == 0)
		{
			object_angle = navigetion_0_360_limit(first_angle + 180) - 9;
		}
		else if(q4_circle_num == 1)
		{
			object_angle = navigetion_0_360_limit(first_angle + 180) - 11;
		}
        else if(q4_circle_num == 2)
		{
			object_angle = navigetion_0_360_limit(first_angle + 180) - 14;
		}
        else if(q4_circle_num == 3)
		{
			object_angle = navigetion_0_360_limit(first_angle + 180) - 13;
		}
		Motion_Car_Control(300,0,0);
        //开始计算里程数	Start calculating mileage
		encoder_odometry_flag = 1;
		Angle_error = get_minor_arc(object_angle,calibratedYaw);
        PID_Value = Dir_PID(Angle_error);
        Motion_Yaw_Calc(PID_Value);
        
		//检测到黑线，并且里程计数值大于等于1200时进入下一个状态	When the black line is detected and the mileage count value is greater than or equal to 1200, enters the next state
		if(odometry_sum >= 1200 && LineCheck() == BLACK)
		{
            //停止计算里程数并清零	Stop calculating mileage and clear
            encoder_odometry_flag = 0;
            odometry_sum = 0;
			bee_time = 300;
			State_Machine.Q3_State = Q3_STATE_4;
		}
	}
	else if(State_Machine.Q3_State == Q3_STATE_4)
	{
		LineWalking();
		//检测到白线，同时角度在设定的范围内时，进入下一个状态	When the white line is detected and the angle is within the set range, it enters the next state
		if((calibratedYaw > 325 ||  calibratedYaw < 90) &&  LineCheck() == WHITE)
		{
			bee_time = 300;
			State_Machine.Q3_State = Q3_STATE_5;
		}
	}

	else
	{
		//增加问题4已运行的圈数	Increase the number of runs in question 4
		q4_circle_num++;
		if(q4_circle_num >= 4)
		{
			State_Machine.Main_State = STOP_STATE;
			//问题4的运行标志位清零	The running flag bit of question 4 is cleared
		    q4_first_flag = 0;
		}
		State_Machine.Q3_State = Q3_STATE_1;
        //开始计算里程数	Start calculating mileage
		encoder_odometry_flag = 1;
	}
}
