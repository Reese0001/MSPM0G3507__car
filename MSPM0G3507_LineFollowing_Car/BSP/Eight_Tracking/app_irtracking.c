#include "app_irtracking.h"

#define IRTrack_Trun_KP (40.0f)
#define IRTrack_Trun_KD (10.0f)
#define IRR_SPEED (120)
#define TRACKING_RECOVERY_SPEED (36)
#define TRACKING_LOST_STOP_CYCLES (3U)

static const int8_t TRACKING_WEIGHTS[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

int pid_output_IRR = 0;
u8 trun_flag = 0;

float APP_ELE_PID_Calc(float error)
{
	static float error_last = 0.0f;
	float turn = error * IRTrack_Trun_KP
	           + (error - error_last) * IRTrack_Trun_KD;

	error_last = error;
	return turn;
}

bool Tracking_ComputeWeightedError(uint8_t sensor_bits, float *error)
{
    int16_t weighted_sum = 0;
    uint8_t active_count = 0;

    if (error == NULL) {
        return false;
    }

    for (uint8_t channel = 0; channel < 8U; channel++) {
        if ((sensor_bits & (uint8_t)(1U << channel)) != 0U) {
            weighted_sum += TRACKING_WEIGHTS[channel];
            active_count++;
        }
    }

    if (active_count == 0) {
        *error = 0.0f;
        return false;
    }

    *error = weighted_sum / (float) active_count;
    return true;
}


// 选择通道 (0-7)
void Gray_SelectChannel(uint8_t channel)
{
    // AD2 AD1 AD0
    if(channel & 0x04) {
        DL_GPIO_setPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    } else {
        DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    }

    if(channel & 0x02) {
        DL_GPIO_setPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    } else {
        DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    }

    if(channel & 0x01) {
        DL_GPIO_setPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
    } else {
        DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
    }

    // 等待信号稳定
    delay_us(10);
}

// 读取指定通道的数据（多次读取取平均，提高稳定性）
uint8_t Gray_ReadChannel(uint8_t channel)
{
    Gray_SelectChannel(channel);
    delay_ms(1);  // 参考代码使用1ms延迟，等待信号稳定
    // 多次读取取平均，提高稳定性
    uint8_t count = 0;
    for(int i = 0; i < 5; i++)
    {
        if(DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN))
        {
            count++;
        }
        delay_us(2);
    }
    // 如果3次以上为高电平，则认为是白线（1），否则是黑线（0）
    return (count >= 3) ? 1 : 0;
}

// 读取所有8路数据
void Gray_ReadAll(uint8_t *x1, uint8_t *x2, uint8_t *x3, uint8_t *x4,
                  uint8_t *x5, uint8_t *x6, uint8_t *x7, uint8_t *x8)
{
    *x1 = Gray_ReadChannel(0);  // 通道1
    *x2 = Gray_ReadChannel(1);  // 通道2
    *x3 = Gray_ReadChannel(2);  // 通道3
    *x4 = Gray_ReadChannel(3);  // 通道4
    *x5 = Gray_ReadChannel(4);  // 通道5
    *x6 = Gray_ReadChannel(5);  // 通道6
    *x7 = Gray_ReadChannel(6);  // 通道7
    *x8 = Gray_ReadChannel(7);  // 通道8
}


void printf_gray_data(void)
{
    static uint8_t ir_x1,ir_x2,ir_x3,ir_x4,ir_x5,ir_x6,ir_x7,ir_x8;
    Gray_ReadAll(&ir_x1,&ir_x2,&ir_x3,&ir_x4,&ir_x5,&ir_x6,&ir_x7,&ir_x8);
    printf("x1:%d,x2:%d,x3:%d,x4:%d,x5:%d,x6:%d,x7:%d,x8:%d\r\n",ir_x1,ir_x2,ir_x3,ir_x4,ir_x5,ir_x6,ir_x7,ir_x8);
}

static int16_t Tracking_LimitTurn(float turn, int16_t base_speed)
{
    const int16_t limit = (int16_t)(((int32_t)base_speed * 800) / (int32_t)Car_APB);

    if (turn > limit) {
        return limit;
    }
    if (turn < -limit) {
        return -limit;
    }
    return (int16_t)turn;
}

void LineWalking(void)
{
    static uint8_t lost_count = 0U;
    static float last_valid_error = 0.0f;
    uint8_t x1, x2, x3, x4, x5, x6, x7, x8;
    uint8_t sensor_bits = 0U;
    float error = 0.0f;
    int16_t base_speed;
    int16_t turn;

    Gray_ReadAll(&x1, &x2, &x3, &x4, &x5, &x6, &x7, &x8);

    /* Gray_ReadChannel 返回 0 表示黑线；转换后 bit0..bit7 对应 X1..X8。 */
    sensor_bits |= (x1 == 0U) ? (1U << 0) : 0U;
    sensor_bits |= (x2 == 0U) ? (1U << 1) : 0U;
    sensor_bits |= (x3 == 0U) ? (1U << 2) : 0U;
    sensor_bits |= (x4 == 0U) ? (1U << 3) : 0U;
    sensor_bits |= (x5 == 0U) ? (1U << 4) : 0U;
    sensor_bits |= (x6 == 0U) ? (1U << 5) : 0U;
    sensor_bits |= (x7 == 0U) ? (1U << 6) : 0U;
    sensor_bits |= (x8 == 0U) ? (1U << 7) : 0U;

    if (!Tracking_ComputeWeightedError(sensor_bits, &error)) {
        if (lost_count < UINT8_MAX) {
            lost_count++;
        }

        if (lost_count >= TRACKING_LOST_STOP_CYCLES) {
            Motion_Car_Control(0, 0, 0);
            return;
        }

        turn = Tracking_LimitTurn(last_valid_error * IRTrack_Trun_KP,
                                  TRACKING_RECOVERY_SPEED);
        Motion_Car_Control(TRACKING_RECOVERY_SPEED, 0, turn);
        return;
    }

    lost_count = 0U;
    last_valid_error = error;

    if (error > 3.0f || error < -3.0f) {
        base_speed = IRR_SPEED / 2;
    } else if (error > 1.0f || error < -1.0f) {
        base_speed = (IRR_SPEED * 3) / 4;
    } else {
        base_speed = IRR_SPEED;
    }

    turn = Tracking_LimitTurn(APP_ELE_PID_Calc(error), base_speed);
    pid_output_IRR = turn;
    Motion_Car_Control(base_speed, 0, turn);
}

/* 旧位型决策仅保留在源码历史中，不参与编译。 */
#if 0
void LineWalking_Legacy(void)
{
	static int8_t err = 0;
	static u8 x1,x2,x3,x4,x5,x6,x7,x8;

	Gray_ReadAll(&x1,&x2,&x3,&x4,&x5,&x6,&x7,&x8);

	//debug
//	static char bufbuf[30]={'\0'};
//	sprintf(bufbuf,"%d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t \r\n",x1,x2,x3,x4,x5,x6,x7,x8);
//	uart0_send_string((char*)bufbuf);

    //优先判断	Priority judgment
	if(x1 == 1 && x2 == 1 &&x3 == 0 &&  x4 == 0  && x5 == 0 && x6  == 0 && x7 == 1 && x8 == 1 ) //过锐角	Over sharp angle
	{
		err = 0;  // 中间检测到黑线，应该直行
	}
	else if(x1 == 1 && x2 == 1 &&x3 == 1 &&  x4 == 1  && x5 == 1 && x6  == 1 && x7 == 1 && x8 == 1 ) //过锐角	Over sharp angle
	{
		if(trun_flag == 0) //合格	Qualified
		{
			err = 0;
			trun_flag = 1;
		}
		//其他情况保持上一个状态	Stay in the previous state in other situations
	}

  else if(x1 == 0 &&  x2 == 0  && x7 == 0 && x8 == 0 ) //两边都亮走直道	Both sides are bright, run straight
	{
		err = 0;
		if(trun_flag == 1)
		{
			trun_flag = 0;//走的圈数	Got into the circle
		}
	}

	//优先判断	Priority judgment
 else if(x1 == 0 &&  x3 == 0 && x4 == 0 && x5 == 0 && x8 == 0 )
	{
		err = 0;
	}
	//加直角	Add a right angle - 急转弯处理
	else if((x1 == 0 || x2 == 0 ) && x8 == 1)
	{
		err = 20;  // 降低err值，减少转弯幅度
		pid_output_IRR = (int)(APP_ELE_PID_Calc(err));
		Motion_Car_Control(IRR_SPEED-50, 0, pid_output_IRR);  // 降低速度转弯
		delay_ms(100);  // 减少延迟
		return;
	}
	//加直角	Add a right angle - 急转弯处理
	else if((x7 == 0 ||  x8 == 0) && x1 == 1)
	{
		err = -20;  // 降低err值，减少转弯幅度
		pid_output_IRR = (int)(APP_ELE_PID_Calc(err));
		Motion_Car_Control(IRR_SPEED-50, 0, pid_output_IRR);  // 降低速度转弯
		delay_ms(100);  // 减少延迟
		return;
	}



	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 0 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1110 1111
	{
		err = 4;  // 降低err值
	}
	else if(x1 == 1 && x2 == 1  && x3 == 0&& x4 == 0 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1100 1111
	{
		err = 8;  // 降低err值
	}
	else if(x1 == 1 && x2 == 1  && x3 == 0&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1101 1111
	{
		err = 8;  // 降低err值
	}
	else if(x1 == 1 && x2 == 0  && x3 == 0&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1001 1111
	{
		err = 12;  // 降低err值
	}
	else if(x1 == 1 && x2 == 0  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1011 1111
	{
		err = 15;  // 降低err值
	}
	else if(x1 == 0 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 0111 1111
	{
		err = 18;  // 降低err值
	}



	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 0 && x6 == 1  && x7 == 1 && x8 == 1) // 1111 0111
	{
		err = -4;  // 降低err值
	}
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 0 && x6 == 0  && x7 == 1 && x8 == 1) // 1111 0011
	{
		err = -8;  // 降低err值
	}
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 0  && x7 == 1 && x8 == 1) // 1111 1011
	{
		err = -8;  // 降低err值
	}
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 0  && x7 == 0 && x8 == 1) // 1111 1001
	{
		err = -12;  // 降低err值
	}
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 0 && x8 == 1) // 1111 1101
	{
		err = -15;  // 降低err值
	}
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 0) // 1111 1110
	{
		err = -18;  // 降低err值
	}


	else if(x1 == 1 &&x2 == 1 &&x3 == 1 && x4 == 0 && x5 == 0 && x6 == 1 && x7 == 1&& x8 == 1) //直走	Go straight
	{
		err = 0;
	}
	// 中间偏差处理
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) // 微偏左
	{
		err = 2;  // 微调
	}
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1) // 微偏右
	{
		err = -2;  // 微调
	}



	//剩下的就保持上一个状态	The rest will remain in the previous state
	pid_output_IRR = (int)(APP_ELE_PID_Calc(err));

	Motion_Car_Control(IRR_SPEED, 0, pid_output_IRR);

}
#endif

//检测当前在黑线上还是白线上	Detection is now on the black line or on the white line
int LineCheck(void)
{
	int if_have = 0;
    static u8 x1,x2,x3,x4,x5,x6,x7,x8;
	Gray_ReadAll(&x1,&x2,&x3,&x4,&x5,&x6,&x7,&x8);
	//Traversing the grayscale sensor
	if(!x1)
	{
		if_have = 1;
	}
    if(!x2)
	{
		if_have = 1;
	}
    if(!x3)
	{
		if_have = 1;
	}
	if(!x4)
	{
		if_have = 1;
	}
	if(!x5)
	{
		if_have = 1;
	}
	if(!x6)
	{
		if_have = 1;
	}
    if(!x7)
	{
		if_have = 1;
	}
    if(!x8)
	{
		if_have = 1;
	}

	//If a black line is detected
	if(if_have >= 1)
	{
		  return BLACK;
	}
	else
	{
		  return WHITE;
	}
}
