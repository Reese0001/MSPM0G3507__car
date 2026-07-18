#include "ti_msp_dl_config.h"
#include "delay.h"
#include "app_irtracking.h"
#include "app_motor.h"
#include "usart.h"
#include "timer.h"
#include "motor_safety.h"
#include "bsp_motor_usart.h"

int main(void)
{
    SYSCFG_DL_init();
    Motor_Usart_init();
    Timer_Init();
    Motor_Safety_Init();

    /* L 型 520 电机参数；配置期间保持零速，随后才允许软启动。 */
    Set_Motor(5);
    Motor_Safety_Arm();

    while(1)
    {
        // 执行循迹
        LineWalking();

        Motor_Safety_Service();

        // 短暂延时
        delay_ms(10);
    }
}
