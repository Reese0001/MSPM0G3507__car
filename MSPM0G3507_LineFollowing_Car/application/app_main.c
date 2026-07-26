#include "app_main.h"

#include "app_motor.h"
#include "bsp_motor_usart.h"
#include "buzzer.h"
#include "led.h"
#include "motor_safety.h"
#include "timer.h"

static void App_Main_Tick1ms(void)
{
    Buzzer_Handle();
    Motor_Safety_Tick1ms();
}

void App_Main_Init(void)
{
    Motor_Usart_init();
    Timer_Init();
    Motor_Safety_Init();
    LED_HeartbeatInit();

    /* Configure the confirmed L-type 520 motor while safety remains disarmed. */
    Set_Motor(5);
    BSP_Time_RegisterTick1ms(App_Main_Tick1ms);
}
