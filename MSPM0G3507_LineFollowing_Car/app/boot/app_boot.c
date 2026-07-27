#include "app_boot.h"

#include "../../modules/motor/configuration/motor_configuration.h"
#include "../../modules/motor/uart/motor_uart.h"
#include "../../modules/buzzer/buzzer.h"
#include "../../modules/led/led.h"
#include "../../modules/motor/safety/motor_safety.h"
#include "../../modules/time/timer.h"
#include "../../modules/display/runtime_log.h"
#include "../../modules/display/ssd1306/ssd1306.h"

static bool motor_configured = false;
static bool display_ready = false;

static void AppBoot_RenderRuntimeLog(void)
{
    if (display_ready) {
        RuntimeLog_Draw();
        display_ready = Ssd1306_FlushDirty();
    }
}

static void AppBoot_Tick1ms(void)
{
    Buzzer_Handle();
    Motor_Safety_Tick1ms();
}

void AppBoot_Init(void)
{
    motor_configured = false;
    display_ready = false;
    Motor_Usart_init();
    Timer_Init();
    Motor_Safety_Init();
    LED_HeartbeatInit();

    RuntimeLog_Init();
    (void)RuntimeLog_Push(Get_Time(), "BOOT");
    AppBoot_RenderRuntimeLog();

    display_ready = Ssd1306_Init();
    (void)RuntimeLog_Push(Get_Time(), display_ready ? "OLED OK" : "OLED FAIL");
    AppBoot_RenderRuntimeLog();
    (void)RuntimeLog_Push(Get_Time(), "AUTO START");
    AppBoot_RenderRuntimeLog();
    (void)RuntimeLog_Push(Get_Time(), "MOTOR CFG");
    AppBoot_RenderRuntimeLog();

    /* Configure the confirmed L-type 520 motor while safety remains disarmed. */
    motor_configured = Set_Motor(5);
    (void)RuntimeLog_Push(Get_Time(),
                          motor_configured ? "CFG OK" : "UART TIMEOUT");
    AppBoot_RenderRuntimeLog();
    BSP_Time_RegisterTick1ms(AppBoot_Tick1ms);
}

bool AppBoot_IsMotorConfigured(void)
{
    return motor_configured;
}

bool AppBoot_IsDisplayReady(void)
{
    return display_ready;
}
