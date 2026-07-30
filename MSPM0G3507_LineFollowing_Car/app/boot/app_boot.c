#include "app_boot.h"

#include "../../modules/buzzer/buzzer.h"
#include "../../modules/diagnostics/boot_trace.h"
#include "../../modules/display/ssd1306/ssd1306.h"
#include "../../modules/led/led.h"
#include "../../modules/motor/configuration/motor_configuration.h"
#include "../../modules/motor/drive.h"
#include "../../modules/motor/uart/motor_uart.h"
#include "../../modules/time/timer.h"

static bool motor_configured;
static bool display_ready;

static void tick_1ms(void)
{
    Buzzer_Handle();
    BootTrace_Tick1ms();
    Drive_Tick1ms();
}

static void show_boot_state(const char *state)
{
    if (!display_ready) {
        return;
    }
    Ssd1306_ClearBuffer();
    Ssd1306_DrawText(0U, 0U, "LINE CAR BOOT");
    Ssd1306_DrawText(1U, 0U, state);
    display_ready = Ssd1306_FlushDirty();
}

void AppBoot_Init(void)
{
    Motor_Usart_init();
    Timer_Init();
    Drive_Init();
    LED_HeartbeatInit();
    display_ready = Ssd1306_Init();
    show_boot_state("MOTOR CFG");
    motor_configured = Set_Motor(5);
    if (motor_configured) {
        /* 只打开速度反馈；速度控制帧格式保持不变。 */
        send_upload_data(false, false, true);
    }
    /* 配置过程可能持续约 500 ms；再次归零，确保 K1 前无遗留命令。 */
    Drive_Init();
    show_boot_state(motor_configured ? "CFG OK" : "CFG ERROR");
    BSP_Time_RegisterTick1ms(tick_1ms);
}

bool AppBoot_IsMotorConfigured(void)
{
    return motor_configured;
}

bool AppBoot_IsDisplayReady(void)
{
    return display_ready;
}
