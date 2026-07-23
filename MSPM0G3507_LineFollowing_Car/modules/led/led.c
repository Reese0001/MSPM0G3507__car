#include "led.h"

#define LED_HEARTBEAT_PERIOD_MS (250U)

static uint32_t heartbeat_last_ms = 0U;

void LED_Toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_D1_PIN);
}

void LED_ON(void)
{
    DL_GPIO_setPins(LED_PORT,LED_D1_PIN);  //LED控制输出高电平  LED control output high level
}

void LED_OFF(void)
{
    DL_GPIO_clearPins(LED_PORT,LED_D1_PIN);//LED控制输出低电平  LED control output low level
}

void LED2_Toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_D2_PIN);
}

void LED_HeartbeatInit(void)
{
    heartbeat_last_ms = 0U;
    DL_GPIO_clearPins(LED_PORT, LED_D2_PIN);
}

void LED_HeartbeatService(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - heartbeat_last_ms) <
        LED_HEARTBEAT_PERIOD_MS) {
        return;
    }
    heartbeat_last_ms = now_ms;
    DL_GPIO_togglePins(LED_PORT, LED_D2_PIN);
}
