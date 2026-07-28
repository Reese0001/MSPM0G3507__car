#include "delay.h"
#include "timer.h"

void delay_us(unsigned long __us)
{
    uint32_t started_us = BSP_Time_GetUs();

    while ((uint32_t)(BSP_Time_GetUs() - started_us) < (uint32_t)__us) {
    }
}

void delay_ms(unsigned long ms)
{
    delay_us(ms * 1000UL);
}
