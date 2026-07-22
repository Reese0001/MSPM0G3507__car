#ifndef BSP_TIME_TIMER_H
#define BSP_TIME_TIMER_H

#include <stdint.h>

typedef void (*BSP_Time_Tick1msCallback)(void);

uint32_t Get_Time(void);
void Timer_Init(void);
void BSP_Time_RegisterTick1ms(BSP_Time_Tick1msCallback callback);

#endif
