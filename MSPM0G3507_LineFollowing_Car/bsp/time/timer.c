#include "timer.h"
#include "ti_msp_dl_config.h"

volatile uint32_t systick_counter = 0;
static BSP_Time_Tick1msCallback tick_callback = 0;

void BSP_Time_RegisterTick1ms(BSP_Time_Tick1msCallback callback)
{
    tick_callback = callback;
}

void Timer_Init(void)
{
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);
}


void TIMER_0_INST_IRQHandler(void)
{
    switch( DL_TimerG_getPendingInterrupt(TIMER_0_INST) )
    {
        case DL_TIMER_IIDX_ZERO://如果是0溢出中断  If it is a 0 overflow interrupt
            systick_counter++; // 每1ms自动+1      +1 per second
            if (tick_callback != 0) {
                tick_callback();
            }
            break;

        default:
            break;
    }

}

uint32_t Get_Time(void)
{
    return systick_counter;
}
