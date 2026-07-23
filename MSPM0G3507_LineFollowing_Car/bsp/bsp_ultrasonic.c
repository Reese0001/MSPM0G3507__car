#include "bsp_ultrasonic.h"

#include "ti_msp_dl_config.h"

#include "../modules/ultrasonic/ultrasonic.h"
#include "time/timer.h"

void BSP_Ultrasonic_Init(void)
{
    DL_GPIO_clearPins(ULTRASONIC_TRIG_PORT,
                      ULTRASONIC_TRIG_TRIG_PIN);
    NVIC_ClearPendingIRQ(ULTRASONIC_ECHO_INST_INT_IRQN);
    NVIC_EnableIRQ(ULTRASONIC_ECHO_INST_INT_IRQN);
    DL_TimerG_startCounter(ULTRASONIC_ECHO_INST);
}

uint32_t BSP_Ultrasonic_NowUs(void)
{
    return BSP_Time_GetUs();
}

void BSP_Ultrasonic_SetTrig(bool high)
{
    if (high) {
        DL_GPIO_setPins(ULTRASONIC_TRIG_PORT,
                        ULTRASONIC_TRIG_TRIG_PIN);
    } else {
        DL_GPIO_clearPins(ULTRASONIC_TRIG_PORT,
                          ULTRASONIC_TRIG_TRIG_PIN);
    }
}

void ULTRASONIC_ECHO_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(ULTRASONIC_ECHO_INST) ==
        DL_TIMERG_IIDX_CC1_DN) {
        bool high =
            (DL_GPIO_readPins(GPIO_ULTRASONIC_ECHO_C1_PORT,
                              GPIO_ULTRASONIC_ECHO_C1_PIN) != 0U);
        Ultrasonic_OnEchoEdge(high, BSP_Time_GetUs());
    }
}
