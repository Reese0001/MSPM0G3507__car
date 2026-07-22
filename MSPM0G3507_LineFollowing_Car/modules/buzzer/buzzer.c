#include "buzzer.h"

#define BUZZER_BEEP_ON_MS  (100U)
#define BUZZER_BEEP_OFF_MS (100U)

volatile uint32_t bee_time = 0U;
static volatile uint8_t requested_beeps = 0U;

void PWM_Buzzer_Init(void)
{
    DL_Timer_startCounter(BUZZER_INST);
}

void Buzzer_Toggle(void)
{
    static int i = 0;

    if (i == 0) {
        Buzzer_ON();
        i = 1;
    } else {
        Buzzer_OFF();
        i = 0;
    }
}

void Buzzer_ON(void)
{
    DL_TimerA_setCaptureCompareValue(BUZZER_INST, 100, GPIO_BUZZER_C3_IDX);
}

void Buzzer_OFF(void)
{
    DL_TimerA_setCaptureCompareValue(BUZZER_INST, 0, GPIO_BUZZER_C3_IDX);
}

void Buzzer_RequestBeeps(uint8_t times)
{
    if (times > 4U) {
        times = 4U;
    }
    requested_beeps = times;
}

void Buzzer_Handle(void)
{
    static bool buzzer_state = false;
    static bool sequence_on = false;
    static uint8_t sequence_remaining = 0U;
    static uint16_t sequence_phase_ms = 0U;

    if (requested_beeps > 0U) {
        sequence_remaining = requested_beeps;
        requested_beeps = 0U;
        sequence_on = true;
        sequence_phase_ms = BUZZER_BEEP_ON_MS;
        LED_ON();
        Buzzer_ON();
        buzzer_state = true;
    }

    if (sequence_remaining > 0U) {
        if (sequence_phase_ms > 0U) {
            sequence_phase_ms--;
            return;
        }

        if (sequence_on) {
            sequence_on = false;
            sequence_phase_ms = BUZZER_BEEP_OFF_MS;
            LED_OFF();
            Buzzer_OFF();
            buzzer_state = false;
            return;
        }

        sequence_remaining--;
        if (sequence_remaining > 0U) {
            sequence_on = true;
            sequence_phase_ms = BUZZER_BEEP_ON_MS;
            LED_ON();
            Buzzer_ON();
            buzzer_state = true;
        }
        return;
    }

    if (bee_time > 0U) {
        if (!buzzer_state) {
            LED_ON();
            Buzzer_ON();
            buzzer_state = true;
        }
        bee_time--;
    } else if (buzzer_state) {
        LED_OFF();
        Buzzer_OFF();
        buzzer_state = false;
    }
}

void Beep_Times(int times)
{
    if (times > 0) {
        Buzzer_RequestBeeps((uint8_t)times);
    }
}
