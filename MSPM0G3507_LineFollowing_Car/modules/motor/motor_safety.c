#include "motor_safety.h"
#include "app_motor_usart.h"
#include "bsp_motor_usart.h"

#define MOTOR_COMMAND_MAX (1000)

typedef enum {
    MOTOR_SAFETY_DISARMED = 0,
    MOTOR_SAFETY_ARMED,
    MOTOR_SAFETY_FAULT_LATCHED
} Motor_Safety_State;

static volatile Motor_Safety_State safety_state = MOTOR_SAFETY_DISARMED;
static volatile uint32_t armed_elapsed_ms = 0U;
static volatile uint32_t watchdog_elapsed_ms = 0U;
static volatile uint8_t stop_pending = 0U;
static int requested_speed[4] = {0, 0, 0, 0};

static void clear_requested_speed(void)
{
    requested_speed[0] = 0;
    requested_speed[1] = 0;
    requested_speed[2] = 0;
    requested_speed[3] = 0;
}

static int clamp_speed(int value, int limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static int current_output_limit(void)
{
    uint32_t elapsed = armed_elapsed_ms;
    uint32_t percent;

    if (elapsed < MOTOR_SAFETY_RAMP_MS) {
        uint32_t step_ms = MOTOR_SAFETY_RAMP_MS / MOTOR_SAFETY_RAMP_STEPS;
        percent = (elapsed / step_ms) *
                  MOTOR_SAFETY_INITIAL_LIMIT_PERCENT / MOTOR_SAFETY_RAMP_STEPS;
    } else {
        percent = MOTOR_SAFETY_INITIAL_LIMIT_PERCENT +
                  ((elapsed - MOTOR_SAFETY_RAMP_MS) / MOTOR_SAFETY_POST_STEP_MS) *
                  MOTOR_SAFETY_POST_STEP_PERCENT;
    }

    if (percent > 100U) percent = 100U;
    return (int)((MOTOR_COMMAND_MAX * percent) / 100U);
}

void Motor_Safety_Init(void)
{
    safety_state = MOTOR_SAFETY_DISARMED;
    armed_elapsed_ms = 0U;
    watchdog_elapsed_ms = 0U;
    stop_pending = 0U;
    clear_requested_speed();
    Contrl_Speed(0, 0, 0, 0);
}

void Motor_Safety_Arm(void)
{
    if (safety_state == MOTOR_SAFETY_FAULT_LATCHED) return;
    armed_elapsed_ms = 0U;
    watchdog_elapsed_ms = 0U;
    safety_state = MOTOR_SAFETY_ARMED;
}

void Motor_Safety_Disarm(void)
{
    safety_state = MOTOR_SAFETY_DISARMED;
    armed_elapsed_ms = 0U;
    watchdog_elapsed_ms = 0U;
    clear_requested_speed();
    stop_pending = 1U;
}

void Motor_Safety_RequestSpeed(int motor1, int motor2, int motor3, int motor4)
{
    if (safety_state != MOTOR_SAFETY_ARMED) return;
    requested_speed[0] = clamp_speed(motor1, MOTOR_COMMAND_MAX);
    requested_speed[1] = clamp_speed(motor2, MOTOR_COMMAND_MAX);
    requested_speed[2] = clamp_speed(motor3, MOTOR_COMMAND_MAX);
    requested_speed[3] = clamp_speed(motor4, MOTOR_COMMAND_MAX);
    watchdog_elapsed_ms = 0U;
}

void Motor_Safety_Service(void)
{
    int limit;

    if (stop_pending != 0U) {
        Contrl_Speed(0, 0, 0, 0);
        stop_pending = 0U;
        return;
    }
    if (safety_state != MOTOR_SAFETY_ARMED) return;
    limit = current_output_limit();
    Contrl_Speed(clamp_speed(requested_speed[0], limit),
                 clamp_speed(requested_speed[1], limit),
                 clamp_speed(requested_speed[2], limit),
                 clamp_speed(requested_speed[3], limit));
}

void Motor_Safety_Tick1ms(void)
{
    if (safety_state != MOTOR_SAFETY_ARMED) return;
    if (armed_elapsed_ms < 0xFFFFFFFFU) armed_elapsed_ms++;
    if (watchdog_elapsed_ms < MOTOR_SAFETY_WATCHDOG_MS) watchdog_elapsed_ms++;
    if (watchdog_elapsed_ms >= MOTOR_SAFETY_WATCHDOG_MS) {
        safety_state = MOTOR_SAFETY_FAULT_LATCHED;
        Motor_EmergencyStop_FromISR();
    }
}

uint8_t Motor_Safety_IsFaultLatched(void)
{
    return (safety_state == MOTOR_SAFETY_FAULT_LATCHED) ? 1U : 0U;
}
