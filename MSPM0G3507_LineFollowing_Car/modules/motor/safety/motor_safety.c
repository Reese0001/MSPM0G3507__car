#include "motor_safety.h"
#include <stdbool.h>

#ifdef MOTOR_SAFETY_HOST_TEST
bool Motor_SendSpeedFrame(int16_t motor1, int16_t motor2,
                          int16_t motor3, int16_t motor4);
bool Motor_EmergencyStop_FromISR(void);
void LED_ON(void);
void LED_OFF(void);
uint32_t Motor_Safety_Host_EnterCritical(void);
void Motor_Safety_Host_ExitCritical(uint32_t previous_state);
#else
#include "../protocol/motor_protocol.h"
#include "../uart/motor_uart.h"
#include "../../led/led.h"
#endif

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
static volatile int applied_speed[4] = {0, 0, 0, 0};
static volatile int8_t last_applied_sign[4] = {0, 0, 0, 0};
static volatile uint32_t zero_elapsed_ms[4] = {0U, 0U, 0U, 0U};
static volatile uint8_t uart_frame_active = 0U;
static volatile uint8_t fault_stop_deferred = 0U;
static volatile uint8_t direction_wait = 0U;
static volatile MotorSafetyFaultReason fault_reason = MOTOR_SAFETY_FAULT_NONE;

/* Protect only shared state transitions; the 1 ms timer remains unmasked. */
static uint32_t motor_safety_enter_critical(void)
{
#ifdef MOTOR_SAFETY_HOST_TEST
    return Motor_Safety_Host_EnterCritical();
#else
    uint32_t previous_state = __get_PRIMASK();

    __disable_irq();
    return previous_state;
#endif
}

static void motor_safety_exit_critical(uint32_t previous_state)
{
#ifdef MOTOR_SAFETY_HOST_TEST
    Motor_Safety_Host_ExitCritical(previous_state);
#else
    __set_PRIMASK(previous_state);
#endif
}

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

static int8_t speed_sign(int value)
{
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}

/* Record the exact values submitted to the UART motor protocol. */
static void apply_speed(int motor1, int motor2, int motor3, int motor4)
{
    int output[4] = {motor1, motor2, motor3, motor4};
    uint8_t index;
    uint32_t previous_irq_state = motor_safety_enter_critical();
    uint8_t send_deferred_stop;

    if (safety_state == MOTOR_SAFETY_FAULT_LATCHED &&
        (output[0] != 0 || output[1] != 0 ||
         output[2] != 0 || output[3] != 0)) {
        motor_safety_exit_critical(previous_irq_state);
        return;
    }
    uart_frame_active = 1U;
    motor_safety_exit_critical(previous_irq_state);

    if (!Motor_SendSpeedFrame((int16_t)output[0], (int16_t)output[1],
                              (int16_t)output[2], (int16_t)output[3])) {
        previous_irq_state = motor_safety_enter_critical();
        uart_frame_active = 0U;
        safety_state = MOTOR_SAFETY_FAULT_LATCHED;
        fault_stop_deferred = 0U;
        direction_wait = 0U;
        fault_reason = MOTOR_SAFETY_FAULT_UART_TIMEOUT;
        motor_safety_exit_critical(previous_irq_state);
        LED_ON();
        if (Motor_SendSpeedFrame(0, 0, 0, 0)) {
            applied_speed[0] = 0;
            applied_speed[1] = 0;
            applied_speed[2] = 0;
            applied_speed[3] = 0;
        }
        return;
    }

    previous_irq_state = motor_safety_enter_critical();
    uart_frame_active = 0U;
    send_deferred_stop = fault_stop_deferred;
    fault_stop_deferred = 0U;
    for (index = 0U; index < 4U; index++) {
        int8_t sign = speed_sign(output[index]);

        applied_speed[index] = output[index];
        if (sign != 0) {
            last_applied_sign[index] = sign;
            zero_elapsed_ms[index] = 0U;
        }
    }
    motor_safety_exit_critical(previous_irq_state);

    if (send_deferred_stop != 0U) {
        if (Motor_SendSpeedFrame(0, 0, 0, 0)) {
            applied_speed[0] = 0;
            applied_speed[1] = 0;
            applied_speed[2] = 0;
            applied_speed[3] = 0;
        }
    }
}

static void reset_direction_interlock(void)
{
    uint8_t index;

    for (index = 0U; index < 4U; index++) {
        applied_speed[index] = 0;
        last_applied_sign[index] = 0;
        zero_elapsed_ms[index] = 0U;
    }
}

static uint8_t direction_change_is_blocked(uint8_t index, int command)
{
    int8_t requested_sign = speed_sign(command);

    if (requested_sign == 0 || last_applied_sign[index] == 0 ||
        requested_sign == last_applied_sign[index]) {
        return 0U;
    }
    return (zero_elapsed_ms[index] <= MOTOR_SAFETY_DIRECTION_CHANGE_PAUSE_MS) ?
           1U : 0U;
}

static uint8_t any_direction_change_is_blocked(void)
{
    uint8_t index;

    for (index = 0U; index < 4U; index++) {
        if (direction_change_is_blocked(index, requested_speed[index]) != 0U) {
            return 1U;
        }
    }
    return 0U;
}

/* A zero or rejected reversal stops the affected wheel without waiting. */
static void apply_immediate_stops(void)
{
    int output[4];
    uint8_t index;
    uint8_t changed = 0U;

    for (index = 0U; index < 4U; index++) {
        output[index] = applied_speed[index];
        if (requested_speed[index] == 0 ||
            direction_change_is_blocked(index, requested_speed[index])) {
            if (output[index] != 0) {
                output[index] = 0;
                changed = 1U;
            }
        }
    }
    if (changed != 0U) {
        apply_speed(output[0], output[1], output[2], output[3]);
    }
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
    uart_frame_active = 0U;
    fault_stop_deferred = 0U;
    direction_wait = 0U;
    fault_reason = MOTOR_SAFETY_FAULT_NONE;
    clear_requested_speed();
    reset_direction_interlock();
    LED_OFF();
    apply_speed(0, 0, 0, 0);
}

void Motor_Safety_Arm(void)
{
    uint32_t previous_irq_state = motor_safety_enter_critical();

    if (safety_state == MOTOR_SAFETY_FAULT_LATCHED) {
        motor_safety_exit_critical(previous_irq_state);
        return;
    }
    armed_elapsed_ms = 0U;
    watchdog_elapsed_ms = 0U;
    safety_state = MOTOR_SAFETY_ARMED;
    motor_safety_exit_critical(previous_irq_state);
}

void Motor_Safety_Disarm(void)
{
    uint32_t previous_irq_state = motor_safety_enter_critical();

    if (safety_state != MOTOR_SAFETY_FAULT_LATCHED) {
        safety_state = MOTOR_SAFETY_DISARMED;
    }
    armed_elapsed_ms = 0U;
    watchdog_elapsed_ms = 0U;
    clear_requested_speed();
    direction_wait = 0U;
    stop_pending = 1U;
    motor_safety_exit_critical(previous_irq_state);
    apply_speed(0, 0, 0, 0);
}

void Motor_Safety_RequestSpeed(int motor1, int motor2, int motor3, int motor4)
{
    uint32_t previous_irq_state;

    if (safety_state != MOTOR_SAFETY_ARMED) return;
    requested_speed[0] = clamp_speed(motor1, MOTOR_COMMAND_MAX);
    requested_speed[1] = clamp_speed(motor2, MOTOR_COMMAND_MAX);
    requested_speed[2] = clamp_speed(motor3, MOTOR_COMMAND_MAX);
    requested_speed[3] = clamp_speed(motor4, MOTOR_COMMAND_MAX);
    watchdog_elapsed_ms = 0U;
    apply_immediate_stops();
    previous_irq_state = motor_safety_enter_critical();
    direction_wait = any_direction_change_is_blocked();
    motor_safety_exit_critical(previous_irq_state);
}

void Motor_Safety_Service(void)
{
    int limit;
    int output[4];
    uint8_t index;

    if (safety_state == MOTOR_SAFETY_FAULT_LATCHED) {
        direction_wait = 0U;
        /* Retry bounded zero frames until one is confirmed complete. */
        apply_speed(0, 0, 0, 0);
        return;
    }
    if (stop_pending != 0U) {
        direction_wait = 0U;
        apply_speed(0, 0, 0, 0);
        stop_pending = 0U;
        return;
    }
    if (safety_state != MOTOR_SAFETY_ARMED) return;
    limit = current_output_limit();
    for (index = 0U; index < 4U; index++) {
        output[index] = clamp_speed(requested_speed[index], limit);
        if (direction_change_is_blocked(index, output[index])) {
            output[index] = 0;
        }
    }
    direction_wait = any_direction_change_is_blocked();
    apply_speed(output[0], output[1], output[2], output[3]);
}

void Motor_Safety_Tick1ms(void)
{
    uint8_t index;

    for (index = 0U; index < 4U; index++) {
        if (applied_speed[index] == 0 && last_applied_sign[index] != 0 &&
            zero_elapsed_ms[index] <= MOTOR_SAFETY_DIRECTION_CHANGE_PAUSE_MS) {
            zero_elapsed_ms[index]++;
        }
    }
    direction_wait = any_direction_change_is_blocked();
    if (safety_state != MOTOR_SAFETY_ARMED) return;
    if (armed_elapsed_ms < 0xFFFFFFFFU) armed_elapsed_ms++;
    if (watchdog_elapsed_ms < MOTOR_SAFETY_WATCHDOG_MS) watchdog_elapsed_ms++;
    if (watchdog_elapsed_ms >= MOTOR_SAFETY_WATCHDOG_MS) {
        safety_state = MOTOR_SAFETY_FAULT_LATCHED;
        direction_wait = 0U;
        fault_reason = MOTOR_SAFETY_FAULT_WATCHDOG;
        LED_ON();
        if (uart_frame_active != 0U) {
            fault_stop_deferred = 1U;
        } else {
            if (Motor_EmergencyStop_FromISR()) {
                applied_speed[0] = 0;
                applied_speed[1] = 0;
                applied_speed[2] = 0;
                applied_speed[3] = 0;
            }
        }
    }
}

uint8_t Motor_Safety_IsFaultLatched(void)
{
    return (safety_state == MOTOR_SAFETY_FAULT_LATCHED) ? 1U : 0U;
}

void Motor_Safety_GetDiagnostics(MotorSafetyDiagnostics *out)
{
    uint32_t previous_irq_state;

    if (out == 0) {
        return;
    }
    previous_irq_state = motor_safety_enter_critical();
    out->left_applied = (int16_t)applied_speed[1];
    out->right_applied = (int16_t)applied_speed[3];
    out->direction_wait = (direction_wait != 0U);
    out->fault_reason = fault_reason;
    motor_safety_exit_critical(previous_irq_state);
}
