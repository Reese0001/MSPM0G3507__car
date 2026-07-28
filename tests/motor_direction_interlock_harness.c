#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "motor_safety.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        (void)fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static int output[4];
static uint32_t emergency_stop_count;
static uint32_t zero_frame_count;
static uint32_t critical_depth;
static uint8_t inject_watchdog_tick;
static uint8_t watchdog_tick_injected;
static uint8_t fail_next_frame;

uint32_t Motor_Safety_Host_EnterCritical(void)
{
    critical_depth++;
    return 0U;
}

void Motor_Safety_Host_ExitCritical(uint32_t previous_state)
{
    (void)previous_state;
    critical_depth--;
    if (inject_watchdog_tick != 0U && critical_depth == 0U &&
        watchdog_tick_injected == 0U) {
        watchdog_tick_injected = 1U;
        Motor_Safety_Tick1ms();
    }
}

bool Motor_SendSpeedFrame(int16_t motor1, int16_t motor2,
                          int16_t motor3, int16_t motor4)
{
    if (fail_next_frame != 0U) {
        fail_next_frame = 0U;
        return false;
    }
    if (inject_watchdog_tick != 0U && critical_depth == 0U &&
        watchdog_tick_injected == 0U) {
        watchdog_tick_injected = 1U;
        Motor_Safety_Tick1ms();
    }
    output[0] = motor1;
    output[1] = motor2;
    output[2] = motor3;
    output[3] = motor4;
    if (motor1 == 0 && motor2 == 0 && motor3 == 0 && motor4 == 0) {
        zero_frame_count++;
    }
    return true;
}

bool Motor_EmergencyStop_FromISR(void)
{
    emergency_stop_count++;
    return Motor_SendSpeedFrame(0, 0, 0, 0);
}

void LED_ON(void)
{
}

void LED_OFF(void)
{
}

static void service_ms(uint32_t count)
{
    uint32_t index;

    for (index = 0U; index < count; index++) {
        Motor_Safety_Tick1ms();
        Motor_Safety_Service();
    }
}

static void init_and_arm(void)
{
    emergency_stop_count = 0U;
    zero_frame_count = 0U;
    critical_depth = 0U;
    inject_watchdog_tick = 0U;
    watchdog_tick_injected = 0U;
    fail_next_frame = 0U;
    Motor_Safety_Init();
    Motor_Safety_Arm();
}

static int init_does_not_latch_uart_fault_before_driver_config(void)
{
    MotorSafetyDiagnostics diagnostics;

    emergency_stop_count = 0U;
    zero_frame_count = 0U;
    critical_depth = 0U;
    inject_watchdog_tick = 0U;
    watchdog_tick_injected = 0U;
    fail_next_frame = 1U;

    Motor_Safety_Init();
    Motor_Safety_GetDiagnostics(&diagnostics);
    CHECK(Motor_Safety_IsFaultLatched() == 0U);
    CHECK(diagnostics.fault_reason == MOTOR_SAFETY_FAULT_NONE);
    CHECK(zero_frame_count == 0U);
    return 0;
}

static int establish_sign(int command)
{
    Motor_Safety_RequestSpeed(0, command, 0, command);
    Motor_Safety_Service();
    CHECK(output[1] == 0 && output[3] == 0);
    service_ms(100U);
    CHECK((command > 0 && output[1] > 0 && output[3] > 0) ||
          (command < 0 && output[1] < 0 && output[3] < 0));
    return 0;
}

static int diagnostics_snapshot_reports_safety_state(void)
{
    MotorSafetyDiagnostics diagnostics;

    init_and_arm();
    Motor_Safety_Service();
    Motor_Safety_GetDiagnostics(&diagnostics);
    CHECK(diagnostics.left_applied == 0);
    CHECK(diagnostics.right_applied == 0);
    CHECK(diagnostics.direction_wait == false);
    CHECK(diagnostics.fault_reason == MOTOR_SAFETY_FAULT_NONE);

    if (establish_sign(100) != 0) {
        return 1;
    }
    Motor_Safety_GetDiagnostics(&diagnostics);
    CHECK(diagnostics.left_applied == output[1]);
    CHECK(diagnostics.right_applied == output[3]);
    Motor_Safety_RequestSpeed(0, -100, 0, -100);
    Motor_Safety_Service();
    Motor_Safety_GetDiagnostics(&diagnostics);
    CHECK(diagnostics.direction_wait == true);

    init_and_arm();
    fail_next_frame = 1U;
    Motor_Safety_Service();
    Motor_Safety_GetDiagnostics(&diagnostics);
    CHECK(diagnostics.fault_reason == MOTOR_SAFETY_FAULT_UART_TIMEOUT);

    init_and_arm();
    service_ms(MOTOR_SAFETY_WATCHDOG_MS);
    Motor_Safety_GetDiagnostics(&diagnostics);
    CHECK(diagnostics.fault_reason == MOTOR_SAFETY_FAULT_WATCHDOG);
    return 0;
}

static int positive_to_negative_is_per_wheel_and_exactly_120ms(void)
{
    init_and_arm();
    if (establish_sign(100) != 0) {
        return 1;
    }

    Motor_Safety_RequestSpeed(0, -100, 0, 100);
    Motor_Safety_Service();
    CHECK(output[1] == 0);
    CHECK(output[3] > 0);
    service_ms(119U);
    CHECK(output[1] == 0);
    CHECK(output[3] > 0);
    service_ms(1U);
    CHECK(output[1] == 0);
    service_ms(1U);
    CHECK(output[1] < 0);
    CHECK(output[3] > 0);
    return 0;
}

static int negative_to_positive_requires_the_same_pause(void)
{
    init_and_arm();
    if (establish_sign(-100) != 0) {
        return 1;
    }

    Motor_Safety_RequestSpeed(0, 100, 0, -100);
    Motor_Safety_Service();
    CHECK(output[1] == 0);
    CHECK(output[3] < 0);
    service_ms(119U);
    CHECK(output[1] == 0);
    service_ms(1U);
    CHECK(output[1] == 0);
    service_ms(1U);
    CHECK(output[1] > 0);
    CHECK(output[3] < 0);
    return 0;
}

static int right_wheel_is_an_independent_mirror(void)
{
    init_and_arm();
    if (establish_sign(100) != 0) {
        return 1;
    }

    Motor_Safety_RequestSpeed(0, 100, 0, -100);
    Motor_Safety_Service();
    CHECK(output[1] > 0);
    CHECK(output[3] == 0);
    service_ms(119U);
    CHECK(output[1] > 0);
    CHECK(output[3] == 0);
    service_ms(1U);
    CHECK(output[3] == 0);
    service_ms(1U);
    CHECK(output[1] > 0);
    CHECK(output[3] < 0);
    return 0;
}

static int returning_to_the_prior_sign_is_allowed_while_waiting(void)
{
    init_and_arm();
    if (establish_sign(100) != 0) {
        return 1;
    }

    Motor_Safety_RequestSpeed(0, -100, 0, 100);
    Motor_Safety_Service();
    CHECK(output[1] == 0);
    service_ms(60U);
    Motor_Safety_RequestSpeed(0, 100, 0, 100);
    Motor_Safety_Service();
    CHECK(output[1] > 0);
    return 0;
}

static int existing_zero_time_is_credited_and_startup_is_not_delayed(void)
{
    init_and_arm();
    Motor_Safety_RequestSpeed(0, -100, 0, 0);
    Motor_Safety_Service();
    service_ms(100U);
    CHECK(output[1] < 0);

    Motor_Safety_RequestSpeed(0, 0, 0, 0);
    Motor_Safety_Service();
    CHECK(output[1] == 0);
    service_ms(121U);
    Motor_Safety_RequestSpeed(0, 100, 0, 0);
    Motor_Safety_Service();
    CHECK(output[1] > 0);
    return 0;
}

static int zero_disarm_and_watchdog_stay_dominant(void)
{
    init_and_arm();
    if (establish_sign(100) != 0) {
        return 1;
    }
    Motor_Safety_RequestSpeed(0, 0, 0, 100);
    CHECK(output[1] == 0);
    CHECK(output[3] > 0);

    Motor_Safety_Disarm();
    CHECK(output[0] == 0 && output[1] == 0 &&
          output[2] == 0 && output[3] == 0);
    CHECK(output[0] == 0 && output[1] == 0 &&
          output[2] == 0 && output[3] == 0);

    init_and_arm();
    if (establish_sign(100) != 0) {
        return 1;
    }
    service_ms(100U);
    CHECK(Motor_Safety_IsFaultLatched() == 1U);
    CHECK(emergency_stop_count == 1U);
    CHECK(output[0] == 0 && output[1] == 0 &&
          output[2] == 0 && output[3] == 0);
    Motor_Safety_RequestSpeed(0, -100, 0, -100);
    Motor_Safety_Service();
    CHECK(output[0] == 0 && output[1] == 0 &&
          output[2] == 0 && output[3] == 0);
    Motor_Safety_Disarm();
    Motor_Safety_Arm();
    Motor_Safety_RequestSpeed(0, -100, 0, -100);
    Motor_Safety_Service();
    CHECK(Motor_Safety_IsFaultLatched() == 1U);
    CHECK(output[0] == 0 && output[1] == 0 &&
          output[2] == 0 && output[3] == 0);
    return 0;
}

static int clamp_and_ramp_remain_enforced(void)
{
    uint32_t index;

    init_and_arm();
    Motor_Safety_RequestSpeed(0, 2000, 0, -2000);
    Motor_Safety_Service();
    CHECK(output[1] == 0 && output[3] == 0);
    for (index = 0U; index < 100U; index++) {
        Motor_Safety_RequestSpeed(0, 2000, 0, -2000);
        Motor_Safety_Tick1ms();
        Motor_Safety_Service();
    }
    CHECK(output[1] == 30 && output[3] == -30);
    return 0;
}

static int watchdog_cannot_interleave_or_be_bypassed_by_a_uart_frame(void)
{
    uint32_t zero_frames_before;

    init_and_arm();
    if (establish_sign(100) != 0) {
        return 1;
    }
    service_ms(99U);
    CHECK(Motor_Safety_IsFaultLatched() == 0U);

    zero_frames_before = zero_frame_count;
    inject_watchdog_tick = 1U;
    Motor_Safety_Service();
    CHECK(watchdog_tick_injected == 1U);
    CHECK(Motor_Safety_IsFaultLatched() == 1U);
    CHECK(emergency_stop_count == 0U);
    CHECK(zero_frame_count == zero_frames_before + 1U);
    CHECK(output[0] == 0 && output[1] == 0 &&
          output[2] == 0 && output[3] == 0);
    return 0;
}

static int failed_uart_frame_latches_fault_and_keeps_zero_output(void)
{
    init_and_arm();
    if (establish_sign(100) != 0) {
        return 1;
    }
    fail_next_frame = 1U;
    Motor_Safety_Service();
    CHECK(Motor_Safety_IsFaultLatched() == 1U);
    CHECK(output[0] == 0 && output[1] == 0 &&
          output[2] == 0 && output[3] == 0);
    Motor_Safety_RequestSpeed(0, 100, 0, 100);
    Motor_Safety_Service();
    CHECK(output[0] == 0 && output[1] == 0 &&
          output[2] == 0 && output[3] == 0);
    return 0;
}

int main(void)
{
    if (init_does_not_latch_uart_fault_before_driver_config() != 0) {
        return 1;
    }
    if (diagnostics_snapshot_reports_safety_state() != 0) {
        return 1;
    }
    if (positive_to_negative_is_per_wheel_and_exactly_120ms() != 0) {
        return 1;
    }
    if (negative_to_positive_requires_the_same_pause() != 0) {
        return 1;
    }
    if (right_wheel_is_an_independent_mirror() != 0) {
        return 1;
    }
    if (returning_to_the_prior_sign_is_allowed_while_waiting() != 0) {
        return 1;
    }
    if (existing_zero_time_is_credited_and_startup_is_not_delayed() != 0) {
        return 1;
    }
    if (zero_disarm_and_watchdog_stay_dominant() != 0) {
        return 1;
    }
    if (watchdog_cannot_interleave_or_be_bypassed_by_a_uart_frame() != 0) {
        return 1;
    }
    if (failed_uart_frame_latches_fault_and_keeps_zero_output() != 0) {
        return 1;
    }
    return clamp_and_ramp_remain_enforced();
}
