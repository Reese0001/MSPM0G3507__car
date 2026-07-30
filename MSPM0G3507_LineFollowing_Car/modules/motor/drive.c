#include "drive.h"

#include "../line_tracking/line_tracking_config.h"
#include "feedback/differential_controller.h"
#include "feedback/motor_feedback.h"

bool Motor_SendSpeedFrame(int16_t motor1, int16_t motor2,
                          int16_t motor3, int16_t motor4);
#ifdef DRIVE_HOST_TEST
bool Motor_EmergencyStop_FromISR(void);
#else
#include "uart/motor_uart.h"
#endif

#define DRIVE_COMMAND_LIMIT       (DRIVE_COMMAND_MAX)
#define DRIVE_RAMP_STAGE_MS       (1000U)
#define DRIVE_RAMP_TOTAL_MS       (2000U)
#define DRIVE_START_LIMIT         (135)
#define DRIVE_REQUEST_MAX_AGE_MS  (50U)
#define DRIVE_WATCHDOG_MS         (200U)
#define DRIVE_SEND_PERIOD_MS      (5U)
#define DRIVE_REVERSE_PAUSE_MS    (120U)
#define DRIVE_SLEW_STEP            (3)

static DriveStatus state;
static MotionRequest target;
static uint32_t ramp_ms;
static uint32_t watchdog_ms;
static uint32_t last_send_ms;
static int8_t last_sign[2];
static uint32_t zero_ms[2];
static DifferentialController differential_controller;
static uint32_t last_feedback_ms;

static int8_t sign_of(int16_t value)
{
    return value < 0 ? -1 : value > 0 ? 1 : 0;
}

static int16_t clamp_target(int16_t value)
{
    if (value > DRIVE_COMMAND_LIMIT) {
        return DRIVE_COMMAND_LIMIT;
    }
    if (value < -DRIVE_COMMAND_LIMIT) {
        return -DRIVE_COMMAND_LIMIT;
    }
    return value;
}

static int16_t ramped(int16_t value)
{
    int32_t limit;

    if (ramp_ms <= DRIVE_RAMP_STAGE_MS) {
        limit = (int32_t)(ramp_ms * DRIVE_START_LIMIT /
                          DRIVE_RAMP_STAGE_MS);
    } else if (ramp_ms < DRIVE_RAMP_TOTAL_MS) {
        limit = DRIVE_START_LIMIT +
                (int32_t)((ramp_ms - DRIVE_RAMP_STAGE_MS) *
                          (DRIVE_COMMAND_LIMIT - DRIVE_START_LIMIT) /
                          DRIVE_RAMP_STAGE_MS);
    } else {
        limit = DRIVE_COMMAND_LIMIT;
    }

    if (value > limit) {
        return (int16_t)limit;
    }
    if (value < -limit) {
        return (int16_t)-limit;
    }
    return value;
}

static int16_t interlocked(uint8_t wheel, int16_t desired)
{
    int8_t sign = sign_of(desired);

    if (sign == 0 || last_sign[wheel] == 0 || sign == last_sign[wheel]) {
        return desired;
    }
    return zero_ms[wheel] > DRIVE_REVERSE_PAUSE_MS ? desired : 0;
}

static void record_applied(uint8_t wheel, int16_t value)
{
    int8_t sign = sign_of(value);

    if (sign != 0) {
        last_sign[wheel] = sign;
        zero_ms[wheel] = 0U;
    }
}

static int16_t slew_toward(int16_t current, int16_t desired)
{
    if (desired > current + DRIVE_SLEW_STEP) {
        return (int16_t)(current + DRIVE_SLEW_STEP);
    }
    if (desired < current - DRIVE_SLEW_STEP) {
        return (int16_t)(current - DRIVE_SLEW_STEP);
    }
    return desired;
}

static void send_output(int16_t left, int16_t right, uint32_t now_ms)
{
    bool immediate_stop =
        (left == 0 && state.left_applied != 0) ||
        (right == 0 && state.right_applied != 0);

    if (!immediate_stop &&
        (uint32_t)(now_ms - last_send_ms) < DRIVE_SEND_PERIOD_MS) {
        return;
    }
    if (!immediate_stop) {
        left = slew_toward(state.left_applied, left);
        right = slew_toward(state.right_applied, right);
    }
    if (!Motor_SendSpeedFrame(0, right, 0, left)) {
        state.fault = true;
        state.error = DRIVE_ERROR_UART;
        state.left_applied = 0;
        state.right_applied = 0;
        (void)Motor_EmergencyStop_FromISR();
        return;
    }
    state.left_applied = left;
    state.right_applied = right;
    record_applied(0U, left);
    record_applied(1U, right);
    last_send_ms = now_ms;
}

void Drive_Init(void)
{
    state = (DriveStatus){0};
    target = (MotionRequest){0};
    ramp_ms = 0U;
    watchdog_ms = 0U;
    last_send_ms = 0U;
    last_sign[0] = last_sign[1] = 0;
    zero_ms[0] = zero_ms[1] = 0U;
    DifferentialController_Init(&differential_controller);
    last_feedback_ms = 0U;
    if (!Motor_EmergencyStop_FromISR()) {
        state.fault = true;
        state.error = DRIVE_ERROR_UART;
    }
}

void Drive_Start(void)
{
    if (!state.fault) {
        state.started = true;
        ramp_ms = 0U;
        watchdog_ms = 0U;
        DifferentialController_Init(&differential_controller);
        last_feedback_ms = 0U;
    }
}

void Drive_SetTarget(const MotionRequest *request)
{
    if (!state.started || state.fault) {
        return;
    }
    if (request == 0 || !request->valid) {
        target.valid = false;
        return;
    }
    target = *request;
    target.left_speed = clamp_target(target.left_speed);
    target.right_speed = clamp_target(target.right_speed);
    state.left_requested = target.left_speed;
    state.right_requested = target.right_speed;
    watchdog_ms = 0U;
}

void Drive_Service(uint32_t now_ms)
{
    int16_t left = 0;
    int16_t right = 0;
    MotorFeedbackSnapshot feedback;
    float corrected_left = 0.0f;
    float corrected_right = 0.0f;

    if (!state.started || state.fault) {
        send_output(0, 0, now_ms);
        return;
    }
    if (!target.valid ||
        (uint32_t)(now_ms - target.timestamp_ms) > DRIVE_REQUEST_MAX_AGE_MS) {
        state.error = DRIVE_ERROR_STALE;
        send_output(0, 0, now_ms);
        return;
    }
    state.error = DRIVE_ERROR_NONE;
    corrected_left = (float)target.left_speed;
    corrected_right = (float)target.right_speed;
    if (MotorFeedback_GetSnapshot(&feedback, now_ms)) {
        float dt_s = last_feedback_ms == 0U ? 0.0f :
                     (float)(now_ms - last_feedback_ms) / 1000.0f;
        DifferentialController_Update(&differential_controller,
                                      (float)target.left_speed,
                                      (float)target.right_speed,
                                      feedback.left_speed_mm_s,
                                      feedback.right_speed_mm_s,
                                      dt_s, &corrected_left,
                                      &corrected_right);
        last_feedback_ms = now_ms;
    }
    left = interlocked(0U, ramped((int16_t)corrected_left));
    right = interlocked(1U, ramped((int16_t)corrected_right));
    send_output(left, right, now_ms);
}

void Drive_Tick1ms(void)
{
    uint8_t wheel;

    if (!state.started || state.fault) {
        return;
    }
    if (ramp_ms < DRIVE_RAMP_TOTAL_MS) {
        ramp_ms++;
    }
    if (watchdog_ms < DRIVE_WATCHDOG_MS) {
        watchdog_ms++;
    }
    for (wheel = 0U; wheel < 2U; wheel++) {
        int16_t applied = wheel == 0U ? state.left_applied : state.right_applied;
        if (applied == 0 && last_sign[wheel] != 0 &&
            zero_ms[wheel] <= DRIVE_REVERSE_PAUSE_MS) {
            zero_ms[wheel]++;
        }
    }
    if (watchdog_ms >= DRIVE_WATCHDOG_MS) {
        state.fault = true;
        state.error = DRIVE_ERROR_WATCHDOG;
        state.left_applied = 0;
        state.right_applied = 0;
        (void)Motor_EmergencyStop_FromISR();
    }
}

void Drive_GetStatus(DriveStatus *status)
{
    if (status != 0) {
        *status = state;
    }
}
