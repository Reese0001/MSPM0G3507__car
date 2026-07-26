#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "application/line_recovery.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        (void)fprintf(stderr, "check failed at line %d: %s\n", \
                      __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static void set_line(LineEstimate *line,
                     uint32_t now_ms,
                     uint16_t sequence,
                     LineEvent event,
                     float predicted_error,
                     float error,
                     uint8_t confidence)
{
    (void)memset(line, 0, sizeof(*line));
    line->status.timestamp_ms = now_ms;
    line->status.sequence = sequence;
    line->status.valid = true;
    line->status.health = MODULE_HEALTH_OK;
    line->event = event;
    line->predicted_error = predicted_error;
    line->error = error;
    line->confidence = confidence;
}

static void set_trend(LineTrendResult *trend,
                      uint32_t now_ms,
                      int8_t direction)
{
    (void)memset(trend, 0, sizeof(*trend));
    trend->status.timestamp_ms = now_ms;
    trend->status.valid = true;
    trend->status.health = MODULE_HEALTH_OK;
    trend->direction = direction;
}

static int step(LineEstimate *line,
                LineTrendResult *trend,
                uint32_t now_ms,
                bool emergency_stop,
                MotionRequest *request,
                bool *owns_motion)
{
    const LineControlOutput follow = {200, 0, true};

    *owns_motion = LineRecovery_Step(line, trend, &follow, 0.0f, false,
                                      emergency_stop, now_ms, request);
    CHECK(request->left_speed >= -450 && request->left_speed <= 450);
    CHECK(request->right_speed >= -450 && request->right_speed <= 450);
    CHECK(!(request->left_speed < 0 && request->right_speed < 0));
    return 0;
}

static int enter_forward_search(int8_t direction,
                                uint32_t *now_ms,
                                uint16_t *sequence,
                                LineEstimate *line,
                                LineTrendResult *trend,
                                MotionRequest *request)
{
    bool owns_motion;

    LineRecovery_Init();
    set_trend(trend, *now_ms, direction);
    set_line(line, *now_ms, (*sequence)++, LINE_EVENT_LOST,
             0.0f, 0.0f, 0U);
    CHECK(step(line, trend, *now_ms, false, request, &owns_motion) == 0);
    CHECK(!owns_motion);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_LOSS_CONFIRM);

    *now_ms += 5U;
    set_trend(trend, *now_ms, direction);
    set_line(line, *now_ms, (*sequence)++, LINE_EVENT_LOST,
             0.0f, 0.0f, 0U);
    CHECK(step(line, trend, *now_ms, false, request, &owns_motion) == 0);
    CHECK(!owns_motion);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_LOSS_CONFIRM);

    *now_ms += 5U;
    set_trend(trend, *now_ms, direction);
    set_line(line, *now_ms, (*sequence)++, LINE_EVENT_LOST,
             0.0f, 0.0f, 0U);
    CHECK(step(line, trend, *now_ms, false, request, &owns_motion) == 0);
    CHECK(owns_motion);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_FORWARD_SEARCH);
    return 0;
}

static int test_forward_pause_rotate_and_lock(void)
{
    LineEstimate line;
    LineTrendResult trend;
    MotionRequest request;
    bool owns_motion;
    uint32_t now_ms = 0U;
    uint16_t sequence = 1U;

    CHECK(enter_forward_search(-1, &now_ms, &sequence, &line, &trend,
                               &request) == 0);
    CHECK(request.valid && request.left_speed == 80 && request.right_speed == 120);

    now_ms += 499U;
    set_trend(&trend, now_ms, 1);
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(owns_motion && request.left_speed == 80 && request.right_speed == 120);

    now_ms += 1U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ROTATION_PAUSE);
    CHECK(request.valid && request.left_speed == 0 && request.right_speed == 0);

    now_ms += 119U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(request.valid && request.left_speed == 0 && request.right_speed == 0);

    now_ms += 1U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ROTATE_SEARCH);
    CHECK(request.valid && request.left_speed == -60 && request.right_speed == 100);

    now_ms += 699U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(owns_motion && request.left_speed == -60 && request.right_speed == 100);

    now_ms += 1U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(!owns_motion && !request.valid);
    CHECK(request.left_speed == 0 && request.right_speed == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_STOPPED);

    /* STOPPED is recoverable: trustworthy frames resume via ALIGN. */
    now_ms += 5U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, 0.0f, 0.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_STOPPED);
    now_ms += 5U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, 0.0f, 0.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_STOPPED);
    now_ms += 5U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, 0.0f, 0.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(owns_motion && request.valid);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ALIGN);
    now_ms += 300U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, 0.0f, 0.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_FOLLOW);
    return 0;
}

static int test_right_pivot_and_reacquisition(void)
{
    LineEstimate line;
    LineTrendResult trend;
    MotionRequest request;
    bool owns_motion;
    uint32_t now_ms = 0U;
    uint16_t sequence = 1U;

    CHECK(enter_forward_search(1, &now_ms, &sequence, &line, &trend,
                               &request) == 0);
    CHECK(request.left_speed == 120 && request.right_speed == 80);
    now_ms += 500U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    now_ms += 120U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(request.left_speed == 100 && request.right_speed == -60);

    now_ms += 5U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, 0.0f, 0.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ROTATE_SEARCH);
    now_ms += 5U;
    set_line(&line, now_ms, sequence - 1U, LINE_EVENT_NONE,
             0.0f, 0.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ROTATE_SEARCH);
    now_ms += 5U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, 0.0f, 0.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    now_ms += 5U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, 0.0f, 0.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ALIGN);
    CHECK(request.valid && request.left_speed == 200 && request.right_speed == 200);

    now_ms += 299U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, 0.0f, 0.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ALIGN);
    now_ms += 1U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, 0.0f, 0.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_FOLLOW);
    return 0;
}

static int test_duplicate_missing_direction_and_faults(void)
{
    LineEstimate line;
    LineTrendResult trend;
    MotionRequest request;
    bool owns_motion;
    uint32_t now_ms = 0U;
    uint16_t sequence = 1U;

    LineRecovery_Init();
    set_trend(&trend, now_ms, -1);
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    now_ms += 5U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    now_ms += 5U;
    set_line(&line, now_ms, sequence - 1U, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_LOSS_CONFIRM);
    now_ms += 5U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_FORWARD_SEARCH);

    LineRecovery_Init();
    now_ms = 0U;
    sequence = 1U;
    CHECK(enter_forward_search(0, &now_ms, &sequence, &line, &trend,
                               &request) == 0);
    CHECK(request.left_speed == 100 && request.right_speed == 100);
    now_ms += 500U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    now_ms += 120U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(!owns_motion && !request.valid);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_STOPPED);

    LineRecovery_Init();
    set_line(&line, 0U, 1U, LINE_EVENT_NONE, 0.0f, 0.0f, 60U);
    set_trend(&trend, 0U, 0);
    CHECK(step(&line, &trend, 21U, false, &request, &owns_motion) == 0);
    CHECK(!owns_motion && LineRecovery_GetState() == LINE_RECOVERY_STOPPED);

    LineRecovery_Init();
    set_line(&line, 0U, 1U, LINE_EVENT_NONE, 0.0f, 0.0f, 60U);
    set_trend(&trend, 0U, 0);
    CHECK(step(&line, &trend, 0U, true, &request, &owns_motion) == 0);
    CHECK(!owns_motion && LineRecovery_GetState() == LINE_RECOVERY_STOPPED);

    now_ms = 0U;
    sequence = 1U;
    CHECK(enter_forward_search(-1, &now_ms, &sequence, &line, &trend,
                               &request) == 0);
    now_ms = 2000U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(!owns_motion && LineRecovery_GetState() == LINE_RECOVERY_STOPPED);
    return 0;
}

/* Plan regression: stable left line, loss, blind forward-left search,
 * same-direction rotate, reacquisition -> ALIGN -> FOLLOW, no fault. */
static int test_left_loss_recovers_without_latching(void)
{
    LineEstimate line;
    LineTrendResult trend;
    MotionRequest request;
    bool owns_motion;
    uint32_t now_ms = 0U;
    uint16_t sequence = 1U;

    LineRecovery_Init();
    set_trend(&trend, now_ms, -1);
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, -5.0f, -5.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(owns_motion && LineRecovery_GetState() == LINE_RECOVERY_FOLLOW);

    CHECK(enter_forward_search(-1, &now_ms, &sequence, &line, &trend,
                               &request) == 0);
    CHECK(request.valid && request.left_speed == 80 &&
          request.right_speed == 120);

    now_ms += 500U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    now_ms += 120U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_LOST, 0.0f, 0.0f, 0U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ROTATE_SEARCH);
    CHECK(request.left_speed == -60 && request.right_speed == 100);

    now_ms += 5U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, -2.0f, -2.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    now_ms += 5U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, -2.0f, -2.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    now_ms += 5U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, -2.0f, -2.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ALIGN);

    now_ms += 300U;
    set_line(&line, now_ms, sequence++, LINE_EVENT_NONE, -2.0f, -2.0f, 60U);
    CHECK(step(&line, &trend, now_ms, false, &request, &owns_motion) == 0);
    CHECK(owns_motion && LineRecovery_GetState() == LINE_RECOVERY_FOLLOW);
    return 0;
}

int main(void)
{
    if (test_forward_pause_rotate_and_lock() != 0) {
        return 1;
    }
    if (test_right_pivot_and_reacquisition() != 0) {
        return 1;
    }
    if (test_duplicate_missing_direction_and_faults() != 0) {
        return 1;
    }
    return test_left_loss_recovers_without_latching();
}
