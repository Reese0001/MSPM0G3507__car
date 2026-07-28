#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "modules/line_tracking/recovery/line_recovery.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        (void)fprintf(stderr, "check failed at line %d: %s\n", \
                      __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static bool request_check_failed = false;

static void set_line(LineEstimate *line,
                     uint32_t timestamp_ms,
                     uint16_t sequence,
                     LineEvent event,
                     uint8_t confidence)
{
    (void)memset(line, 0, sizeof(*line));
    line->status.timestamp_ms = timestamp_ms;
    line->status.sequence = sequence;
    line->status.valid = true;
    line->status.health = MODULE_HEALTH_OK;
    line->event = event;
    line->confidence = confidence;
}

static bool step(LineEstimate *line,
                 int8_t predicted_direction,
                 const LineControlOutput *follow,
                 float yaw_deg,
                 float yaw_rate_dps,
                 bool yaw_fresh,
                 bool emergency_stop,
                 uint32_t now_ms,
                 MotionRequest *request)
{
    bool owns_motion = LineRecovery_Step(line, predicted_direction, follow,
                                          yaw_deg, yaw_rate_dps, yaw_fresh,
                                          emergency_stop, now_ms, request);

    if (request->valid) {
        if (request->left_speed < 0 || request->left_speed > 140 ||
            request->right_speed < 0 || request->right_speed > 140) {
            request_check_failed = true;
        }
    } else {
        if (request->left_speed != 0 || request->right_speed != 0) {
            request_check_failed = true;
        }
    }
    return owns_motion;
}

static int check_seek(int8_t direction, int16_t left, int16_t right,
                      LineRecoveryState state)
{
    const LineControlOutput follow = {100, 20, true};
    LineEstimate line;
    LineRecoveryDiagnostics diagnostics;
    MotionRequest request;

    LineRecovery_Init();
    set_line(&line, 0U, 1U, LINE_EVENT_LOST, 0U);
    CHECK(step(&line, direction, &follow, 10.0f, 0.0f, true, false,
               0U, &request));
    CHECK(LineRecovery_GetState() == state);
    CHECK(request.valid && request.left_speed == left &&
          request.right_speed == right);
    LineRecovery_GetDiagnostics(&diagnostics);
    CHECK(diagnostics.state == state);
    CHECK(diagnostics.direction == (direction < 0 ? -1 : 1));
    CHECK(diagnostics.yaw_delta_deg == 0.0f);
    CHECK(diagnostics.yaw_fresh);
    return 0;
}

static int test_first_lost_frame_pivots_without_reverse(void)
{
    CHECK(check_seek(-1, 0, 100, LINE_RECOVERY_SEEK_LEFT) == 0);
    CHECK(check_seek(1, 100, 0, LINE_RECOVERY_SEEK_RIGHT) == 0);
    CHECK(check_seek(0, 100, 0, LINE_RECOVERY_SEEK_RIGHT) == 0);
    return 0;
}

static int test_direction_stays_locked_and_yaw_only_limits_speed(void)
{
    const LineControlOutput follow = {100, 20, true};
    LineEstimate line;
    LineRecoveryDiagnostics diagnostics;
    MotionRequest request;

    LineRecovery_Init();
    set_line(&line, 0U, 1U, LINE_EVENT_LOST, 0U);
    CHECK(step(&line, -1, &follow, 10.0f, 0.0f, true, false,
               0U, &request));

    set_line(&line, 5000U, 2U, LINE_EVENT_LOST, 0U);
    CHECK(step(&line, 1, &follow, 80.0f, 0.0f, true, false,
               5000U, &request));
    CHECK(request.left_speed == 0 && request.right_speed == 100);
    LineRecovery_GetDiagnostics(&diagnostics);
    CHECK(diagnostics.direction == -1 && diagnostics.yaw_delta_deg == 70.0f);

    set_line(&line, 5010U, 3U, LINE_EVENT_LOST, 0U);
    CHECK(step(&line, 1, &follow, 110.0f, -120.0f, true, false,
               5010U, &request));
    CHECK(request.left_speed == 0 && request.right_speed == 80);

    set_line(&line, 5020U, 4U, LINE_EVENT_LOST, 0U);
    CHECK(step(&line, 1, &follow, 370.0f, 1000.0f, false, false,
               5020U, &request));
    CHECK(request.left_speed == 0 && request.right_speed == 100);
    LineRecovery_GetDiagnostics(&diagnostics);
    CHECK(diagnostics.state == LINE_RECOVERY_SEEK_LEFT);
    CHECK(diagnostics.direction == -1);
    CHECK(diagnostics.yaw_delta_deg == 360.0f);
    CHECK(!diagnostics.yaw_fresh);
    return 0;
}

static int test_three_unique_trustworthy_frames_enter_align(void)
{
    const LineControlOutput follow = {30, 100, true};
    LineEstimate line;
    MotionRequest request;
    uint32_t now_ms = 0U;

    LineRecovery_Init();
    set_line(&line, now_ms, 1U, LINE_EVENT_LOST, 0U);
    CHECK(step(&line, -1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));

    now_ms += 5U;
    set_line(&line, now_ms, 2U, LINE_EVENT_NONE, 40U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_SEEK_LEFT);

    now_ms += 5U;
    set_line(&line, now_ms, 2U, LINE_EVENT_NONE, 100U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_SEEK_LEFT);

    now_ms += 5U;
    set_line(&line, now_ms, 3U, LINE_EVENT_HARD_LEFT, 100U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));

    now_ms += 5U;
    set_line(&line, now_ms, 4U, LINE_EVENT_NONE, 60U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    now_ms += 5U;
    set_line(&line, now_ms, 5U, LINE_EVENT_NONE, 60U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    now_ms += 5U;
    set_line(&line, now_ms, 5U, LINE_EVENT_NONE, 60U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_SEEK_LEFT);

    now_ms += 5U;
    set_line(&line, now_ms, 6U, LINE_EVENT_NONE, 60U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ALIGN);
    CHECK(request.valid && request.left_speed == 0 &&
          request.right_speed == 80);

    now_ms += 100U;
    set_line(&line, now_ms, 7U, LINE_EVENT_LOST, 0U);
    CHECK(step(&line, 1, &follow, 30.0f, 0.0f, true, false,
               now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_SEEK_LEFT);
    CHECK(request.left_speed == 0 && request.right_speed == 100);

    LineRecovery_Init();
    now_ms = 0U;
    set_line(&line, now_ms, 1U, LINE_EVENT_LOST, 0U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    now_ms += 5U;
    set_line(&line, now_ms, 2U, LINE_EVENT_NONE, 60U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    now_ms += 5U;
    set_line(&line, now_ms, 3U, LINE_EVENT_NONE, 60U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    now_ms += 5U;
    set_line(&line, now_ms, 4U, LINE_EVENT_NONE, 60U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ALIGN);

    now_ms += 299U;
    set_line(&line, now_ms, 5U, LINE_EVENT_NONE, 60U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ALIGN);
    CHECK(request.left_speed == 0 && request.right_speed == 80);

    now_ms += 1U;
    set_line(&line, now_ms, 6U, LINE_EVENT_NONE, 60U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_FOLLOW);
    CHECK(request.left_speed == 0 && request.right_speed == 130);
    return 0;
}

static int test_stopped_and_recovery_paths(void)
{
    const LineControlOutput follow = {100, 20, true};
    LineEstimate line;
    LineRecoveryDiagnostics diagnostics;
    MotionRequest request;
    uint32_t now_ms = 21U;

    LineRecovery_Init();
    set_line(&line, 0U, 1U, LINE_EVENT_NONE, 60U);
    CHECK(!step(&line, 1, &follow, 0.0f, 0.0f, true, false,
                now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_STOPPED);
    CHECK(!request.valid);

    set_line(&line, now_ms, 2U, LINE_EVENT_LOST, 0U);
    CHECK(step(&line, -1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_SEEK_LEFT);

    now_ms += 1U;
    set_line(&line, now_ms, 3U, LINE_EVENT_LOST, 0U);
    CHECK(!step(&line, 1, &follow, 0.0f, 0.0f, true, true,
                now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_STOPPED);
    CHECK(!request.valid);

    now_ms += 1U;
    set_line(&line, now_ms, 4U, LINE_EVENT_NONE, 60U);
    CHECK(!step(&line, 1, &follow, 0.0f, 0.0f, true, false,
                now_ms, &request));
    now_ms += 1U;
    set_line(&line, now_ms, 5U, LINE_EVENT_NONE, 60U);
    CHECK(!step(&line, 1, &follow, 0.0f, 0.0f, true, false,
                now_ms, &request));
    now_ms += 1U;
    set_line(&line, now_ms, 6U, LINE_EVENT_NONE, 60U);
    CHECK(step(&line, 1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ALIGN);

    LineRecovery_Init();
    now_ms = 21U;
    set_line(&line, 0U, 1U, LINE_EVENT_NONE, 60U);
    CHECK(!step(&line, -1, &follow, 0.0f, 0.0f, true, false,
                now_ms, &request));
    now_ms += 1U;
    set_line(&line, now_ms, 2U, LINE_EVENT_NONE, 60U);
    CHECK(!step(&line, -1, &follow, 0.0f, 0.0f, true, false,
                now_ms, &request));
    now_ms += 1U;
    set_line(&line, now_ms, 3U, LINE_EVENT_NONE, 60U);
    CHECK(!step(&line, -1, &follow, 0.0f, 0.0f, true, false,
                now_ms, &request));
    now_ms += 1U;
    set_line(&line, now_ms, 4U, LINE_EVENT_NONE, 60U);
    CHECK(step(&line, -1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_ALIGN);
    now_ms += 1U;
    set_line(&line, now_ms, 5U, LINE_EVENT_LOST, 0U);
    CHECK(step(&line, -1, &follow, 0.0f, 0.0f, true, false,
               now_ms, &request));
    LineRecovery_GetDiagnostics(&diagnostics);
    CHECK(diagnostics.state == LINE_RECOVERY_SEEK_RIGHT);
    CHECK(diagnostics.direction == 1);
    CHECK(request.left_speed == 100 && request.right_speed == 0);
    return 0;
}

int main(void)
{
    if (test_first_lost_frame_pivots_without_reverse() != 0) {
        return 1;
    }
    if (test_direction_stays_locked_and_yaw_only_limits_speed() != 0) {
        return 1;
    }
    if (test_three_unique_trustworthy_frames_enter_align() != 0) {
        return 1;
    }
    if (test_stopped_and_recovery_paths() != 0) {
        return 1;
    }
    CHECK(!request_check_failed);
    return 0;
}
