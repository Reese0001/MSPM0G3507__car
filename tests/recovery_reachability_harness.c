#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "application/corner_maneuver.h"
#include "modules/line_tracking/recovery/line_recovery.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        (void)fprintf(stderr, "check failed at line %d: %s\n", \
                      __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static void set_features(LineFeatures *features,
                         uint32_t now_ms,
                         uint16_t sequence,
                         uint8_t active_count)
{
    (void)memset(features, 0, sizeof(*features));
    features->status.timestamp_ms = now_ms;
    features->status.sequence = sequence;
    features->status.valid = true;
    features->status.health = MODULE_HEALTH_OK;
    features->active_count = active_count;
    features->confidence = active_count == 0U ? 0U : 60U;
}

static void set_path_event(LinePathEvent *path_event,
                           uint32_t now_ms,
                           uint16_t sequence,
                           LinePathEventType type)
{
    (void)memset(path_event, 0, sizeof(*path_event));
    path_event->status.timestamp_ms = now_ms;
    path_event->status.sequence = sequence;
    path_event->status.valid = true;
    path_event->status.health = MODULE_HEALTH_OK;
    path_event->type = type;
}

static void set_line(LineEstimate *line,
                     uint32_t now_ms,
                     uint16_t sequence,
                     LineEvent event)
{
    (void)memset(line, 0, sizeof(*line));
    line->status.timestamp_ms = now_ms;
    line->status.sequence = sequence;
    line->status.valid = true;
    line->status.health = MODULE_HEALTH_OK;
    line->event = event;
}

static void set_trend(LineTrendResult *trend, uint32_t now_ms)
{
    (void)memset(trend, 0, sizeof(*trend));
    trend->status.timestamp_ms = now_ms;
    trend->status.valid = true;
    trend->status.health = MODULE_HEALTH_OK;
    trend->direction = -1;
}

static bool scheduler_step(const LineFeatures *features,
                           const LinePathEvent *path_event,
                           const LineEstimate *line,
                           const LineTrendResult *trend,
                           const LineControlOutput *follow,
                           uint32_t now_ms,
                           CornerManeuverOutput *corner,
                           MotionRequest *request)
{
    (void)CornerManeuver_Step(features, path_event, follow, false, now_ms,
                              corner);
    if (corner->owns_motion) {
        *request = corner->request;
        return true;
    }
    return LineRecovery_Step(line, trend, follow, 0.0f, false, false, now_ms,
                             request);
}

static int test_lost_follow_reaches_recovery_after_three_frames(void)
{
    const LineControlOutput lost_follow = {0, 0, false};
    LineFeatures features;
    LinePathEvent path_event;
    LineEstimate line;
    LineTrendResult trend;
    CornerManeuverOutput corner;
    MotionRequest request;
    uint32_t now_ms;
    uint16_t sequence;
    bool recovery_owns;

    CornerManeuver_Init();
    LineRecovery_Init();
    for (sequence = 1U; sequence <= 3U; sequence++) {
        now_ms = (uint32_t)(sequence - 1U) * 5U;
        set_features(&features, now_ms, sequence, 0U);
        set_path_event(&path_event, now_ms, sequence, LINE_PATH_LOST);
        set_line(&line, now_ms, sequence, LINE_EVENT_LOST);
        set_trend(&trend, now_ms);
        recovery_owns = scheduler_step(&features, &path_event, &line, &trend,
                                       &lost_follow, now_ms, &corner, &request);
        CHECK(!corner.fault);
        CHECK(!corner.owns_motion);
        CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_FOLLOW);
        if (sequence < 3U) {
            CHECK(!recovery_owns);
            CHECK(!request.valid);
        }
    }
    CHECK(recovery_owns);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_FORWARD_SEARCH);
    CHECK(request.valid && request.left_speed == 80 && request.right_speed == 120);
    return 0;
}

static int test_seek_keeps_pivot_during_temporary_lost_frames(void)
{
    const LineControlOutput follow = {200, 0, true};
    const LineControlOutput lost_follow = {0, 0, false};
    LineFeatures features;
    LinePathEvent path_event;
    CornerManeuverOutput corner;
    uint32_t now_ms;

    CornerManeuver_Init();
    set_features(&features, 0U, 1U, 2U);
    set_path_event(&path_event, 0U, 1U, LINE_PATH_RIGHT_ANGLE_LEFT);
    CHECK(CornerManeuver_Step(&features, &path_event, &follow, false, 0U,
                              &corner));
    CHECK(corner.owns_motion && !corner.fault);

    set_features(&features, 120U, 2U, 2U);
    set_path_event(&path_event, 120U, 2U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &path_event, &follow, false, 120U,
                              &corner));
    set_features(&features, 220U, 3U, 2U);
    set_path_event(&path_event, 220U, 3U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &path_event, &follow, false, 220U,
                              &corner));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_SEEK);

    for (now_ms = 225U; now_ms <= 230U; now_ms += 5U) {
        uint16_t sequence = (uint16_t)(now_ms / 5U) - 41U;

        set_features(&features, now_ms, sequence, 0U);
        set_path_event(&path_event, now_ms, sequence, LINE_PATH_LOST);
        CHECK(CornerManeuver_Step(&features, &path_event, &lost_follow, false,
                                  now_ms, &corner));
        CHECK(!corner.fault && corner.owns_motion);
        CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_SEEK);
        CHECK(corner.request.valid);
        CHECK(corner.request.left_speed == -80 &&
              corner.request.right_speed == 120);
    }
    return 0;
}

static int test_settle_loss_returns_control_to_recovery(void)
{
    const LineControlOutput follow = {200, 0, true};
    const LineControlOutput lost_follow = {0, 0, false};
    LineFeatures features;
    LinePathEvent path_event;
    LineEstimate line;
    LineTrendResult trend;
    CornerManeuverOutput corner;
    MotionRequest request;
    uint32_t now_ms;
    uint16_t sequence;
    bool recovery_owns;

    CornerManeuver_Init();
    LineRecovery_Init();
    set_features(&features, 0U, 1U, 2U);
    set_path_event(&path_event, 0U, 1U, LINE_PATH_RIGHT_ANGLE_LEFT);
    CHECK(CornerManeuver_Step(&features, &path_event, &follow, false, 0U,
                              &corner));
    set_features(&features, 120U, 2U, 2U);
    set_path_event(&path_event, 120U, 2U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &path_event, &follow, false, 120U,
                              &corner));
    set_features(&features, 220U, 3U, 2U);
    set_path_event(&path_event, 220U, 3U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &path_event, &follow, false, 220U,
                              &corner));

    for (sequence = 4U, now_ms = 225U;
         sequence <= 6U;
         sequence++, now_ms += 5U) {
        set_features(&features, now_ms, sequence, 1U);
        set_path_event(&path_event, now_ms, sequence, LINE_PATH_NORMAL);
        CHECK(CornerManeuver_Step(&features, &path_event, &follow, false,
                                  now_ms, &corner));
    }
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_SETTLE);

    now_ms = 240U;
    set_features(&features, now_ms, 7U, 0U);
    set_path_event(&path_event, now_ms, 7U, LINE_PATH_LOST);
    set_line(&line, now_ms, 7U, LINE_EVENT_LOST);
    set_trend(&trend, now_ms);
    recovery_owns = scheduler_step(&features, &path_event, &line, &trend,
                                   &lost_follow, now_ms, &corner, &request);
    CHECK(!corner.fault);
    CHECK(!corner.owns_motion);
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_FOLLOW);
    CHECK(!recovery_owns && !request.valid);
    CHECK(LineRecovery_GetState() == LINE_RECOVERY_LOSS_CONFIRM);
    return 0;
}

int main(void)
{
    if (test_lost_follow_reaches_recovery_after_three_frames() != 0) {
        return 1;
    }
    if (test_seek_keeps_pivot_during_temporary_lost_frames() != 0) {
        return 1;
    }
    return test_settle_loss_returns_control_to_recovery();
}
