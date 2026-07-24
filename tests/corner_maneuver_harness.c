#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "application/corner_maneuver.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        (void)fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); \
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
    features->confidence = 60U;
    features->centroid_error = 0.0f;
}

static void set_event(LinePathEvent *event,
                      uint32_t now_ms,
                      LinePathEventType type)
{
    (void)memset(event, 0, sizeof(*event));
    event->status.timestamp_ms = now_ms;
    event->status.valid = true;
    event->status.health = MODULE_HEALTH_OK;
    event->type = type;
}

static void check_not_reversing(const CornerManeuverOutput *out)
{
    if (out->request.left_speed < 0 && out->request.right_speed < 0) {
        (void)fprintf(stderr, "both wheels reverse\n");
        exit(1);
    }
}

static int run_left_probe_sequence(void)
{
    LineFeatures features;
    LinePathEvent event;
    LineControlOutput follow = {200, 0, true};
    CornerManeuverOutput out;

    CornerManeuver_Init();
    set_features(&features, 0U, 1U, 4U);
    set_event(&event, 0U, LINE_PATH_WIDE_PENDING);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 0U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_FORWARD_PROBE);
    CHECK(out.owns_motion && out.request.valid);
    CHECK(out.request.left_speed == 100 && out.request.right_speed == 100);
    check_not_reversing(&out);

    set_features(&features, 5U, 2U, 4U);
    set_event(&event, 5U, LINE_PATH_RIGHT_ANGLE_LEFT);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 5U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_BRAKE);
    CHECK(out.request.left_speed == 0 && out.request.right_speed == 0);
    check_not_reversing(&out);

    set_features(&features, 124U, 2U, 4U);
    set_event(&event, 124U, LINE_PATH_RIGHT_ANGLE_LEFT);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 124U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_BRAKE);
    CHECK(out.request.left_speed == 0 && out.request.right_speed == 0);
    check_not_reversing(&out);

    set_features(&features, 125U, 2U, 4U);
    set_event(&event, 125U, LINE_PATH_RIGHT_ANGLE_LEFT);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 125U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_COMMIT);
    CHECK(out.request.left_speed == -80 && out.request.right_speed == 120);
    check_not_reversing(&out);

    set_features(&features, 225U, 2U, 4U);
    set_event(&event, 225U, LINE_PATH_RIGHT_ANGLE_LEFT);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 225U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_SEEK);
    CHECK(out.request.left_speed == -80 && out.request.right_speed == 120);
    check_not_reversing(&out);

    set_features(&features, 230U, 3U, 1U);
    set_event(&event, 230U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 230U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_SEEK);
    check_not_reversing(&out);

    set_features(&features, 235U, 3U, 1U);
    set_event(&event, 235U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 235U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_SEEK);
    check_not_reversing(&out);

    set_features(&features, 240U, 4U, 2U);
    set_event(&event, 240U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 240U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_SEEK);
    check_not_reversing(&out);

    set_features(&features, 245U, 5U, 3U);
    set_event(&event, 245U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 245U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_SETTLE);
    CHECK(out.request.left_speed == 200 && out.request.right_speed == 200);
    check_not_reversing(&out);

    set_features(&features, 545U, 6U, 2U);
    set_event(&event, 545U, LINE_PATH_RIGHT_ANGLE_RIGHT);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 545U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_FOLLOW);
    CHECK(out.completed && !out.fault);
    check_not_reversing(&out);

    set_features(&features, 550U, 7U, 2U);
    set_event(&event, 550U, LINE_PATH_RIGHT_ANGLE_RIGHT);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 550U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_FOLLOW);
    CHECK(!out.completed && !out.fault && !out.owns_motion);
    check_not_reversing(&out);
    return 0;
}

static int run_right_mirror_and_faults(void)
{
    LineFeatures features;
    LinePathEvent event;
    LineControlOutput follow = {200, 0, true};
    CornerManeuverOutput out;

    CornerManeuver_Init();
    set_features(&features, 0U, 1U, 4U);
    set_event(&event, 0U, LINE_PATH_RIGHT_ANGLE_RIGHT);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 0U, &out));
    CHECK(out.request.left_speed == 0 && out.request.right_speed == 0);
    set_features(&features, 120U, 2U, 4U);
    set_event(&event, 120U, LINE_PATH_RIGHT_ANGLE_RIGHT);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 120U, &out));
    CHECK(out.request.left_speed == 120 && out.request.right_speed == -80);
    check_not_reversing(&out);

    CornerManeuver_Init();
    set_features(&features, 0U, 1U, 2U);
    set_event(&event, 0U, LINE_PATH_NORMAL);
    CHECK(!CornerManeuver_Step(0, &event, &follow, false, 0U, &out));
    CHECK(out.fault && !out.completed && !out.request.valid);
    CHECK(out.request.left_speed == 0 && out.request.right_speed == 0);

    CornerManeuver_Init();
    set_features(&features, 0U, 1U, 2U);
    set_event(&event, 0U, LINE_PATH_NORMAL);
    CHECK(!CornerManeuver_Step(&features, &event, &follow, true, 0U, &out));
    CHECK(out.fault && !out.completed && !out.request.valid);
    CHECK(out.request.left_speed == 0 && out.request.right_speed == 0);

    CornerManeuver_Init();
    set_features(&features, 0U, 1U, 2U);
    set_event(&event, 0U, LINE_PATH_NORMAL);
    CHECK(!CornerManeuver_Step(&features, &event, &follow, false, 21U, &out));
    CHECK(out.fault && !out.completed && !out.request.valid);
    CHECK(out.request.left_speed == 0 && out.request.right_speed == 0);

    CornerManeuver_Init();
    set_features(&features, 0U, 1U, 4U);
    set_event(&event, 0U, LINE_PATH_WIDE_PENDING);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 0U, &out));
    set_features(&features, 2000U, 2U, 4U);
    set_event(&event, 2000U, LINE_PATH_NORMAL);
    CHECK(!CornerManeuver_Step(&features, &event, &follow, false, 2000U, &out));
    CHECK(out.fault && !out.completed && !out.request.valid);
    CHECK(out.request.left_speed == 0 && out.request.right_speed == 0);
    return 0;
}

static int run_timeout_boundaries_and_follow_safety(void)
{
    LineFeatures features;
    LinePathEvent event;
    LineControlOutput follow = {200, 0, true};
    LineControlOutput reverse_follow = {-100, 0, true};
    LineControlOutput over_limit_follow = {450, 450, true};
    CornerManeuverOutput out;

    CornerManeuver_Init();
    set_features(&features, 0U, 1U, 4U);
    set_event(&event, 0U, LINE_PATH_WIDE_PENDING);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 0U, &out));
    set_features(&features, 79U, 2U, 4U);
    set_event(&event, 79U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 79U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_FORWARD_PROBE);
    CHECK(out.request.valid);
    CHECK(out.request.left_speed == 100 && out.request.right_speed == 100);
    check_not_reversing(&out);
    set_features(&features, 80U, 3U, 4U);
    set_event(&event, 80U, LINE_PATH_NORMAL);
    CHECK(!CornerManeuver_Step(&features, &event, &follow, false, 80U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_FAULT);
    CHECK(out.fault && !out.completed && !out.request.valid);
    CHECK(out.request.left_speed == 0 && out.request.right_speed == 0);

    CornerManeuver_Init();
    set_features(&features, 0U, 1U, 4U);
    set_event(&event, 0U, LINE_PATH_RIGHT_ANGLE_LEFT);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 0U, &out));
    set_features(&features, 120U, 2U, 4U);
    set_event(&event, 120U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 120U, &out));
    set_features(&features, 220U, 3U, 4U);
    set_event(&event, 220U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 220U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_SEEK);
    set_features(&features, 1119U, 4U, 4U);
    set_event(&event, 1119U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &event, &follow, false, 1119U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_SEEK);
    CHECK(out.request.left_speed == -80 && out.request.right_speed == 120);
    check_not_reversing(&out);
    set_features(&features, 1120U, 5U, 4U);
    set_event(&event, 1120U, LINE_PATH_NORMAL);
    CHECK(!CornerManeuver_Step(&features, &event, &follow, false, 1120U, &out));
    CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_FAULT);
    CHECK(out.fault && !out.completed && !out.request.valid);
    CHECK(out.request.left_speed == 0 && out.request.right_speed == 0);

    CornerManeuver_Init();
    set_features(&features, 0U, 1U, 2U);
    set_event(&event, 0U, LINE_PATH_NORMAL);
    CHECK(!CornerManeuver_Step(&features, &event, &reverse_follow,
                               false, 0U, &out));
    CHECK(out.fault && !out.completed && !out.request.valid);
    CHECK(out.request.left_speed == 0 && out.request.right_speed == 0);

    CornerManeuver_Init();
    set_features(&features, 0U, 1U, 2U);
    set_event(&event, 0U, LINE_PATH_NORMAL);
    CHECK(CornerManeuver_Step(&features, &event, &over_limit_follow,
                              false, 0U, &out));
    CHECK(!out.fault && out.request.valid && !out.owns_motion);
    CHECK(out.request.left_speed == 0 && out.request.right_speed == 450);
    CHECK(out.request.left_speed >= -450 && out.request.left_speed <= 450);
    CHECK(out.request.right_speed >= -450 && out.request.right_speed <= 450);
    check_not_reversing(&out);
    return 0;
}

int main(void)
{
    if (run_left_probe_sequence() != 0) {
        return 1;
    }
    if (run_right_mirror_and_faults() != 0) {
        return 1;
    }
    return run_timeout_boundaries_and_follow_safety();
}
