#include "motor_feedback.h"

#include <stdio.h>
#include <string.h>

#define MOTOR_FEEDBACK_STALE_MS (50U)

static MotorFeedbackSnapshot snapshot;
static uint32_t odometry_ms;
static float distance_mm;
static float command_speed_mm_s;
static bool have_odometry_time;

void MotorFeedback_Init(void)
{
    snapshot = (MotorFeedbackSnapshot){0};
    odometry_ms = 0U;
    distance_mm = 0.0f;
    command_speed_mm_s = 0.0f;
    have_odometry_time = false;
}

bool MotorFeedback_ParseSpeedFrame(const char *frame, uint32_t now_ms)
{
    float values[4];

    if (frame == 0 || strncmp(frame, "MSPD:", 5U) != 0 ||
        sscanf(frame + 5, "%f,%f,%f,%f",
               &values[0], &values[1], &values[2], &values[3]) != 4) {
        return false;
    }
    /* Driver-board order is M1,M2,M3,M4; the car uses M4 left and M2 right. */
    MotorFeedback_Publish(values[3], values[1], now_ms);
    return true;
}

void MotorFeedback_Publish(float left_speed_mm_s,
                           float right_speed_mm_s,
                           uint32_t now_ms)
{
    snapshot.left_speed_mm_s = left_speed_mm_s;
    snapshot.right_speed_mm_s = right_speed_mm_s;
    snapshot.valid = true;
    snapshot.timestamp_ms = now_ms;
    snapshot.sequence++;
}

bool MotorFeedback_IsFresh(uint32_t now_ms)
{
    return snapshot.valid &&
           (uint32_t)(now_ms - snapshot.timestamp_ms) <=
               MOTOR_FEEDBACK_STALE_MS;
}

bool MotorFeedback_GetSnapshot(MotorFeedbackSnapshot *out, uint32_t now_ms)
{
    if (out == 0 || !MotorFeedback_IsFresh(now_ms)) {
        return false;
    }
    *out = snapshot;
    return true;
}

void MotorFeedback_UpdateOdometry(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    float average_speed;

    if (!have_odometry_time) {
        odometry_ms = now_ms;
        have_odometry_time = true;
        return;
    }
    elapsed_ms = (uint32_t)(now_ms - odometry_ms);
    odometry_ms = now_ms;
    if (elapsed_ms > 200U) {
        return;
    }
    average_speed = MotorFeedback_IsFresh(now_ms) ?
                    (snapshot.left_speed_mm_s +
                     snapshot.right_speed_mm_s) * 0.5f :
                    command_speed_mm_s;
    distance_mm += average_speed * ((float)elapsed_ms / 1000.0f);
}

void MotorFeedback_SetCommandSpeed(float average_speed_mm_s)
{
    command_speed_mm_s = average_speed_mm_s;
}

void MotorFeedback_ResetDistance(void)
{
    distance_mm = 0.0f;
    command_speed_mm_s = 0.0f;
    have_odometry_time = false;
}

float MotorFeedback_GetDistanceMm(void)
{
    return distance_mm;
}
