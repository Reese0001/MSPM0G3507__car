#ifndef MOTOR_FEEDBACK_H
#define MOTOR_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float left_speed_mm_s;
    float right_speed_mm_s;
    bool valid;
    uint32_t timestamp_ms;
    uint16_t sequence;
} MotorFeedbackSnapshot;

void MotorFeedback_Init(void);
bool MotorFeedback_ParseSpeedFrame(const char *frame, uint32_t now_ms);
void MotorFeedback_Publish(float left_speed_mm_s,
                           float right_speed_mm_s,
                           uint32_t now_ms);
bool MotorFeedback_IsFresh(uint32_t now_ms);
bool MotorFeedback_GetSnapshot(MotorFeedbackSnapshot *out,
                               uint32_t now_ms);
void MotorFeedback_UpdateOdometry(uint32_t now_ms);
void MotorFeedback_SetCommandSpeed(float average_speed_mm_s);
void MotorFeedback_ResetDistance(void);
float MotorFeedback_GetDistanceMm(void);

#endif
