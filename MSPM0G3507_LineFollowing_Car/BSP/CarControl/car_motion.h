#ifndef __CAR_MOTION_H__
#define __CAR_MOTION_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * Encoder feedback is deliberately reported as raw driver values for now.
 * The field names retain the planned units for API stability, but
 * units_valid remains false until pulses/rev, gear ratio, effective 65 mm
 * wheel diameter and the driver's conversion have all been measured.
 */
typedef struct {
    int32_t left_ticks;
    int32_t right_ticks;
    float left_speed_mm_s;
    float right_speed_mm_s;
    uint32_t timestamp_ms;
    bool units_valid;
} CarMotionFeedback;

void CarMotion_Reset(void);
bool CarMotion_ReadFeedback(CarMotionFeedback *feedback);
void CarMotion_Command(int16_t linear_speed, int16_t angular_command);
void CarMotion_Stop(void);
bool CarMotion_DriveDistanceStart(int32_t distance_mm, int16_t speed);
bool CarMotion_DriveDistanceStep(void);
bool CarMotion_TurnAngleStart(float angle_deg, int16_t speed);
bool CarMotion_TurnAngleStep(void);

#endif
