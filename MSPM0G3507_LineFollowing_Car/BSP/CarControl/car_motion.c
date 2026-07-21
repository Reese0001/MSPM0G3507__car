#include "car_motion.h"

#include "app_motor.h"
#include "app_motor_usart.h"

/* Timer.c owns the millisecond tick; keeping this dependency narrow avoids
 * pulling timer peripheral headers into the public car-motion API. */
extern uint32_t Get_Time(void);

/* Conversion is intentionally disabled until the assembled chassis is
 * measured.  In particular, the advertised 65 mm wheel diameter is not yet
 * an effective rolling diameter and must not be used to label raw counts. */
#define CAR_MOTION_UNITS_CONFIRMED 0
#define CAR_MOTION_FEEDBACK_MAX_AGE_MS 200U

typedef enum {
    CAR_MOTION_ACTION_NONE = 0,
    CAR_MOTION_ACTION_DISTANCE,
    CAR_MOTION_ACTION_TURN
} CarMotionAction;

static CarMotionAction motion_action = CAR_MOTION_ACTION_NONE;
static int16_t motion_speed = 0;
static int32_t motion_distance_mm = 0;
static float motion_angle_deg = 0.0f;

void CarMotion_Reset(void)
{
    motion_action = CAR_MOTION_ACTION_NONE;
    motion_speed = 0;
    motion_distance_mm = 0;
    motion_angle_deg = 0.0f;
    CarMotion_Stop();
}

bool CarMotion_ReadFeedback(CarMotionFeedback *feedback)
{
    uint32_t now;

    if (feedback == (CarMotionFeedback *)0) {
        return false;
    }

    now = Get_Time();
    if (g_motor_feedback_valid == 0U ||
        (now - g_motor_feedback_timestamp_ms) > CAR_MOTION_FEEDBACK_MAX_AGE_MS) {
        return false;
    }

    /* M2 is the left drive and M4 is the right drive.  M1/M3 are castors. */
    feedback->left_ticks = Encoder_Offset[1];
    feedback->right_ticks = Encoder_Offset[3];
    feedback->left_speed_mm_s = g_Speed[1];
    feedback->right_speed_mm_s = g_Speed[3];
    feedback->timestamp_ms = g_motor_feedback_timestamp_ms;
    /* Raw counts/speed cannot be called millimetres until calibration. */
    feedback->units_valid = false;
    return true;
}

void CarMotion_Command(int16_t linear_speed, int16_t angular_command)
{
    /* Motion_Car_Control is the sole command boundary; it routes through the
     * Motor Safety layer and keeps M1/M3 at zero for this 2WD chassis. */
    Motion_Car_Control(linear_speed, 0, angular_command);
}

void CarMotion_Stop(void)
{
    motion_action = CAR_MOTION_ACTION_NONE;
    motion_speed = 0;
    Motion_Car_Control(0, 0, 0);
}

bool CarMotion_DriveDistanceStart(int32_t distance_mm, int16_t speed)
{
    motion_distance_mm = distance_mm;
    motion_speed = speed;

#if CAR_MOTION_UNITS_CONFIRMED
    if (distance_mm == 0 || speed == 0) {
        CarMotion_Stop();
        return false;
    }
    motion_action = CAR_MOTION_ACTION_DISTANCE;
    CarMotion_Command(speed, 0);
    return true;
#else
    /* Reject rather than silently treating encoder ticks as millimetres. */
    (void)distance_mm;
    (void)speed;
    CarMotion_Stop();
    return false;
#endif
}

bool CarMotion_DriveDistanceStep(void)
{
    if (motion_action == CAR_MOTION_ACTION_NONE) {
        CarMotion_Stop();
        return false;
    }

#if CAR_MOTION_UNITS_CONFIRMED
    /* The calibrated implementation will compare a fresh M2/M4 snapshot and
     * stop in this bounded, cooperative step. */
    (void)motion_distance_mm;
    CarMotion_Command(motion_speed, 0);
    return true;
#else
    CarMotion_Stop();
    return false;
#endif
}

bool CarMotion_TurnAngleStart(float angle_deg, int16_t speed)
{
    motion_angle_deg = angle_deg;
    motion_speed = speed;

#if CAR_MOTION_UNITS_CONFIRMED
    if (angle_deg == 0.0f || speed == 0) {
        CarMotion_Stop();
        return false;
    }
    motion_action = CAR_MOTION_ACTION_TURN;
    CarMotion_Command(0, speed);
    return true;
#else
    /* Angle conversion also depends on the unmeasured wheel/encoder scale. */
    (void)angle_deg;
    (void)speed;
    CarMotion_Stop();
    return false;
#endif
}

bool CarMotion_TurnAngleStep(void)
{
    if (motion_action == CAR_MOTION_ACTION_NONE) {
        CarMotion_Stop();
        return false;
    }

#if CAR_MOTION_UNITS_CONFIRMED
    (void)motion_angle_deg;
    CarMotion_Command(0, motion_speed);
    return true;
#else
    CarMotion_Stop();
    return false;
#endif
}
