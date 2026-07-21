#include "car_route.h"

#include <stdint.h>

#include "../Motor/motor_safety.h"

/* The route layer only requests bounded commands; Motor_Safety applies the
 * startup ramp and watchdog before anything reaches the driver UART. */
#define CAR_ROUTE_FORWARD_COMMAND      (300)
#define CAR_ROUTE_TURN_GAIN             (80.0f)
#define CAR_ROUTE_TURN_LIMIT            (300)
#define CAR_ROUTE_STOP_MARKER_FRAMES   (2U)

static CarRouteState route_state = CAR_ROUTE_IDLE;
static uint8_t stop_marker_frames = 0U;

extern uint32_t Get_Time(void);

static void CarRoute_FaultStop(void)
{
    CarMotion_Stop();
    stop_marker_frames = 0U;
    route_state = CAR_ROUTE_FAULT;
}

static int16_t CarRoute_TurnCommand(float weighted_error)
{
    float command = weighted_error * CAR_ROUTE_TURN_GAIN;

    if (command > (float)CAR_ROUTE_TURN_LIMIT) {
        command = (float)CAR_ROUTE_TURN_LIMIT;
    } else if (command < -(float)CAR_ROUTE_TURN_LIMIT) {
        command = -(float)CAR_ROUTE_TURN_LIMIT;
    }
    return (int16_t)command;
}

void CarRoute_Init(void)
{
    CarMotion_Reset();
    stop_marker_frames = 0U;
    route_state = CAR_ROUTE_IDLE;
}

void CarRoute_Start(void)
{
    if (route_state != CAR_ROUTE_IDLE && route_state != CAR_ROUTE_STOPPED) {
        return;
    }
    if (Motor_Safety_IsFaultLatched() != 0U) {
        route_state = CAR_ROUTE_FAULT;
        CarMotion_Stop();
        return;
    }

    /* Start is deliberately explicit.  Arming here also makes a route start
     * safe when the caller did not previously arm the motor layer. */
    Motor_Safety_Arm();
    stop_marker_frames = 0U;
    route_state = CAR_ROUTE_LINE_FOLLOW;
}

void CarRoute_Stop(void)
{
    CarMotion_Stop();
    stop_marker_frames = 0U;
    route_state = CAR_ROUTE_STOPPED;
}

void CarRoute_Run(const CarSensorFrame *sensor,
                  const CarMotionFeedback *motion)
{
    CarLineEvent classified_event;

    switch (route_state) {
    case CAR_ROUTE_IDLE:
    case CAR_ROUTE_STOPPED:
        return;

    case CAR_ROUTE_LINE_FOLLOW:
        /* CarMotion_ReadFeedback already rejects old frames.  The timestamp
         * check keeps a caller from feeding an uninitialised/stale snapshot. */
        if (sensor == (const CarSensorFrame *)0 ||
            motion == (const CarMotionFeedback *)0 ||
            motion->timestamp_ms == 0U ||
            (Get_Time() - motion->timestamp_ms) >=
                CAR_MOTION_FEEDBACK_MAX_AGE_MS ||
            !sensor->line_valid) {
            CarRoute_FaultStop();
            return;
        }

        classified_event = CarSensor_Classify(sensor);
        if (classified_event != sensor->event) {
            CarRoute_FaultStop();
            return;
        }

        if (classified_event == CAR_LINE_NONE) {
            CarRoute_FaultStop();
            return;
        }
        if (classified_event == CAR_LINE_STOP_MARKER) {
            /* Hold still while debouncing; do not preserve the previous
             * forward request during the first candidate frame. */
            CarMotion_Stop();
            if (stop_marker_frames < CAR_ROUTE_STOP_MARKER_FRAMES) {
                stop_marker_frames++;
            }
            if (stop_marker_frames >= CAR_ROUTE_STOP_MARKER_FRAMES) {
                CarRoute_Stop();
            }
            return;
        }

        stop_marker_frames = 0U;
        CarMotion_Command(CAR_ROUTE_FORWARD_COMMAND,
                          CarRoute_TurnCommand(sensor->weighted_error));
        return;

    case CAR_ROUTE_DRIVE_DISTANCE:
    case CAR_ROUTE_TURN_ANGLE:
        /* Task 3B must supply calibrated encoder/yaw units before either
         * operation is enabled.  Fail closed even if a stale state is forced. */
        if (motion == (const CarMotionFeedback *)0 || !motion->units_valid) {
            CarRoute_FaultStop();
            return;
        }
        CarRoute_FaultStop();
        return;

    case CAR_ROUTE_SEARCH_LINE:
    case CAR_ROUTE_TARGET_APPROACH:
    default:
        /* These strategies are intentionally not enabled for this hardware
         * revision; no automatic search or target movement is permitted. */
        CarRoute_FaultStop();
        return;

    case CAR_ROUTE_FAULT:
        CarMotion_Stop();
        return;
    }
}

CarRouteState CarRoute_GetState(void)
{
    return route_state;
}
