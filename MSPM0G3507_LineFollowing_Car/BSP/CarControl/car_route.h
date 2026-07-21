#ifndef CAR_ROUTE_H
#define CAR_ROUTE_H

#include "car_motion.h"
#include "car_sensor_events.h"

typedef enum {
    CAR_ROUTE_IDLE = 0,
    CAR_ROUTE_LINE_FOLLOW,
    CAR_ROUTE_DRIVE_DISTANCE,
    CAR_ROUTE_TURN_ANGLE,
    CAR_ROUTE_SEARCH_LINE,
    CAR_ROUTE_TARGET_APPROACH,
    CAR_ROUTE_STOPPED,
    CAR_ROUTE_FAULT
} CarRouteState;

void CarRoute_Init(void);
void CarRoute_Start(void);
void CarRoute_Stop(void);
void CarRoute_Run(const CarSensorFrame *sensor,
                  const CarMotionFeedback *motion);
CarRouteState CarRoute_GetState(void);

#endif
