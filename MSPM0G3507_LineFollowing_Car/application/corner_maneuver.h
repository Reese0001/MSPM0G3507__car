#ifndef CORNER_MANEUVER_H
#define CORNER_MANEUVER_H

#include <stdbool.h>
#include <stdint.h>

#include "../modules/common/motion_request.h"
#include "../modules/line_tracking/line_controller.h"
#include "../modules/line_tracking/line_event_classifier.h"
#include "../modules/line_tracking/line_features.h"

typedef enum {
    CORNER_MANEUVER_FOLLOW = 0,
    CORNER_MANEUVER_FORWARD_PROBE,
    CORNER_MANEUVER_BRAKE,
    CORNER_MANEUVER_COMMIT,
    CORNER_MANEUVER_SEEK,
    CORNER_MANEUVER_SETTLE,
    CORNER_MANEUVER_FAULT
} CornerManeuverState;

typedef struct {
    MotionRequest request;
    bool owns_motion;
    bool completed;
    bool fault;
} CornerManeuverOutput;

void CornerManeuver_Init(void);
void CornerManeuver_Reset(void);
CornerManeuverState CornerManeuver_GetState(void);
bool CornerManeuver_Step(const LineFeatures *features,
                         const LinePathEvent *path_event,
                         const LineControlOutput *follow,
                         bool emergency_stop,
                         uint32_t now_ms,
                         CornerManeuverOutput *out);
bool CornerManeuver_StepWithYaw(const LineFeatures *features,
                                const LinePathEvent *path_event,
                                const LineControlOutput *follow,
                                float yaw_rate_dps,
                                bool yaw_fresh,
                                bool emergency_stop,
                                uint32_t now_ms,
                                CornerManeuverOutput *out);

#endif
