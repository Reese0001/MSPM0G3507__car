#ifndef MOTION_PRIMITIVES_H
#define MOTION_PRIMITIVES_H

#include <stdbool.h>
#include <stdint.h>

#include "../modules/common/motion_request.h"
#include "../modules/line_tracking/line_estimator.h"

typedef enum {
    MOTION_RUNNING = 0,
    MOTION_COMPLETE,
    MOTION_FAILED
} MotionResult;

typedef enum {
    MOTION_PRIMITIVE_FOLLOW_LINE = 0,
    MOTION_PRIMITIVE_DRIVE_DISTANCE,
    MOTION_PRIMITIVE_TURN_RELATIVE,
    MOTION_PRIMITIVE_SEARCH_LINE,
    MOTION_PRIMITIVE_STOP_AT_MARKER,
    MOTION_PRIMITIVE_WAIT_VISION
} MotionPrimitiveType;

typedef struct { uint32_t timeout_ms; } FollowLineParams;
typedef struct { int32_t distance_mm; int16_t speed; uint32_t timeout_ms; } DriveDistanceParams;
typedef struct { float angle_deg; uint32_t timeout_ms; } TurnRelativeParams;
typedef struct { bool prefer_left; uint32_t timeout_ms; } SearchLineParams;
typedef struct { uint32_t timeout_ms; } StopAtMarkerParams;
typedef struct { uint8_t event_id; uint32_t timeout_ms; } WaitVisionParams;

typedef union {
    FollowLineParams follow_line;
    DriveDistanceParams drive_distance;
    TurnRelativeParams turn_relative;
    SearchLineParams search_line;
    StopAtMarkerParams stop_at_marker;
    WaitVisionParams wait_vision;
} MotionPrimitiveParams;

typedef struct {
    LineEstimate line;
    int32_t distance_mm;
    float yaw_deg;
    float yaw_rate_dps;
    uint8_t vision_event;
    bool odometry_fresh;
    bool yaw_fresh;
    bool vision_fresh;
    bool emergency_stop;
} MotionContext;

bool MotionPrimitive_Init(void);
bool MotionPrimitive_Start(MotionPrimitiveType type,
                           const MotionPrimitiveParams *params,
                           uint32_t now_ms);
MotionResult MotionPrimitive_StepWithContext(uint32_t now_ms,
                                             const MotionContext *context,
                                             MotionRequest *request);
void MotionPrimitive_Cancel(void);

#endif
