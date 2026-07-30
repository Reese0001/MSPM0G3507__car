#ifndef DISPLAY_DASHBOARD_H
#define DISPLAY_DASHBOARD_H

#include <stdbool.h>

#include "../../shared/motion_request.h"
#include "../line_tracking/line_follower.h"
#include "../line_tracking/lap_tracker.h"
#include "../motor/drive.h"
#include "../motor/feedback/motor_feedback.h"
#include "../motor/feedback/stop_controller.h"

typedef struct {
    bool run_started;
    bool motor_ready;
    uint8_t raw_x_bits;
    MotionRequest request;
    LineFollowerStatus line;
    LapStatus lap;
    DriveStatus drive;
    MotorFeedbackSnapshot feedback;
    StopControllerStatus stop;
    float distance_mm;
} AppDiagnostics;

void Dashboard_Render(const AppDiagnostics *data);

#endif
