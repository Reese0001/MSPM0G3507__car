#ifndef LINE_RECOVERY_H
#define LINE_RECOVERY_H

#include <stdbool.h>
#include <stdint.h>

#include "../modules/common/motion_request.h"
#include "../modules/line_tracking/line_controller.h"

typedef enum {
    LINE_RECOVERY_FOLLOW = 0,
    LINE_RECOVERY_LOSS_CONFIRM,
    LINE_RECOVERY_PIVOT_LEFT,
    LINE_RECOVERY_PIVOT_RIGHT,
    LINE_RECOVERY_ALIGN,
    LINE_RECOVERY_FAULT
} LineRecoveryState;

void LineRecovery_Init(void);
void LineRecovery_Reset(void);
LineRecoveryState LineRecovery_GetState(void);
bool LineRecovery_Step(const LineEstimate *line,
                       const LineControlOutput *follow,
                       float yaw_deg,
                       bool yaw_fresh,
                       bool emergency_stop,
                       uint32_t now_ms,
                       MotionRequest *request);

#endif
