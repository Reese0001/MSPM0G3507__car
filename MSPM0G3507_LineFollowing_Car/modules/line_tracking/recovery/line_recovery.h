#ifndef LINE_RECOVERY_H
#define LINE_RECOVERY_H

#include <stdbool.h>
#include <stdint.h>

#include "../../../shared/motion_request.h"
#include "../line_model.h"

typedef enum {
    LINE_RECOVERY_FOLLOW = 0,
    LINE_RECOVERY_LOSS_CONFIRM,
    LINE_RECOVERY_FORWARD_SEARCH,
    LINE_RECOVERY_ROTATION_PAUSE,
    LINE_RECOVERY_ROTATE_SEARCH,
    LINE_RECOVERY_ALIGN,
    /* Exhausted search: motors stop, but trustworthy line frames
     * resume control; ordinary loss never latches a permanent fault. */
    LINE_RECOVERY_STOPPED
} LineRecoveryState;

void LineRecovery_Init(void);
void LineRecovery_Reset(void);
LineRecoveryState LineRecovery_GetState(void);
bool LineRecovery_Step(const LineEstimate *line,
                       const LineTrendResult *trend,
                       const LineControlOutput *follow,
                       float yaw_deg,
                       bool yaw_fresh,
                       bool emergency_stop,
                       uint32_t now_ms,
                       MotionRequest *request);

#endif
