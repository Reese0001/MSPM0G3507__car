#ifndef LINE_RECOVERY_H
#define LINE_RECOVERY_H

#include <stdbool.h>
#include <stdint.h>

#include "../../../shared/motion_request.h"
#include "../line_model.h"

typedef enum {
    LINE_RECOVERY_FOLLOW = 0,
    LINE_RECOVERY_SEEK_LEFT,
    LINE_RECOVERY_SEEK_RIGHT,
    LINE_RECOVERY_ALIGN,
    LINE_RECOVERY_STOPPED
} LineRecoveryState;

typedef struct {
    LineRecoveryState state;
    int8_t direction;
    float yaw_delta_deg;
    bool yaw_fresh;
} LineRecoveryDiagnostics;

void LineRecovery_Init(void);
void LineRecovery_Reset(void);
LineRecoveryState LineRecovery_GetState(void);
bool LineRecovery_Step(const LineEstimate *line,
                       int8_t predicted_direction,
                       const LineControlOutput *follow,
                       float yaw_deg,
                       float yaw_rate_dps,
                       bool yaw_fresh,
                       bool emergency_stop,
                       uint32_t now_ms,
                       MotionRequest *request);
void LineRecovery_GetDiagnostics(LineRecoveryDiagnostics *out);

#endif
