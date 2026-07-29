#ifndef LINE_OFFICIAL_CONTROL_H
#define LINE_OFFICIAL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "../decoder/line_position.h"
#include "../line_model.h"

typedef struct {
    LineEstimate estimate;
    LineControlOutput follow;
    int8_t recovery_direction;
    bool imu_used;
} LineOfficialControlResult;

typedef struct {
    int8_t direction;
    float yaw_rate_dps;
    int16_t damping_command;
    bool imu_used;
} LineOfficialControlDiagnostics;

void LineOfficialControl_Init(void);
bool LineOfficialControl_Step(const LinePositionResult *position,
                              uint16_t sequence,
                              uint32_t now_ms,
                              float yaw_rate_dps,
                              bool imu_fresh,
                              LineOfficialControlResult *out);
void LineOfficialControl_GetDiagnostics(LineOfficialControlDiagnostics *out);

#endif
