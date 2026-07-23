#ifndef LINE_CONTROLLER_H
#define LINE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "line_estimator.h"

typedef struct {
    int16_t forward;
    int16_t turn;
    bool valid;
} LineControlOutput;

typedef struct {
    int16_t max_forward;
    int16_t curve_forward;
    int16_t hard_curve_forward;
    int16_t wide_black_forward;
    int16_t low_confidence_forward;
    int16_t hard_turn_forward;
    int16_t hard_turn_command;
    uint8_t turn_limit_percent;
    int16_t accel_step;
    int16_t decel_step;
    int16_t turn_slew_step;
    float kp;
    float kd;
    float steering_polarity;
    float curve_error_threshold;
    float hard_curve_error_threshold;
    float high_yaw_rate_dps;
    uint8_t low_confidence;
    uint8_t medium_confidence;
    uint32_t estimate_stale_ms;
} LineControlConfig;

bool LineController_Init(const LineControlConfig *settings);
void LineController_Reset(void);
bool LineController_Step(const LineEstimate *estimate,
                         float yaw_rate_dps,
                         bool yaw_fresh,
                         uint32_t now_ms,
                         LineControlOutput *output);

#endif
