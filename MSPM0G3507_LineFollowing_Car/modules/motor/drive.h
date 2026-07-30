#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

#include "../../shared/motion_request.h"

typedef enum {
    DRIVE_ERROR_NONE = 0,
    DRIVE_ERROR_STALE,
    DRIVE_ERROR_UART,
    DRIVE_ERROR_WATCHDOG
} DriveError;

typedef struct {
    bool started;
    bool fault;
    int16_t left_requested;
    int16_t right_requested;
    int16_t left_applied;
    int16_t right_applied;
    DriveError error;
} DriveStatus;

void Drive_Init(void);
void Drive_Start(void);
void Drive_SetTarget(const MotionRequest *request);
void Drive_Service(uint32_t now_ms);
void Drive_Tick1ms(void);
void Drive_GetStatus(DriveStatus *status);

#endif
