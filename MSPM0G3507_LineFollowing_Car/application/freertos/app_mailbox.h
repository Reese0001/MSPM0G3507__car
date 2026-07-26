#ifndef APPLICATION_FREERTOS_APP_MAILBOX_H
#define APPLICATION_FREERTOS_APP_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

#include "../../modules/common/motion_request.h"
#include "../../modules/line_tracking/line_position.h"
#include "../../modules/mpu6050/mpu6050.h"

/*
 * Latest-value mailboxes between the static FreeRTOS tasks. Each slot
 * holds exactly one fixed-size snapshot; publish overwrites, read
 * copies. All copies happen inside short critical sections so a
 * higher-priority reader can never observe a torn struct.
 */

typedef struct {
    LinePositionResult position;
    uint16_t sequence;
    uint32_t timestamp_ms;
} AppLineSample;

void AppMailbox_Init(void);
void AppMailbox_PublishLineSample(const AppLineSample *sample);
bool AppMailbox_ReadLineSample(AppLineSample *out);
void AppMailbox_PublishImu(const Mpu6050Snapshot *snapshot);
bool AppMailbox_ReadImu(Mpu6050Snapshot *out);
void AppMailbox_PublishMotionRequest(const MotionRequest *request);
bool AppMailbox_ReadMotionRequest(MotionRequest *out);

#endif
