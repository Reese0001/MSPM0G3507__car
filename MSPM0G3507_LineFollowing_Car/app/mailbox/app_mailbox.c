#include "app_mailbox.h"

#include "FreeRTOS.h"
#include "task.h"

static AppLineSample line_sample;
static bool line_sample_valid;
static Mpu6050Snapshot imu_snapshot;
static bool imu_snapshot_valid;
static MotionRequest motion_request;
static bool motion_request_valid;

void AppMailbox_Init(void)
{
    taskENTER_CRITICAL();
    line_sample_valid = false;
    imu_snapshot_valid = false;
    motion_request_valid = false;
    taskEXIT_CRITICAL();
}

void AppMailbox_PublishLineSample(const AppLineSample *sample)
{
    if (sample == 0) {
        return;
    }
    taskENTER_CRITICAL();
    line_sample = *sample;
    line_sample_valid = true;
    taskEXIT_CRITICAL();
}

bool AppMailbox_ReadLineSample(AppLineSample *out)
{
    bool valid;

    if (out == 0) {
        return false;
    }
    taskENTER_CRITICAL();
    *out = line_sample;
    valid = line_sample_valid;
    taskEXIT_CRITICAL();
    return valid;
}

void AppMailbox_PublishImu(const Mpu6050Snapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }
    taskENTER_CRITICAL();
    imu_snapshot = *snapshot;
    imu_snapshot_valid = true;
    taskEXIT_CRITICAL();
}

bool AppMailbox_ReadImu(Mpu6050Snapshot *out)
{
    bool valid;

    if (out == 0) {
        return false;
    }
    taskENTER_CRITICAL();
    *out = imu_snapshot;
    valid = imu_snapshot_valid;
    taskEXIT_CRITICAL();
    return valid;
}

void AppMailbox_PublishMotionRequest(const MotionRequest *request)
{
    if (request == 0) {
        return;
    }
    taskENTER_CRITICAL();
    motion_request = *request;
    motion_request_valid = true;
    taskEXIT_CRITICAL();
}

bool AppMailbox_ReadMotionRequest(MotionRequest *out)
{
    bool valid;

    if (out == 0) {
        return false;
    }
    taskENTER_CRITICAL();
    *out = motion_request;
    valid = motion_request_valid;
    taskEXIT_CRITICAL();
    return valid;
}
