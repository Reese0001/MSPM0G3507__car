#include "app_mailbox.h"

static AppLineSample line_sample;
static bool line_sample_valid;
static Mpu6050Snapshot imu_snapshot;
static bool imu_snapshot_valid;
static MotionRequest motion_request;
static bool motion_request_valid;

void AppMailbox_Init(void)
{
    line_sample_valid = false;
    imu_snapshot_valid = false;
    motion_request_valid = false;
}

void AppMailbox_PublishLineSample(const AppLineSample *sample)
{
    if (sample == 0) {
        return;
    }
    line_sample = *sample;
    line_sample_valid = true;
}

bool AppMailbox_ReadLineSample(AppLineSample *out)
{
    bool valid;

    if (out == 0) {
        return false;
    }
    *out = line_sample;
    valid = line_sample_valid;
    return valid;
}

void AppMailbox_PublishImu(const Mpu6050Snapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }
    imu_snapshot = *snapshot;
    imu_snapshot_valid = true;
}

bool AppMailbox_ReadImu(Mpu6050Snapshot *out)
{
    bool valid;

    if (out == 0) {
        return false;
    }
    *out = imu_snapshot;
    valid = imu_snapshot_valid;
    return valid;
}

void AppMailbox_PublishMotionRequest(const MotionRequest *request)
{
    if (request == 0) {
        return;
    }
    motion_request = *request;
    motion_request_valid = true;
}

bool AppMailbox_ReadMotionRequest(MotionRequest *out)
{
    bool valid;

    if (out == 0) {
        return false;
    }
    *out = motion_request;
    valid = motion_request_valid;
    return valid;
}
