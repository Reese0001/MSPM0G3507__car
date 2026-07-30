#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#define DRIVE_HOST_TEST 1
#include "modules/motor/drive.h"

static int16_t sent[4];
static int16_t previous[4];
static unsigned int sends;
static unsigned int emergency_stops;
static int16_t max_running_delta;

static int16_t absolute_delta(int16_t a, int16_t b)
{
    int16_t delta = (int16_t)(a - b);
    return delta < 0 ? (int16_t)-delta : delta;
}

bool Motor_SendSpeedFrame(int16_t m1, int16_t m2,
                          int16_t m3, int16_t m4)
{
    int16_t left_delta = absolute_delta(m4, previous[3]);
    int16_t right_delta = absolute_delta(m2, previous[1]);

    if (left_delta > max_running_delta) max_running_delta = left_delta;
    if (right_delta > max_running_delta) max_running_delta = right_delta;
    sent[0] = m1;
    sent[1] = m2;
    sent[2] = m3;
    sent[3] = m4;
    previous[0] = m1;
    previous[1] = m2;
    previous[2] = m3;
    previous[3] = m4;
    sends++;
    return true;
}

bool Motor_EmergencyStop_FromISR(void)
{
    sent[0] = sent[1] = sent[2] = sent[3] = 0;
    emergency_stops++;
    return true;
}

int main(void)
{
    MotionRequest request = {140, 100, 0U, true};
    DriveStatus status;
    uint32_t now;

    Drive_Init();
    assert(emergency_stops == 1U);
    assert(sent[0] == 0 && sent[1] == 0 && sent[2] == 0 && sent[3] == 0);
    Drive_SetTarget(&request);
    Drive_Service(10U);
    Drive_GetStatus(&status);
    assert(!status.started && status.left_applied == 0);

    Drive_Start();
    for (now = 1U; now <= 1000U; now++) {
        request.timestamp_ms = now;
        Drive_SetTarget(&request);
        Drive_Tick1ms();
        Drive_Service(now);
    }
    Drive_GetStatus(&status);
    assert(status.started && !status.fault);
    assert(status.left_applied <= 135 && status.right_applied <= 135);

    for (now = 1001U; now <= 2000U; now++) {
        request.timestamp_ms = now;
        Drive_SetTarget(&request);
        Drive_Tick1ms();
        Drive_Service(now);
    }
    Drive_GetStatus(&status);
    assert(status.left_applied >= 139 && status.right_applied == 100);
    assert(sent[0] == 0 && sent[1] == 100 && sent[2] == 0 && sent[3] >= 139);

    max_running_delta = 0;
    request.left_speed = 20;
    request.right_speed = 140;
    for (now = 2001U; now <= 2100U; now++) {
        request.timestamp_ms = now;
        Drive_SetTarget(&request);
        Drive_Tick1ms();
        Drive_Service(now);
    }
    Drive_GetStatus(&status);
    assert(max_running_delta <= 3);
    assert(status.left_applied < 140 && status.right_applied > 100);

    Drive_Service(2151U);
    Drive_GetStatus(&status);
    assert(status.left_applied == 0 && status.right_applied == 0);
    assert(sent[0] == 0 && sent[1] == 0 && sent[2] == 0 && sent[3] == 0);
    assert(sends > 0U);
    return 0;
}
