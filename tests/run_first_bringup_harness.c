#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "app/mailbox/app_mailbox.h"
#include "app/safety/safety_runtime.h"
#include "modules/key/key.h"
#include "modules/motor/safety/motor_safety.h"

static int16_t last_motor1;
static int16_t last_motor2;
static int16_t last_motor3;
static int16_t last_motor4;
static uint32_t nonzero_frame_ms = 0xFFFFFFFFU;

#define CHECK(condition)                                                     \
    do {                                                                     \
        if (!(condition)) {                                                   \
            fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__,         \
                    __LINE__, #condition);                                   \
            exit(1);                                                          \
        }                                                                    \
    } while (0)

bool AppBoot_IsMotorConfigured(void)
{
    return true;
}

bool BootTrace_MotionTasksOnline(void)
{
    return true;
}

bool BootTrace_AllTasksOnline(void)
{
    return true;
}

void LED_ON(void)
{
}

void LED_OFF(void)
{
}

void LED_HeartbeatService(uint32_t now_ms)
{
    (void)now_ms;
}

KeyEvent Key_PollEvent(void)
{
    return KEY_EVENT_NONE;
}

uint32_t Motor_Safety_Host_EnterCritical(void)
{
    return 0U;
}

void Motor_Safety_Host_ExitCritical(uint32_t previous_state)
{
    (void)previous_state;
}

bool Motor_SendSpeedFrame(int16_t motor1, int16_t motor2,
                          int16_t motor3, int16_t motor4)
{
    last_motor1 = motor1;
    last_motor2 = motor2;
    last_motor3 = motor3;
    last_motor4 = motor4;
    if ((motor2 != 0 || motor4 != 0) && nonzero_frame_ms == 0xFFFFFFFFU) {
        nonzero_frame_ms = 1U;
    }
    return true;
}

bool Motor_EmergencyStop_FromISR(void)
{
    last_motor1 = 0;
    last_motor2 = 0;
    last_motor3 = 0;
    last_motor4 = 0;
    return true;
}

int main(void)
{
    uint32_t now_ms;
    MotorSafetyDiagnostics diagnostics;

    AppMailbox_Init();
    Motor_Safety_Init();
    SafetyRuntime_Init(0U);

    for (now_ms = 0U; now_ms <= 350U; ++now_ms) {
        SafetyRuntime_Step(now_ms);
        Motor_Safety_Tick1ms();
        Motor_Safety_GetDiagnostics(&diagnostics);
        if ((diagnostics.left_applied != 0 ||
             diagnostics.right_applied != 0) &&
            nonzero_frame_ms == 1U) {
            nonzero_frame_ms = now_ms;
            break;
        }
    }

    Motor_Safety_GetDiagnostics(&diagnostics);
    CHECK(diagnostics.armed);
    CHECK(diagnostics.fault_reason == MOTOR_SAFETY_FAULT_NONE);
    CHECK(nonzero_frame_ms != 0xFFFFFFFFU);
    CHECK(nonzero_frame_ms <= 350U);
    CHECK(last_motor1 == 0);
    CHECK(last_motor3 == 0);
    CHECK(last_motor2 > 0);
    CHECK(last_motor4 > 0);

    return 0;
}
