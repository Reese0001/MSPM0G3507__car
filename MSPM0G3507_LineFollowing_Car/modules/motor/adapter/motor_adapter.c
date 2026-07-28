#include "motor_adapter.h"

#include "../safety/motor_safety.h"

#define MOTOR_ADAPTER_MAX_COMMAND (450)

static bool command_is_safe(int16_t command)
{
    int32_t value = command;
    if (value < 0) {
        value = -value;
    }
    return value <= MOTOR_ADAPTER_MAX_COMMAND;
}

void MotorAdapter_Apply(const SafetyDecision *decision)
{
    if (decision == 0 || !decision->approved ||
        !command_is_safe(decision->left_speed) ||
        !command_is_safe(decision->right_speed)) {
        Motor_Safety_RequestSpeed(0, 0, 0, 0);
        return;
    }
    Motor_Safety_RequestSpeed(0, decision->left_speed,
                              0, decision->right_speed);
}
