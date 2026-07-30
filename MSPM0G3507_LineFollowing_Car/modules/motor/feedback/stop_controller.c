#include "stop_controller.h"

#define STOP_DECEL_MM_S2 (1200.0f)
#define STOP_MARGIN_MM (15.0f)

void StopController_Init(StopController *controller)
{
    if (controller != 0) {
        *controller = (StopController){0};
    }
}

void StopController_Start(StopController *controller,
                          uint32_t target_distance_mm,
                          uint32_t now_ms)
{
    (void)now_ms;
    if (controller != 0) {
        controller->active = true;
        controller->done = false;
        controller->target_distance_mm = target_distance_mm;
    }
}

void StopController_Update(StopController *controller,
                           uint32_t distance_mm,
                           float actual_speed_mm_s,
                           uint32_t now_ms,
                           StopControllerStatus *status)
{
    float remaining;
    float brake_distance;
    float command;

    (void)now_ms;
    if (status == 0) {
        return;
    }
    *status = (StopControllerStatus){0};
    if (controller == 0 || !controller->active || controller->done) {
        status->done = controller != 0 && controller->done;
        return;
    }
    if (distance_mm >= controller->target_distance_mm) {
        controller->done = true;
        controller->active = false;
        status->done = true;
        return;
    }

    remaining = (float)(controller->target_distance_mm - distance_mm);
    brake_distance = actual_speed_mm_s * actual_speed_mm_s /
                     (2.0f * STOP_DECEL_MM_S2) + STOP_MARGIN_MM;
    if (brake_distance < STOP_MARGIN_MM) {
        brake_distance = STOP_MARGIN_MM;
    }
    command = actual_speed_mm_s;
    if (remaining < brake_distance) {
        command *= remaining / brake_distance;
    }
    if (command < 0.0f) {
        command = 0.0f;
    }
    status->active = true;
    status->speed_command_mm_s = command;
    status->remaining_mm = (uint32_t)remaining;
}
