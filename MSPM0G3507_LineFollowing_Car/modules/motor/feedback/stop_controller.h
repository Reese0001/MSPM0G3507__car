#ifndef STOP_CONTROLLER_H
#define STOP_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool active;
    bool done;
    uint32_t target_distance_mm;
} StopController;

typedef struct {
    bool active;
    bool done;
    float speed_command_mm_s;
    uint32_t remaining_mm;
} StopControllerStatus;

void StopController_Init(StopController *controller);
void StopController_Start(StopController *controller,
                          uint32_t target_distance_mm,
                          uint32_t now_ms);
void StopController_Update(StopController *controller,
                           uint32_t distance_mm,
                           float actual_speed_mm_s,
                           uint32_t now_ms,
                           StopControllerStatus *status);

#endif
