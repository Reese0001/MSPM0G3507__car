#include <assert.h>

#include "modules/motor/feedback/stop_controller.h"

int main(void)
{
    StopController controller;
    StopControllerStatus status;

    StopController_Init(&controller);
    StopController_Start(&controller, 1000U, 0U);
    StopController_Update(&controller, 970U, 300.0f, 100U, &status);
    assert(status.speed_command_mm_s > 0.0f);
    assert(status.speed_command_mm_s < 300.0f);
    StopController_Update(&controller, 1000U, 80.0f, 200U, &status);
    assert(status.done);
    assert(status.speed_command_mm_s == 0.0f);
    return 0;
}
