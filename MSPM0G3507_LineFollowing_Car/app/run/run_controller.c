#include "run_controller.h"

#define RUN_CONTROLLER_BRINGUP_SPEED (120)

static bool run_requested;

void RunController_Init(void)
{
    run_requested = true;
}

void RunController_OnKeyEvent(KeyEvent event)
{
    if (event == KEY_EVENT_SHORT) {
        run_requested = true;
    }
}

bool RunController_BuildRequest(uint32_t now_ms, MotionRequest *request)
{
    if (request == 0 || !run_requested) {
        return false;
    }
    request->left_speed = RUN_CONTROLLER_BRINGUP_SPEED;
    request->right_speed = RUN_CONTROLLER_BRINGUP_SPEED;
    request->timestamp_ms = now_ms;
    request->valid = true;
    return true;
}
