#include "run_controller.h"

static bool run_requested;

void RunController_Init(void)
{
    run_requested = false;
}

void RunController_OnKeyEvent(KeyEvent event)
{
    if (event == KEY_EVENT_PRESS) {
        run_requested = true;
    }
}

bool RunController_IsRunRequested(void)
{
    return run_requested;
}
