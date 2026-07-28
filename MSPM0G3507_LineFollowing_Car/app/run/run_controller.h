#ifndef APP_RUN_RUN_CONTROLLER_H
#define APP_RUN_RUN_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "../../modules/key/key.h"
#include "../../shared/motion_request.h"

void RunController_Init(void);
void RunController_OnKeyEvent(KeyEvent event);
bool RunController_BuildRequest(uint32_t now_ms, MotionRequest *request);

#endif
