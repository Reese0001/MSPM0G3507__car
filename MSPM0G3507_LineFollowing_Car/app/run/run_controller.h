#ifndef APP_RUN_RUN_CONTROLLER_H
#define APP_RUN_RUN_CONTROLLER_H

#include <stdbool.h>

#include "../../modules/key/key.h"

void RunController_Init(void);
void RunController_OnKeyEvent(KeyEvent event);
bool RunController_IsRunRequested(void);

#endif
