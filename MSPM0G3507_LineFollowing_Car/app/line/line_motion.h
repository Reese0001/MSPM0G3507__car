#ifndef APP_LINE_LINE_MOTION_H
#define APP_LINE_LINE_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "../mailbox/app_mailbox.h"
#include "../../shared/motion_request.h"

void AppLineMotion_Init(uint32_t now_ms);
void AppLineMotion_ServiceImu(uint32_t now_ms);
bool AppLineMotion_BuildRequest(const AppLineSample *sample,
                                uint32_t now_ms,
                                MotionRequest *request);

#endif
