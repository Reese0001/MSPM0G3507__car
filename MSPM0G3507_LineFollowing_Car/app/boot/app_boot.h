#ifndef APP_BOOT_H
#define APP_BOOT_H

#include <stdbool.h>

void AppBoot_Init(void);
bool AppBoot_IsMotorConfigured(void);
bool AppBoot_IsDisplayReady(void);

#endif
