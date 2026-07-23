#ifndef	__LED_H__
#define __LED_H__

#include "ti_msp_dl_config.h"

#include <stdint.h>

void LED_Toggle(void);
void LED_ON(void);
void LED_OFF(void);
void LED2_Toggle(void);
void LED_HeartbeatInit(void);
void LED_HeartbeatService(uint32_t now_ms);

#endif
