#ifndef WIFI_UART_H
#define WIFI_UART_H

#include <stdint.h>

#include "ti_msp_dl_config.h"
#include "wifi_at_probe.h"

void WifiUart_Init(uint32_t now_ms);
void WifiUart_Service(uint32_t now_ms);
WifiAtProbeState WifiUart_GetProbeState(void);
void Wifi_INST_IRQHandler(void);

#endif
