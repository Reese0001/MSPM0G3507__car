#ifndef WIFI_AT_PROBE_H
#define WIFI_AT_PROBE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    WIFI_AT_PROBE_BOOT_WAIT = 0,
    WIFI_AT_PROBE_WAITING_FOR_OK,
    WIFI_AT_PROBE_OK,
    WIFI_AT_PROBE_TIMEOUT
} WifiAtProbeState;

typedef struct {
    WifiAtProbeState state;
    uint32_t boot_ready_ms;
    uint32_t deadline_ms;
    uint8_t ok_match_count;
} WifiAtProbe;

void WifiAtProbe_Init(WifiAtProbe *probe, uint32_t now_ms);
bool WifiAtProbe_TakeSendRequest(WifiAtProbe *probe, uint32_t now_ms);
void WifiAtProbe_Feed(WifiAtProbe *probe, uint8_t byte);
void WifiAtProbe_Service(WifiAtProbe *probe, uint32_t now_ms);
WifiAtProbeState WifiAtProbe_GetState(const WifiAtProbe *probe);

#endif
