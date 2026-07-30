#include "wifi_at_probe.h"

#define WIFI_AT_PROBE_BOOT_WAIT_MS (500U)
#define WIFI_AT_PROBE_RESPONSE_TIMEOUT_MS (1500U)

static bool time_reached(uint32_t now_ms, uint32_t target_ms)
{
    return (int32_t)(now_ms - target_ms) >= 0;
}

void WifiAtProbe_Init(WifiAtProbe *probe, uint32_t now_ms)
{
    probe->state = WIFI_AT_PROBE_BOOT_WAIT;
    probe->boot_ready_ms = now_ms + WIFI_AT_PROBE_BOOT_WAIT_MS;
    probe->deadline_ms = 0U;
    probe->ok_match_count = 0U;
}

bool WifiAtProbe_TakeSendRequest(WifiAtProbe *probe, uint32_t now_ms)
{
    if ((probe->state != WIFI_AT_PROBE_BOOT_WAIT) ||
        !time_reached(now_ms, probe->boot_ready_ms)) {
        return false;
    }

    probe->state = WIFI_AT_PROBE_WAITING_FOR_OK;
    probe->deadline_ms = now_ms + WIFI_AT_PROBE_RESPONSE_TIMEOUT_MS;
    probe->ok_match_count = 0U;
    return true;
}

void WifiAtProbe_Feed(WifiAtProbe *probe, uint8_t byte)
{
    if (probe->state != WIFI_AT_PROBE_WAITING_FOR_OK) {
        return;
    }

    if ((probe->ok_match_count == 0U) && (byte == (uint8_t)'O')) {
        probe->ok_match_count = 1U;
    } else if ((probe->ok_match_count == 1U) && (byte == (uint8_t)'K')) {
        probe->state = WIFI_AT_PROBE_OK;
        probe->ok_match_count = 0U;
    } else {
        probe->ok_match_count = (byte == (uint8_t)'O') ? 1U : 0U;
    }
}

void WifiAtProbe_Service(WifiAtProbe *probe, uint32_t now_ms)
{
    if ((probe->state == WIFI_AT_PROBE_WAITING_FOR_OK) &&
        time_reached(now_ms, probe->deadline_ms)) {
        probe->state = WIFI_AT_PROBE_TIMEOUT;
    }
}

WifiAtProbeState WifiAtProbe_GetState(const WifiAtProbe *probe)
{
    return probe->state;
}
