#include <assert.h>

#include "modules/wifi/wifi_at_probe.h"

int main(void)
{
    WifiAtProbe probe;

    WifiAtProbe_Init(&probe, 100U);
    assert(WifiAtProbe_GetState(&probe) == WIFI_AT_PROBE_BOOT_WAIT);
    assert(!WifiAtProbe_TakeSendRequest(&probe, 599U));
    assert(WifiAtProbe_TakeSendRequest(&probe, 600U));
    WifiAtProbe_Feed(&probe, 'O');
    WifiAtProbe_Feed(&probe, 'K');
    assert(WifiAtProbe_GetState(&probe) == WIFI_AT_PROBE_OK);

    WifiAtProbe_Init(&probe, 0U);
    assert(WifiAtProbe_TakeSendRequest(&probe, 500U));
    WifiAtProbe_Service(&probe, 1999U);
    assert(WifiAtProbe_GetState(&probe) == WIFI_AT_PROBE_WAITING_FOR_OK);
    WifiAtProbe_Service(&probe, 2000U);
    assert(WifiAtProbe_GetState(&probe) == WIFI_AT_PROBE_TIMEOUT);

    return 0;
}
