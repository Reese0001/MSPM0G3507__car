#ifndef K230_CONFIG_H
#define K230_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define K230_FRAME_MAX_LEN       (128U)
#define K230_VISION_STALE_MS     (300U)
#define K230_PROTOCOL_MAX_FIELDS (6U)
#define K230_PROTOCOL_TEXT_LEN   (48U)
#define K230_VENDOR_ID_MIN       (1U)
#define K230_VENDOR_ID_MAX       (17U)

/* Protocol self-test event; enable mission IDs only after task selection. */
#define K230_ALLOWED_EVENT_ID    (16U)

static inline bool K230_Config_IsIdAllowed(uint8_t id)
{
    return id == K230_ALLOWED_EVENT_ID;
}

#endif
