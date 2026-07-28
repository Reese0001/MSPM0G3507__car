#ifndef MODULE_STATUS_H
#define MODULE_STATUS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MODULE_HEALTH_UNKNOWN = 0,
    MODULE_HEALTH_OK,
    MODULE_HEALTH_DEGRADED,
    MODULE_HEALTH_FAULT
} ModuleHealth;

typedef struct {
    uint32_t timestamp_ms;
    uint16_t sequence;
    bool valid;
    ModuleHealth health;
} ModuleStatus;

static inline bool ModuleStatus_IsFresh(const ModuleStatus *status,
                                        uint32_t now_ms,
                                        uint32_t max_age_ms)
{
    return status != 0 && status->valid &&
           status->health != MODULE_HEALTH_FAULT &&
           (uint32_t)(now_ms - status->timestamp_ms) <= max_age_ms;
}

#endif
