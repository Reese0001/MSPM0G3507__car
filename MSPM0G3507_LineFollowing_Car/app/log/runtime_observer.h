#ifndef APP_LOG_RUNTIME_OBSERVER_H
#define APP_LOG_RUNTIME_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool safety_loop_seen;
    bool sensor_frame_seen;
    bool control_request_seen;
} RuntimeObserverInputs;

void RuntimeObserver_Init(bool display_ready);
bool RuntimeObserver_Update(uint32_t now_ms,
                            const RuntimeObserverInputs *inputs);

#endif
