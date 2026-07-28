#ifndef APP_LOG_RUNTIME_OBSERVER_H
#define APP_LOG_RUNTIME_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

void RuntimeObserver_Init(bool display_ready);
void RuntimeObserver_MarkSafetyLoop(void);
void RuntimeObserver_MarkSensorFrame(void);
void RuntimeObserver_MarkControlRequest(void);
bool RuntimeObserver_Update(uint32_t now_ms);

#endif
