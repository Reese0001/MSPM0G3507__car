#ifndef APP_SENSOR_SENSOR_RUNTIME_H
#define APP_SENSOR_SENSOR_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

void SensorRuntime_Init(void);
bool SensorRuntime_Step(uint32_t now_ms);

#endif
