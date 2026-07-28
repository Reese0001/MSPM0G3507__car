#ifndef RUNTIME_LOG_H
#define RUNTIME_LOG_H

#include <stdbool.h>
#include <stdint.h>

#define RUNTIME_LOG_CAPACITY 8U
#define RUNTIME_LOG_LINE_CHARS 21U
#define RUNTIME_LOG_LINE_BUFFER (RUNTIME_LOG_LINE_CHARS + 1U)

void RuntimeLog_Init(void);
bool RuntimeLog_Push(uint32_t now_ms, const char *event);
bool RuntimeLog_PushMotor(uint32_t now_ms, int16_t left, int16_t right);
bool RuntimeLog_PushTaskMask(uint32_t now_ms, uint8_t mask);
uint8_t RuntimeLog_Snapshot(
    char out[RUNTIME_LOG_CAPACITY][RUNTIME_LOG_LINE_BUFFER]);
void RuntimeLog_Draw(void);

#endif
