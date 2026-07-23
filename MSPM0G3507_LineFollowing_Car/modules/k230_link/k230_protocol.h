#ifndef K230_PROTOCOL_H
#define K230_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t id;
    uint8_t field_count;
    int32_t fields[6];
    char text[48];
} K230Frame;

void K230Protocol_Init(void);
void K230Protocol_ConsumeByte(uint8_t byte);
bool K230Protocol_TakeFrame(K230Frame *out);
uint32_t K230Protocol_GetRejectedCount(void);

#endif
