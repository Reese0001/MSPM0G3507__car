#include "ybimu_protocol.h"

#include <string.h>

_Static_assert(sizeof(float) == 4U, "YbImu protocol requires 32-bit float");

int16_t YbImuProtocol_DecodeI16LE(const uint8_t bytes[2])
{
    uint16_t raw;
    int16_t value;

    if (bytes == 0) {
        return 0;
    }

    raw = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    memcpy(&value, &raw, sizeof(value));
    return value;
}

float YbImuProtocol_DecodeFloatLE(const uint8_t bytes[4])
{
    uint32_t raw;
    float value = 0.0f;

    if (bytes == 0) {
        return value;
    }

    raw = (uint32_t)bytes[0] |
          ((uint32_t)bytes[1] << 8) |
          ((uint32_t)bytes[2] << 16) |
          ((uint32_t)bytes[3] << 24);
    memcpy(&value, &raw, sizeof(value));
    return value;
}
