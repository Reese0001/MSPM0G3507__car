#ifndef YBIMU_PROTOCOL_H
#define YBIMU_PROTOCOL_H

#include <stdint.h>

#define YBIMU_I2C_ADDRESS   (0x23U)
#define YBIMU_REG_GYRO      (0x0AU)
#define YBIMU_REG_MAG       (0x10U)
#define YBIMU_REG_QUAT      (0x16U)
#define YBIMU_REG_EULER     (0x26U)

int16_t YbImuProtocol_DecodeI16LE(const uint8_t bytes[2]);
float YbImuProtocol_DecodeFloatLE(const uint8_t bytes[4]);

#endif
