#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "bsp/bsp_i2c.h"
#include "modules/mpu6050/mpu6050.h"

static BSP_I2C_Status fake_status;
static uint8_t fake_register;
static uint8_t *fake_read_buffer;
static uint8_t fake_read_length;
static int16_t fake_gyro_z;
static bool fake_force_error;

bool BSP_I2C_Init(void)
{
    fake_status = BSP_I2C_STATUS_IDLE;
    return true;
}

bool BSP_I2C_BeginRead(uint8_t address,
                       uint8_t reg,
                       uint8_t *buffer,
                       uint8_t length)
{
    assert(address == 0x68U);
    assert(fake_status != BSP_I2C_STATUS_BUSY);
    fake_register = reg;
    fake_read_buffer = buffer;
    fake_read_length = length;
    fake_status = BSP_I2C_STATUS_BUSY;
    return true;
}

bool BSP_I2C_BeginWrite(uint8_t address,
                        uint8_t reg,
                        const uint8_t *buffer,
                        uint8_t length)
{
    (void)reg;
    assert(address == 0x68U);
    assert(buffer != 0);
    assert(length == 1U);
    assert(fake_status != BSP_I2C_STATUS_BUSY);
    fake_status = BSP_I2C_STATUS_BUSY;
    return true;
}

void BSP_I2C_Service(uint32_t now_us)
{
    (void)now_us;
}

BSP_I2C_Status BSP_I2C_GetStatus(void)
{
    if (fake_status != BSP_I2C_STATUS_BUSY) {
        return fake_status;
    }
    if (fake_force_error) {
        fake_status = BSP_I2C_STATUS_ERROR;
        return fake_status;
    }
    if (fake_register == 0x75U) {
        assert(fake_read_length == 1U);
        fake_read_buffer[0] = 0x68U;
    } else if (fake_register == 0x47U) {
        uint16_t raw = (uint16_t)fake_gyro_z;
        assert(fake_read_length == 2U);
        fake_read_buffer[0] = (uint8_t)(raw >> 8);
        fake_read_buffer[1] = (uint8_t)raw;
    }
    fake_status = BSP_I2C_STATUS_DONE;
    return fake_status;
}

static void service_twice(uint32_t now_ms)
{
    Mpu6050_Service(now_ms);
    Mpu6050_Service(now_ms);
}

int main(void)
{
    Mpu6050Snapshot snapshot;
    float first_positive;
    uint32_t now_ms;

    fake_gyro_z = 655;
    fake_force_error = false;
    Mpu6050_Init(0U);

    for (now_ms = 0U;
         now_ms < 100U &&
         Mpu6050_GetState() == MPU6050_STATE_STARTUP;
         now_ms++) {
        service_twice(now_ms);
    }
    assert(Mpu6050_GetState() == MPU6050_STATE_CALIBRATING);

    for (now_ms = 100U; now_ms <= 2200U; now_ms += 10U) {
        service_twice(now_ms);
    }
    assert(Mpu6050_GetState() == MPU6050_STATE_READY);
    assert(Mpu6050_GetSnapshot(&snapshot));
    assert(snapshot.status.valid);
    assert(snapshot.yaw_rate_dps > -0.1f);
    assert(snapshot.yaw_rate_dps < 0.1f);

    fake_gyro_z = 6550;
    service_twice(now_ms);
    assert(Mpu6050_GetSnapshot(&snapshot));
    assert(snapshot.yaw_rate_dps > 0.0f);
    assert(snapshot.yaw_rate_dps < 90.0f);
    first_positive = snapshot.yaw_rate_dps;

    now_ms += 10U;
    fake_gyro_z = -5240;
    service_twice(now_ms);
    assert(Mpu6050_GetSnapshot(&snapshot));
    assert(snapshot.yaw_rate_dps < first_positive);
    assert(snapshot.yaw_rate_dps < 0.0f);

    fake_force_error = true;
    for (uint8_t error = 0U; error < 3U; error++) {
        now_ms += 10U;
        service_twice(now_ms);
    }
    assert(Mpu6050_GetState() == MPU6050_STATE_DEGRADED);

    fake_force_error = false;
    now_ms += 10U;
    service_twice(now_ms);
    assert(Mpu6050_GetState() == MPU6050_STATE_READY);

    fake_force_error = true;
    Mpu6050_Init(0U);
    for (now_ms = 0U; now_ms < 20U; now_ms++) {
        service_twice(now_ms);
    }
    assert(Mpu6050_GetState() == MPU6050_STATE_DEGRADED);

    fake_force_error = false;
    for (now_ms = 100U;
         now_ms < 700U &&
         Mpu6050_GetState() != MPU6050_STATE_CALIBRATING;
         now_ms++) {
        service_twice(now_ms);
    }
    assert(Mpu6050_GetState() == MPU6050_STATE_CALIBRATING);

    return 0;
}
