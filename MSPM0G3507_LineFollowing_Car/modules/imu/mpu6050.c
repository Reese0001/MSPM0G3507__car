#include "mpu6050.h"

#include "../../bsp/bsp_i2c.h"
#include "yaw_estimator.h"

#define MPU6050_ADDRESS (0x68U)
#define MPU6050_PWR_MGMT_1 (0x6BU)
#define MPU6050_GYRO_XOUT_H (0x43U)
#define MPU6050_GYRO_ZOUT_H (0x47U)
#define MPU6050_GYRO_SCALE_RAD_S \
    ((2000.0f / 32768.0f) * (3.1415926f / 180.0f))

typedef enum {
    MPU6050_INIT_START = 0,
    MPU6050_INIT_WAIT,
    MPU6050_READ_START,
    MPU6050_READ_WAIT
} Mpu6050State;

static Mpu6050Snapshot published;
static uint8_t transfer[6];
static uint32_t last_sample_ms;
static Mpu6050State state;
static uint32_t calibration_end_ms;
static uint32_t last_yaw_sample_ms;
static YawEstimator yaw_estimator;

static int16_t decode_i16_be(const uint8_t bytes[2])
{
    return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static void mark_error(void)
{
    published.status.valid = false;
    published.status.health = MODULE_HEALTH_DEGRADED;
    state = MPU6050_INIT_START;
}

void Mpu6050_Init(uint32_t now_ms)
{
    published = (Mpu6050Snapshot){0};
    published.status.timestamp_ms = now_ms;
    published.status.health = BSP_I2C_Init() ? MODULE_HEALTH_UNKNOWN :
                                                MODULE_HEALTH_DEGRADED;
    last_sample_ms = now_ms;
    calibration_end_ms = now_ms + 1000U;
    last_yaw_sample_ms = now_ms;
    YawEstimator_Init(&yaw_estimator);
    state = MPU6050_INIT_START;
}

void Mpu6050_Service(uint32_t now_ms)
{
    BSP_I2C_Status i2c_status;
    static const uint8_t wake = 0U;

    if (state == MPU6050_INIT_START) {
        if (BSP_I2C_BeginWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1,
                               &wake, 1U)) {
            state = MPU6050_INIT_WAIT;
        }
        return;
    }
    if (state == MPU6050_INIT_WAIT) {
        i2c_status = BSP_I2C_GetStatus();
        if (i2c_status == BSP_I2C_STATUS_BUSY) {
            return;
        }
        if (i2c_status != BSP_I2C_STATUS_DONE) {
            mark_error();
            return;
        }
        last_sample_ms = now_ms - MPU6050_SAMPLE_PERIOD_MS;
        state = MPU6050_READ_START;
        return;
    }
    if (state == MPU6050_READ_START) {
        if ((uint32_t)(now_ms - last_sample_ms) < MPU6050_SAMPLE_PERIOD_MS) {
            return;
        }
        if (BSP_I2C_BeginRead(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H,
                              transfer, 6U)) {
            state = MPU6050_READ_WAIT;
        }
        return;
    }

    i2c_status = BSP_I2C_GetStatus();
    if (i2c_status == BSP_I2C_STATUS_BUSY) {
        return;
    }
    if (i2c_status != BSP_I2C_STATUS_DONE) {
        mark_error();
        return;
    }
    published.gyro_rad_s[0] = (float)decode_i16_be(&transfer[0]) *
                              MPU6050_GYRO_SCALE_RAD_S;
    published.gyro_rad_s[1] = (float)decode_i16_be(&transfer[2]) *
                              MPU6050_GYRO_SCALE_RAD_S *
                              MPU6050_GYRO_Y_SIGN;
    published.gyro_rad_s[2] = (float)decode_i16_be(&transfer[4]) *
                              MPU6050_GYRO_SCALE_RAD_S *
                              MPU6050_GYRO_Z_SIGN;
    if (!yaw_estimator.calibrated && now_ms < calibration_end_ms) {
        YawEstimator_CalibrateSample(&yaw_estimator,
                                     published.gyro_rad_s[1],
                                     MPU6050_SAMPLE_PERIOD_MS / 1000.0f);
    } else {
        if (!yaw_estimator.calibrated) {
            YawEstimator_FinishCalibration(&yaw_estimator);
        }
        YawEstimator_Update(&yaw_estimator, published.gyro_rad_s[1], true,
                            (float)(now_ms - last_yaw_sample_ms) / 1000.0f);
    }
    published.yaw_angle_deg = yaw_estimator.yaw_angle_deg;
    last_yaw_sample_ms = now_ms;
    published.status.timestamp_ms = now_ms;
    published.status.sequence++;
    published.status.valid = true;
    published.status.health = MODULE_HEALTH_OK;
    last_sample_ms = now_ms;
    state = MPU6050_READ_START;
}

bool Mpu6050_GetSnapshot(Mpu6050Snapshot *out)
{
    if (out == 0) {
        return false;
    }
    *out = published;
    return published.status.valid &&
           published.status.health == MODULE_HEALTH_OK;
}

void Mpu6050_ResetYawReference(void)
{
    YawEstimator_Reset(&yaw_estimator);
    published.yaw_angle_deg = 0.0f;
}
