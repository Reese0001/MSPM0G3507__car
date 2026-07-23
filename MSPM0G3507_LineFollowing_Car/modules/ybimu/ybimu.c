#include "ybimu.h"

#include "bsp_i2c.h"
#include "ybimu_config.h"
#include "ybimu_protocol.h"

enum {
    YBIMU_READ_GYRO = 0,
    YBIMU_READ_MAG,
    YBIMU_READ_QUAT,
    YBIMU_READ_EULER,
    YBIMU_READ_COUNT
};

static const uint8_t registers[] = {
    YBIMU_REG_GYRO,
    YBIMU_REG_MAG,
    YBIMU_REG_QUAT,
    YBIMU_REG_EULER
};

static const uint8_t register_lengths[] = {6U, 6U, 16U, 12U};

static YbImuSnapshot published = {0};
static YbImuSnapshot working = {0};
static uint32_t last_group_start_ms = 0U;
static uint8_t read_index = 0U;
static uint8_t consecutive_errors = 0U;
static bool group_active = false;

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t start_ms)
{
    return (uint32_t)(now_ms - start_ms);
}

static void map_vector(const float sensor[3], float body[3])
{
    body[0] = sensor[YBIMU_BODY_X_SOURCE] * YBIMU_BODY_X_SIGN;
    body[1] = sensor[YBIMU_BODY_Y_SOURCE] * YBIMU_BODY_Y_SIGN;
    body[2] = sensor[YBIMU_BODY_Z_SOURCE] * YBIMU_BODY_Z_SIGN;
}

static void decode_current_register(const uint8_t bytes[16])
{
    float sensor[3];

    if (read_index == YBIMU_READ_GYRO) {
        sensor[0] = (float)YbImuProtocol_DecodeI16LE(&bytes[0]) *
                    YBIMU_GYRO_SCALE_RAD_S;
        sensor[1] = (float)YbImuProtocol_DecodeI16LE(&bytes[2]) *
                    YBIMU_GYRO_SCALE_RAD_S;
        sensor[2] = (float)YbImuProtocol_DecodeI16LE(&bytes[4]) *
                    YBIMU_GYRO_SCALE_RAD_S;
        map_vector(sensor, working.gyro_rad_s);
    } else if (read_index == YBIMU_READ_MAG) {
        sensor[0] = (float)YbImuProtocol_DecodeI16LE(&bytes[0]) *
                    YBIMU_MAG_SCALE_UT;
        sensor[1] = (float)YbImuProtocol_DecodeI16LE(&bytes[2]) *
                    YBIMU_MAG_SCALE_UT;
        sensor[2] = (float)YbImuProtocol_DecodeI16LE(&bytes[4]) *
                    YBIMU_MAG_SCALE_UT;
        map_vector(sensor, working.mag_uT);
    } else if (read_index == YBIMU_READ_QUAT) {
        working.quat[0] = YbImuProtocol_DecodeFloatLE(&bytes[0]);
        working.quat[1] = YbImuProtocol_DecodeFloatLE(&bytes[4]);
        working.quat[2] = YbImuProtocol_DecodeFloatLE(&bytes[8]);
        working.quat[3] = YbImuProtocol_DecodeFloatLE(&bytes[12]);
    } else {
        sensor[0] = YbImuProtocol_DecodeFloatLE(&bytes[0]) * YBIMU_RAD_TO_DEG;
        sensor[1] = YbImuProtocol_DecodeFloatLE(&bytes[4]) * YBIMU_RAD_TO_DEG;
        sensor[2] = YbImuProtocol_DecodeFloatLE(&bytes[8]) * YBIMU_RAD_TO_DEG;
        map_vector(sensor, working.euler_deg);
    }
}

static void mark_group_failed(void)
{
    if (consecutive_errors < 0xFFU) {
        consecutive_errors++;
    }
    published.status.health = MODULE_HEALTH_DEGRADED;
    group_active = false;
    read_index = 0U;
}

static void publish_complete_group(uint32_t now_ms)
{
    working.status.timestamp_ms = now_ms;
    working.status.sequence = (uint16_t)(published.status.sequence + 1U);
    working.status.valid = true;
    working.status.health = MODULE_HEALTH_OK;
    working.magnetic_heading_healthy = false;
    published = working;
    consecutive_errors = 0U;
    group_active = false;
    read_index = 0U;
}

void YbImu_Init(uint32_t now_ms)
{
    bool bsp_ready = BSP_I2C_Init();

    published = (YbImuSnapshot){0};
    working = (YbImuSnapshot){0};
    published.status.timestamp_ms = now_ms;
    published.status.health = bsp_ready ? MODULE_HEALTH_UNKNOWN :
                                          MODULE_HEALTH_DEGRADED;
    last_group_start_ms = now_ms - YBIMU_SAMPLE_PERIOD_MS;
    read_index = 0U;
    consecutive_errors = 0U;
    group_active = false;
}

void YbImu_Service(uint32_t now_ms)
{
    uint8_t bytes[BSP_I2C_MAX_TRANSFER] = {0};

    if (published.status.valid &&
        elapsed_ms(now_ms, published.status.timestamp_ms) >
            YBIMU_STALE_TIMEOUT_MS) {
        published.status.health = MODULE_HEALTH_DEGRADED;
    }

    if (!group_active) {
        if (elapsed_ms(now_ms, last_group_start_ms) < YBIMU_SAMPLE_PERIOD_MS) {
            return;
        }
        last_group_start_ms = now_ms;
        working = published;
        working.status.valid = false;
        group_active = true;
        read_index = 0U;
    }

    if (!BSP_I2C_Read(YBIMU_I2C_ADDRESS,
                      registers[read_index],
                      bytes,
                      register_lengths[read_index])) {
        mark_group_failed();
        return;
    }

    decode_current_register(bytes);
    read_index++;
    if (read_index >= YBIMU_READ_COUNT) {
        publish_complete_group(now_ms);
    }
}

bool YbImu_GetSnapshot(YbImuSnapshot *out)
{
    if (out == 0) {
        return false;
    }

    *out = published;
    return true;
}
