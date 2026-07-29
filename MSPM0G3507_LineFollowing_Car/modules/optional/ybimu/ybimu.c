#include "ybimu.h"

#include "../../../bsp/bsp_i2c.h"
#include "ybimu_config.h"
#include "ybimu_protocol.h"

#if YBIMU_GYRO_ONLY
enum {
    YBIMU_READ_GYRO = 0,
    YBIMU_READ_COUNT
};

static const uint8_t registers[] = {YBIMU_REG_GYRO};
static const uint8_t register_lengths[] = {6U};
#else
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
#endif

static YbImuSnapshot published = {0};
static YbImuSnapshot working = {0};
static uint32_t last_group_start_ms = 0U;
static uint8_t read_index = 0U;
static uint8_t consecutive_errors = 0U;
static bool group_active = false;
static bool read_pending = false;
static uint8_t transfer_buffer[BSP_I2C_MAX_TRANSFER] = {0};
static YbImuCalibrationState calibration_state = YBIMU_CAL_IDLE;
static YbImuCalibrationType calibration_type = YBIMU_CAL_TYPE_IMU;
static uint8_t calibration_register = YBIMU_REG_CAL_IMU;
static uint32_t calibration_start_ms = 0U;
static uint32_t calibration_last_poll_ms = 0U;
static uint8_t calibration_errors = 0U;
typedef enum {
    YBIMU_CAL_TRANSFER_NONE = 0,
    YBIMU_CAL_TRANSFER_WRITE,
    YBIMU_CAL_TRANSFER_READ
} YbImuCalibrationTransfer;
static YbImuCalibrationTransfer calibration_transfer =
    YBIMU_CAL_TRANSFER_NONE;
static float previous_mag_norm_sq = 0.0f;
static bool previous_mag_valid = false;

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t start_ms)
{
    return (uint32_t)(now_ms - start_ms);
}

#if !YBIMU_GYRO_ONLY
static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}
#endif

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
#if !YBIMU_GYRO_ONLY
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
#endif
    }
}

static void mark_group_failed(void)
{
    if (consecutive_errors < 0xFFU) {
        consecutive_errors++;
    }
    published.status.health = MODULE_HEALTH_DEGRADED;
    group_active = false;
    read_pending = false;
    read_index = 0U;
}

#if !YBIMU_GYRO_ONLY
static void update_magnetic_health(void)
{
    float norm_sq = working.mag_uT[0] * working.mag_uT[0] +
                    working.mag_uT[1] * working.mag_uT[1] +
                    working.mag_uT[2] * working.mag_uT[2];
    float min_sq = YBIMU_MAG_MIN_UT * YBIMU_MAG_MIN_UT;
    float max_sq = YBIMU_MAG_MAX_UT * YBIMU_MAG_MAX_UT;
    bool plausible = norm_sq >= min_sq && norm_sq <= max_sq;
    bool stable = !previous_mag_valid ||
                  abs_float(norm_sq - previous_mag_norm_sq) <=
                      YBIMU_MAG_NORM_SQ_DELTA_MAX;
    bool calibration_allows_heading =
        calibration_state != YBIMU_CAL_RUNNING &&
        calibration_state != YBIMU_CAL_FAILED;

    working.magnetic_heading_healthy =
        plausible && stable && calibration_allows_heading;
    previous_mag_norm_sq = norm_sq;
    previous_mag_valid = true;
}
#endif

static void publish_complete_group(uint32_t now_ms)
{
#if YBIMU_GYRO_ONLY
    working.magnetic_heading_healthy = false;
#else
    update_magnetic_health();
#endif
    working.status.timestamp_ms = now_ms;
    working.status.sequence = (uint16_t)(published.status.sequence + 1U);
    working.status.valid = true;
    working.status.health = MODULE_HEALTH_OK;
    published = working;
    consecutive_errors = 0U;
    group_active = false;
    read_index = 0U;
}

static void finish_calibration(YbImuCalibrationState result, uint32_t now_ms)
{
    calibration_state = result;
    calibration_transfer = YBIMU_CAL_TRANSFER_NONE;
    calibration_errors = 0U;
    published.status.valid = false;
    published.status.health = (result == YBIMU_CAL_SUCCESS) ?
                                  MODULE_HEALTH_UNKNOWN :
                                  MODULE_HEALTH_FAULT;
    published.magnetic_heading_healthy = false;
    previous_mag_valid = false;
    last_group_start_ms = now_ms - YBIMU_SAMPLE_PERIOD_MS;
}

static void service_calibration(uint32_t now_ms)
{
    BSP_I2C_Status i2c_status;
    uint32_t timeout_ms = (calibration_type == YBIMU_CAL_TYPE_IMU) ?
                              YBIMU_CAL_IMU_TIMEOUT_MS :
                              YBIMU_CAL_MAG_TIMEOUT_MS;

    if (calibration_transfer != YBIMU_CAL_TRANSFER_NONE) {
        i2c_status = BSP_I2C_GetStatus();
        if (i2c_status == BSP_I2C_STATUS_BUSY) {
            return;
        }
        if (i2c_status != BSP_I2C_STATUS_DONE) {
            if (calibration_errors < 0xFFU) {
                calibration_errors++;
            }
            calibration_transfer = YBIMU_CAL_TRANSFER_NONE;
            if (calibration_errors >= YBIMU_MAX_CONSECUTIVE_ERRORS) {
                finish_calibration(YBIMU_CAL_FAILED, now_ms);
            }
            return;
        }
        calibration_errors = 0U;
        if (calibration_transfer == YBIMU_CAL_TRANSFER_WRITE) {
            calibration_transfer = YBIMU_CAL_TRANSFER_NONE;
            calibration_start_ms = now_ms;
            calibration_last_poll_ms = now_ms;
            return;
        }
        calibration_transfer = YBIMU_CAL_TRANSFER_NONE;
        if (transfer_buffer[0] == 1U) {
            finish_calibration(YBIMU_CAL_SUCCESS, now_ms);
        } else if (transfer_buffer[0] != 0U) {
            finish_calibration(YBIMU_CAL_FAILED, now_ms);
        }
        return;
    }

    if (elapsed_ms(now_ms, calibration_start_ms) >= timeout_ms) {
        finish_calibration(YBIMU_CAL_FAILED, now_ms);
        return;
    }
    if (elapsed_ms(now_ms, calibration_last_poll_ms) <
        YBIMU_CAL_POLL_PERIOD_MS) {
        return;
    }

    calibration_last_poll_ms = now_ms;
    if (!BSP_I2C_BeginRead(YBIMU_I2C_ADDRESS,
                           calibration_register,
                           transfer_buffer,
                           1U)) {
        if (calibration_errors < 0xFFU) {
            calibration_errors++;
        }
        if (calibration_errors >= YBIMU_MAX_CONSECUTIVE_ERRORS) {
            finish_calibration(YBIMU_CAL_FAILED, now_ms);
        }
        return;
    }
    calibration_transfer = YBIMU_CAL_TRANSFER_READ;
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
    read_pending = false;
    calibration_state = YBIMU_CAL_IDLE;
    calibration_type = YBIMU_CAL_TYPE_IMU;
    calibration_register = YBIMU_REG_CAL_IMU;
    calibration_start_ms = now_ms;
    calibration_last_poll_ms = now_ms;
    calibration_errors = 0U;
    calibration_transfer = YBIMU_CAL_TRANSFER_NONE;
    previous_mag_norm_sq = 0.0f;
    previous_mag_valid = false;
}

void YbImu_Service(uint32_t now_ms)
{
    BSP_I2C_Status i2c_status;

    if (calibration_state == YBIMU_CAL_RUNNING) {
        service_calibration(now_ms);
        return;
    }

    if (published.status.valid &&
        elapsed_ms(now_ms, published.status.timestamp_ms) >
            YBIMU_STALE_TIMEOUT_MS) {
        published.status.health = MODULE_HEALTH_DEGRADED;
    }

    if (read_pending) {
        i2c_status = BSP_I2C_GetStatus();
        if (i2c_status == BSP_I2C_STATUS_BUSY) {
            return;
        }
        read_pending = false;
        if (i2c_status != BSP_I2C_STATUS_DONE) {
            mark_group_failed();
            return;
        }
        decode_current_register(transfer_buffer);
        read_index++;
        if (read_index >= YBIMU_READ_COUNT) {
            publish_complete_group(now_ms);
            return;
        }
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

    if (!BSP_I2C_BeginRead(YBIMU_I2C_ADDRESS,
                           registers[read_index],
                           transfer_buffer,
                           register_lengths[read_index])) {
        mark_group_failed();
        return;
    }
    read_pending = true;
}

bool YbImu_GetSnapshot(YbImuSnapshot *out)
{
    if (out == 0) {
        return false;
    }

    *out = published;
    return true;
}

bool YbImu_RequestCalibration(YbImuCalibrationType type, uint32_t now_ms)
{
    uint8_t calibration_value = 0x01U;
    uint8_t reg;

    if (calibration_state == YBIMU_CAL_RUNNING ||
        (type != YBIMU_CAL_TYPE_IMU && type != YBIMU_CAL_TYPE_MAG)) {
        return false;
    }
    /*
     * Do not tear down an in-flight sensor read.  The caller can retry the
     * calibration request after the bounded I2C transaction completes.
     */
    if (BSP_I2C_GetStatus() == BSP_I2C_STATUS_BUSY) {
        return false;
    }

    reg = (type == YBIMU_CAL_TYPE_IMU) ? YBIMU_REG_CAL_IMU :
                                         YBIMU_REG_CAL_MAG;
    group_active = false;
    read_pending = false;
    read_index = 0U;
    published.status.valid = false;
    published.status.health = MODULE_HEALTH_DEGRADED;
    published.magnetic_heading_healthy = false;
    previous_mag_valid = false;

    if (!BSP_I2C_BeginWrite(YBIMU_I2C_ADDRESS,
                            reg,
                            &calibration_value,
                            1U)) {
        calibration_state = YBIMU_CAL_FAILED;
        published.status.health = MODULE_HEALTH_FAULT;
        return false;
    }

    calibration_type = type;
    calibration_register = reg;
    calibration_start_ms = now_ms;
    calibration_last_poll_ms = now_ms;
    calibration_errors = 0U;
    calibration_transfer = YBIMU_CAL_TRANSFER_WRITE;
    calibration_state = YBIMU_CAL_RUNNING;
    return true;
}

void YbImu_CancelCalibration(void)
{
    if (calibration_state == YBIMU_CAL_RUNNING) {
        calibration_state = YBIMU_CAL_FAILED;
        calibration_transfer = YBIMU_CAL_TRANSFER_NONE;
        published.status.valid = false;
        published.status.health = MODULE_HEALTH_FAULT;
        published.magnetic_heading_healthy = false;
        previous_mag_valid = false;
    }
}

YbImuCalibrationState YbImu_GetCalibrationState(void)
{
    return calibration_state;
}
