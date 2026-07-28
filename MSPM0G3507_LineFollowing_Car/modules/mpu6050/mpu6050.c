#include "mpu6050.h"

#include "../../bsp/bsp_i2c.h"
#include "mpu6050_config.h"
#include "mpu6050_kalman.h"

#define MPU6050_REG_SMPLRT_DIV (0x19U)
#define MPU6050_REG_CONFIG (0x1AU)
#define MPU6050_REG_GYRO_CONFIG (0x1BU)
#define MPU6050_REG_GYRO_ZOUT_H (0x47U)
#define MPU6050_REG_PWR_MGMT_1 (0x6BU)
#define MPU6050_REG_WHO_AM_I (0x75U)

typedef enum {
    INIT_READ_ID = 0,
    INIT_WAKE,
    INIT_FILTER,
    INIT_GYRO_RANGE,
    INIT_SAMPLE_RATE,
    INIT_COMPLETE
} InitStep;

typedef enum {
    TRANSFER_NONE = 0,
    TRANSFER_INIT_READ,
    TRANSFER_INIT_WRITE,
    TRANSFER_GYRO_READ
} TransferKind;

static Mpu6050Snapshot published;
static Mpu6050State state;
static InitStep init_step;
static TransferKind transfer_kind;
static uint8_t transfer_buffer[2];
static uint8_t consecutive_errors;
static uint32_t last_sample_ms;
static uint32_t last_processed_sample_ms;
static uint32_t calibration_started_ms;
static uint32_t retry_started_ms;
static uint16_t calibration_samples;
static float calibration_sum;
static float gyro_bias_dps;
static float filtered_yaw_rate_dps;
static float yaw_angle_deg;
static Mpu6050KalmanState yaw_kalman;
static bool initialized;
static bool calibrated;

static float absolute_value(float value)
{
    return value < 0.0f ? -value : value;
}

static float clamp_rate(float value)
{
    if (value > MPU6050_MAX_RATE_DPS) {
        return MPU6050_MAX_RATE_DPS;
    }
    if (value < -MPU6050_MAX_RATE_DPS) {
        return -MPU6050_MAX_RATE_DPS;
    }
    return value;
}

static void enter_degraded(uint32_t now_ms)
{
    state = MPU6050_STATE_DEGRADED;
    published.status.health = MODULE_HEALTH_DEGRADED;
    retry_started_ms = now_ms;
}

static void note_error(uint32_t now_ms)
{
    if (consecutive_errors < UINT8_MAX) {
        consecutive_errors++;
    }
    published.status.health = MODULE_HEALTH_DEGRADED;
    if (consecutive_errors >= MPU6050_MAX_CONSECUTIVE_ERRORS) {
        enter_degraded(now_ms);
    }
}

static void start_calibration(uint32_t now_ms)
{
    state = MPU6050_STATE_CALIBRATING;
    calibrated = false;
    calibration_started_ms = now_ms;
    calibration_samples = 0U;
    calibration_sum = 0.0f;
    gyro_bias_dps = 0.0f;
    filtered_yaw_rate_dps = 0.0f;
    yaw_angle_deg = MPU6050_ANGLE_RESET;
    Mpu6050Kalman_Reset(&yaw_kalman);
    last_sample_ms = now_ms - MPU6050_SAMPLE_PERIOD_MS;
    last_processed_sample_ms = now_ms;
    published.status.valid = false;
    published.status.health = MODULE_HEALTH_UNKNOWN;
}

static void publish_rate(float yaw_rate_dps, uint32_t now_ms)
{
    published.yaw_rate_dps = yaw_rate_dps;
    published.yaw_angle_deg = yaw_angle_deg;
    published.status.timestamp_ms = now_ms;
    published.status.sequence++;
    published.status.valid = true;
    published.status.health = MODULE_HEALTH_OK;
}

static void finish_calibration(uint32_t now_ms)
{
    gyro_bias_dps = calibration_sum / (float)calibration_samples;
    filtered_yaw_rate_dps = 0.0f;
    yaw_angle_deg = MPU6050_ANGLE_RESET;
    Mpu6050Kalman_Reset(&yaw_kalman);
    last_processed_sample_ms = now_ms;
    calibrated = true;
    state = MPU6050_STATE_READY;
    publish_rate(0.0f, now_ms);
}

static void process_gyro_sample(uint32_t now_ms)
{
    int16_t raw = (int16_t)(((uint16_t)transfer_buffer[0] << 8) |
                            (uint16_t)transfer_buffer[1]);
    float sensor_rate =
        ((float)raw / MPU6050_GYRO_LSB_PER_DPS) * MPU6050_YAW_SIGN;
    float published_rate;
    bool stationary;
    uint32_t elapsed_ms;

    if (!calibrated) {
        if (state != MPU6050_STATE_CALIBRATING) {
            state = MPU6050_STATE_CALIBRATING;
        }
        calibration_sum += sensor_rate;
        if (calibration_samples < UINT16_MAX) {
            calibration_samples++;
        }
        if ((uint32_t)(now_ms - calibration_started_ms) >=
                MPU6050_CALIBRATION_MS &&
            calibration_samples >= MPU6050_MIN_CALIBRATION_SAMPLES) {
            finish_calibration(now_ms);
        }
        return;
    }

    sensor_rate = clamp_rate(sensor_rate - gyro_bias_dps);
    stationary = absolute_value(sensor_rate) < MPU6050_DEADBAND_DPS;
    filtered_yaw_rate_dps +=
        MPU6050_FILTER_ALPHA * (sensor_rate - filtered_yaw_rate_dps);
    published_rate = stationary ? 0.0f : filtered_yaw_rate_dps;
    elapsed_ms = (uint32_t)(now_ms - last_processed_sample_ms);
    last_processed_sample_ms = now_ms;
    if (elapsed_ms == 0U) {
        elapsed_ms = MPU6050_SAMPLE_PERIOD_MS;
    }
    if (elapsed_ms > MPU6050_KALMAN_MAX_DT_MS) {
        elapsed_ms = MPU6050_KALMAN_MAX_DT_MS;
    }
    yaw_angle_deg = Mpu6050Kalman_Update(
        &yaw_kalman,
        filtered_yaw_rate_dps,
        sensor_rate,
        (float)elapsed_ms / 1000.0f,
        stationary);
    state = MPU6050_STATE_READY;
    publish_rate(published_rate, now_ms);
}

static void complete_initialization(uint32_t now_ms)
{
    initialized = true;
    init_step = INIT_COMPLETE;
    start_calibration(now_ms);
}

static void process_completed_transfer(uint32_t now_ms)
{
    consecutive_errors = 0U;
    if (transfer_kind == TRANSFER_INIT_READ) {
        if (transfer_buffer[0] != MPU6050_I2C_ADDRESS) {
            note_error(now_ms);
            return;
        }
        init_step = INIT_WAKE;
    } else if (transfer_kind == TRANSFER_INIT_WRITE) {
        init_step = (InitStep)((uint8_t)init_step + 1U);
        if (init_step == INIT_COMPLETE) {
            complete_initialization(now_ms);
        }
    } else if (transfer_kind == TRANSFER_GYRO_READ) {
        process_gyro_sample(now_ms);
    }
}

static bool service_pending_transfer(uint32_t now_ms)
{
    BSP_I2C_Status status;

    if (transfer_kind == TRANSFER_NONE) {
        return false;
    }
    status = BSP_I2C_GetStatus();
    if (status == BSP_I2C_STATUS_BUSY) {
        return true;
    }
    if (status != BSP_I2C_STATUS_DONE) {
        transfer_kind = TRANSFER_NONE;
        note_error(now_ms);
        return true;
    }
    process_completed_transfer(now_ms);
    transfer_kind = TRANSFER_NONE;
    return true;
}

static bool begin_init_transfer(void)
{
    static const uint8_t registers[] = {
        MPU6050_REG_WHO_AM_I,
        MPU6050_REG_PWR_MGMT_1,
        MPU6050_REG_CONFIG,
        MPU6050_REG_GYRO_CONFIG,
        MPU6050_REG_SMPLRT_DIV
    };
    static const uint8_t values[] = {
        0x00U,
        0x01U,
        0x03U,
        0x08U,
        0x09U
    };

    if (init_step == INIT_READ_ID) {
        if (!BSP_I2C_BeginRead(MPU6050_I2C_ADDRESS,
                               registers[init_step],
                               transfer_buffer, 1U)) {
            return false;
        }
        transfer_kind = TRANSFER_INIT_READ;
        return true;
    }
    if (init_step < INIT_COMPLETE &&
        BSP_I2C_BeginWrite(MPU6050_I2C_ADDRESS,
                           registers[init_step],
                           &values[init_step], 1U)) {
        transfer_kind = TRANSFER_INIT_WRITE;
        return true;
    }
    return false;
}

static bool begin_gyro_read(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - last_sample_ms) < MPU6050_SAMPLE_PERIOD_MS) {
        return true;
    }
    if (!BSP_I2C_BeginRead(MPU6050_I2C_ADDRESS,
                           MPU6050_REG_GYRO_ZOUT_H,
                           transfer_buffer, 2U)) {
        return false;
    }
    last_sample_ms = now_ms;
    transfer_kind = TRANSFER_GYRO_READ;
    return true;
}

void Mpu6050_Init(uint32_t now_ms)
{
    bool bus_ready = BSP_I2C_Init();

    published = (Mpu6050Snapshot){0};
    state = bus_ready ? MPU6050_STATE_STARTUP : MPU6050_STATE_DEGRADED;
    init_step = INIT_READ_ID;
    transfer_kind = TRANSFER_NONE;
    consecutive_errors = 0U;
    last_sample_ms = now_ms;
    calibration_started_ms = now_ms;
    retry_started_ms = now_ms;
    calibration_samples = 0U;
    calibration_sum = 0.0f;
    gyro_bias_dps = 0.0f;
    filtered_yaw_rate_dps = 0.0f;
    yaw_angle_deg = MPU6050_ANGLE_RESET;
    Mpu6050Kalman_Reset(&yaw_kalman);
    initialized = false;
    calibrated = false;
    published.status.timestamp_ms = now_ms;
    published.status.health = bus_ready ? MODULE_HEALTH_UNKNOWN :
                                          MODULE_HEALTH_DEGRADED;
}

void Mpu6050_Service(uint32_t now_ms)
{
    if (service_pending_transfer(now_ms)) {
        return;
    }

    if (!initialized) {
        if (state == MPU6050_STATE_DEGRADED &&
            (uint32_t)(now_ms - retry_started_ms) < MPU6050_RETRY_MS) {
            return;
        }
        if (state == MPU6050_STATE_DEGRADED) {
            init_step = INIT_READ_ID;
            retry_started_ms = now_ms;
            state = MPU6050_STATE_STARTUP;
        }
        if (!begin_init_transfer()) {
            note_error(now_ms);
        }
        return;
    }

    if (!begin_gyro_read(now_ms)) {
        note_error(now_ms);
    }
}

Mpu6050State Mpu6050_GetState(void)
{
    return state;
}

bool Mpu6050_GetSnapshot(Mpu6050Snapshot *out)
{
    if (out == 0) {
        return false;
    }
    *out = published;
    return true;
}
