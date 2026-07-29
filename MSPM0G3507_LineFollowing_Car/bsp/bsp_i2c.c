#include "bsp_i2c.h"

#include <string.h>

#include "ti_msp_dl_config.h"

#define BSP_I2C_HALF_PERIOD_US (5U)

typedef enum {
    I2C_OP_READ = 0,
    I2C_OP_WRITE
} I2cOperation;

typedef enum {
    I2C_SEND_ADDRESS_WRITE = 0,
    I2C_SEND_REGISTER,
    I2C_SEND_ADDRESS_READ,
    I2C_SEND_WRITE_DATA
} I2cSendStage;

typedef enum {
    I2C_PHASE_START_RELEASE = 0,
    I2C_PHASE_START_SDA_LOW,
    I2C_PHASE_START_SCL_LOW,
    I2C_PHASE_TX_BIT_LOW,
    I2C_PHASE_TX_BIT_HIGH,
    I2C_PHASE_TX_BIT_FALL,
    I2C_PHASE_TX_ACK_LOW,
    I2C_PHASE_TX_ACK_HIGH,
    I2C_PHASE_TX_ACK_SAMPLE,
    I2C_PHASE_RESTART_RELEASE_SDA,
    I2C_PHASE_RESTART_RELEASE_SCL,
    I2C_PHASE_RESTART_SDA_LOW,
    I2C_PHASE_RESTART_SCL_LOW,
    I2C_PHASE_RX_BIT_LOW,
    I2C_PHASE_RX_BIT_HIGH,
    I2C_PHASE_RX_BIT_SAMPLE,
    I2C_PHASE_RX_ACK_LOW,
    I2C_PHASE_RX_ACK_HIGH,
    I2C_PHASE_RX_ACK_FALL,
    I2C_PHASE_STOP_LOW,
    I2C_PHASE_STOP_SCL_HIGH,
    I2C_PHASE_STOP_SDA_HIGH
} I2cPhase;

static BSP_I2C_Status transfer_status = BSP_I2C_STATUS_IDLE;
static I2cOperation operation = I2C_OP_READ;
static I2cSendStage send_stage = I2C_SEND_ADDRESS_WRITE;
static I2cPhase phase = I2C_PHASE_START_RELEASE;
static uint8_t device_address = 0U;
static uint8_t register_address = 0U;
static uint8_t transfer_length = 0U;
static uint8_t transfer_index = 0U;
static uint8_t bit_index = 0U;
static uint8_t tx_byte = 0U;
static uint8_t rx_byte = 0U;
static uint8_t *read_buffer = 0;
static uint8_t write_buffer[BSP_I2C_MAX_TRANSFER] = {0};
static uint32_t transaction_start_us = 0U;
static uint32_t next_step_us = 0U;
static bool transaction_started = false;
static bool stop_with_error = false;

static void line_drive_low(uint32_t iomux, uint32_t pin)
{
    DL_GPIO_initDigitalOutput(iomux);
    DL_GPIO_clearPins(YBIMU_I2C_PORT, pin);
    DL_GPIO_enableOutput(YBIMU_I2C_PORT, pin);
}

static void line_release(uint32_t iomux, uint32_t pin)
{
    DL_GPIO_disableOutput(YBIMU_I2C_PORT, pin);
    DL_GPIO_initDigitalInput(iomux);
}

static bool line_is_high(uint32_t pin)
{
    return (DL_GPIO_readPins(YBIMU_I2C_PORT, pin) & pin) != 0U;
}

static void scl_low(void)
{
    line_drive_low(YBIMU_I2C_SCL_IOMUX, YBIMU_I2C_SCL_PIN);
}

static void scl_release(void)
{
    line_release(YBIMU_I2C_SCL_IOMUX, YBIMU_I2C_SCL_PIN);
}

static void sda_low(void)
{
    line_drive_low(YBIMU_I2C_SDA_IOMUX, YBIMU_I2C_SDA_PIN);
}

static void sda_release(void)
{
    line_release(YBIMU_I2C_SDA_IOMUX, YBIMU_I2C_SDA_PIN);
}

static void schedule_phase(I2cPhase next, uint32_t now_us)
{
    phase = next;
    next_step_us = now_us + BSP_I2C_HALF_PERIOD_US;
}

static void begin_send(uint8_t value, I2cSendStage stage, uint32_t now_us)
{
    tx_byte = value;
    bit_index = 0U;
    send_stage = stage;
    phase = I2C_PHASE_TX_BIT_LOW;
    next_step_us = now_us;
}

static void begin_stop(bool error, uint32_t now_us)
{
    stop_with_error = error;
    phase = I2C_PHASE_STOP_LOW;
    next_step_us = now_us;
}

static bool begin_transaction(uint8_t address,
                              uint8_t reg,
                              uint8_t *read_destination,
                              const uint8_t *write_source,
                              uint8_t length,
                              I2cOperation requested_operation)
{
    if (transfer_status == BSP_I2C_STATUS_BUSY ||
        address == 0U || length == 0U ||
        length > BSP_I2C_MAX_TRANSFER ||
        (requested_operation == I2C_OP_READ && read_destination == 0) ||
        (requested_operation == I2C_OP_WRITE && write_source == 0)) {
        return false;
    }

    if (requested_operation == I2C_OP_WRITE) {
        memcpy(write_buffer, write_source, length);
    }
    operation = requested_operation;
    device_address = address;
    register_address = reg;
    transfer_length = length;
    transfer_index = 0U;
    read_buffer = read_destination;
    transaction_started = false;
    stop_with_error = false;
    phase = I2C_PHASE_START_RELEASE;
    next_step_us = 0U;
    transfer_status = BSP_I2C_STATUS_BUSY;
    return true;
}

bool BSP_I2C_Init(void)
{
    scl_release();
    sda_release();
    transfer_status = BSP_I2C_STATUS_IDLE;
    transaction_started = false;
    return line_is_high(YBIMU_I2C_SCL_PIN) &&
           line_is_high(YBIMU_I2C_SDA_PIN);
}

bool BSP_I2C_BeginRead(uint8_t address,
                       uint8_t reg,
                       uint8_t *buffer,
                       uint8_t length)
{
    return begin_transaction(address, reg, buffer, 0, length, I2C_OP_READ);
}

bool BSP_I2C_BeginWrite(uint8_t address,
                        uint8_t reg,
                        const uint8_t *buffer,
                        uint8_t length)
{
    return begin_transaction(address, reg, 0, buffer, length, I2C_OP_WRITE);
}

BSP_I2C_Status BSP_I2C_GetStatus(void)
{
    return transfer_status;
}

static void handle_ack_success(uint32_t now_us)
{
    if (send_stage == I2C_SEND_ADDRESS_WRITE) {
        begin_send(register_address, I2C_SEND_REGISTER, now_us);
    } else if (send_stage == I2C_SEND_REGISTER) {
        if (operation == I2C_OP_READ) {
            schedule_phase(I2C_PHASE_RESTART_RELEASE_SDA, now_us);
        } else {
            begin_send(write_buffer[0], I2C_SEND_WRITE_DATA, now_us);
        }
    } else if (send_stage == I2C_SEND_ADDRESS_READ) {
        bit_index = 0U;
        rx_byte = 0U;
        phase = I2C_PHASE_RX_BIT_LOW;
        next_step_us = now_us;
    } else {
        transfer_index++;
        if (transfer_index >= transfer_length) {
            begin_stop(false, now_us);
        } else {
            begin_send(write_buffer[transfer_index],
                       I2C_SEND_WRITE_DATA, now_us);
        }
    }
}

void BSP_I2C_Service(uint32_t now_us)
{
    if (transfer_status != BSP_I2C_STATUS_BUSY ||
        (int32_t)(now_us - next_step_us) < 0) {
        return;
    }
    if (!transaction_started) {
        transaction_start_us = now_us;
        transaction_started = true;
    } else if ((uint32_t)(now_us - transaction_start_us) >
               BSP_I2C_TRANSACTION_TIMEOUT_US) {
        scl_release();
        sda_release();
        transfer_status = BSP_I2C_STATUS_ERROR;
        return;
    }

    switch (phase) {
        case I2C_PHASE_START_RELEASE:
            scl_release();
            sda_release();
            if (!line_is_high(YBIMU_I2C_SCL_PIN) ||
                !line_is_high(YBIMU_I2C_SDA_PIN)) {
                transfer_status = BSP_I2C_STATUS_ERROR;
                return;
            }
            schedule_phase(I2C_PHASE_START_SDA_LOW, now_us);
            break;
        case I2C_PHASE_START_SDA_LOW:
            sda_low();
            schedule_phase(I2C_PHASE_START_SCL_LOW, now_us);
            break;
        case I2C_PHASE_START_SCL_LOW:
            scl_low();
            begin_send((uint8_t)(device_address << 1),
                       I2C_SEND_ADDRESS_WRITE, now_us);
            break;
        case I2C_PHASE_TX_BIT_LOW:
            scl_low();
            if ((tx_byte & (uint8_t)(0x80U >> bit_index)) != 0U) {
                sda_release();
            } else {
                sda_low();
            }
            schedule_phase(I2C_PHASE_TX_BIT_HIGH, now_us);
            break;
        case I2C_PHASE_TX_BIT_HIGH:
            scl_release();
            if (!line_is_high(YBIMU_I2C_SCL_PIN)) {
                next_step_us = now_us + 1U;
                break;
            }
            schedule_phase(I2C_PHASE_TX_BIT_FALL, now_us);
            break;
        case I2C_PHASE_TX_BIT_FALL:
            scl_low();
            bit_index++;
            if (bit_index >= 8U) {
                sda_release();
                schedule_phase(I2C_PHASE_TX_ACK_LOW, now_us);
            } else {
                phase = I2C_PHASE_TX_BIT_LOW;
                next_step_us = now_us;
            }
            break;
        case I2C_PHASE_TX_ACK_LOW:
            scl_low();
            sda_release();
            schedule_phase(I2C_PHASE_TX_ACK_HIGH, now_us);
            break;
        case I2C_PHASE_TX_ACK_HIGH:
            scl_release();
            if (!line_is_high(YBIMU_I2C_SCL_PIN)) {
                next_step_us = now_us + 1U;
                break;
            }
            schedule_phase(I2C_PHASE_TX_ACK_SAMPLE, now_us);
            break;
        case I2C_PHASE_TX_ACK_SAMPLE:
            if (line_is_high(YBIMU_I2C_SDA_PIN)) {
                scl_low();
                begin_stop(true, now_us);
            } else {
                scl_low();
                handle_ack_success(now_us);
            }
            break;
        case I2C_PHASE_RESTART_RELEASE_SDA:
            sda_release();
            schedule_phase(I2C_PHASE_RESTART_RELEASE_SCL, now_us);
            break;
        case I2C_PHASE_RESTART_RELEASE_SCL:
            scl_release();
            if (!line_is_high(YBIMU_I2C_SCL_PIN)) {
                next_step_us = now_us + 1U;
                break;
            }
            schedule_phase(I2C_PHASE_RESTART_SDA_LOW, now_us);
            break;
        case I2C_PHASE_RESTART_SDA_LOW:
            sda_low();
            schedule_phase(I2C_PHASE_RESTART_SCL_LOW, now_us);
            break;
        case I2C_PHASE_RESTART_SCL_LOW:
            scl_low();
            begin_send((uint8_t)((device_address << 1) | 1U),
                       I2C_SEND_ADDRESS_READ, now_us);
            break;
        case I2C_PHASE_RX_BIT_LOW:
            scl_low();
            sda_release();
            schedule_phase(I2C_PHASE_RX_BIT_HIGH, now_us);
            break;
        case I2C_PHASE_RX_BIT_HIGH:
            scl_release();
            if (!line_is_high(YBIMU_I2C_SCL_PIN)) {
                next_step_us = now_us + 1U;
                break;
            }
            schedule_phase(I2C_PHASE_RX_BIT_SAMPLE, now_us);
            break;
        case I2C_PHASE_RX_BIT_SAMPLE:
            rx_byte = (uint8_t)(rx_byte << 1);
            if (line_is_high(YBIMU_I2C_SDA_PIN)) {
                rx_byte |= 1U;
            }
            scl_low();
            bit_index++;
            if (bit_index >= 8U) {
                read_buffer[transfer_index] = rx_byte;
                phase = I2C_PHASE_RX_ACK_LOW;
                next_step_us = now_us;
            } else {
                phase = I2C_PHASE_RX_BIT_LOW;
                next_step_us = now_us;
            }
            break;
        case I2C_PHASE_RX_ACK_LOW:
            scl_low();
            if ((uint8_t)(transfer_index + 1U) < transfer_length) {
                sda_low();
            } else {
                sda_release();
            }
            schedule_phase(I2C_PHASE_RX_ACK_HIGH, now_us);
            break;
        case I2C_PHASE_RX_ACK_HIGH:
            scl_release();
            if (!line_is_high(YBIMU_I2C_SCL_PIN)) {
                next_step_us = now_us + 1U;
                break;
            }
            schedule_phase(I2C_PHASE_RX_ACK_FALL, now_us);
            break;
        case I2C_PHASE_RX_ACK_FALL:
            scl_low();
            sda_release();
            transfer_index++;
            if (transfer_index >= transfer_length) {
                begin_stop(false, now_us);
            } else {
                bit_index = 0U;
                rx_byte = 0U;
                phase = I2C_PHASE_RX_BIT_LOW;
                next_step_us = now_us;
            }
            break;
        case I2C_PHASE_STOP_LOW:
            scl_low();
            sda_low();
            schedule_phase(I2C_PHASE_STOP_SCL_HIGH, now_us);
            break;
        case I2C_PHASE_STOP_SCL_HIGH:
            scl_release();
            if (!line_is_high(YBIMU_I2C_SCL_PIN)) {
                next_step_us = now_us + 1U;
                break;
            }
            schedule_phase(I2C_PHASE_STOP_SDA_HIGH, now_us);
            break;
        case I2C_PHASE_STOP_SDA_HIGH:
            sda_release();
            transfer_status = stop_with_error ? BSP_I2C_STATUS_ERROR :
                                                BSP_I2C_STATUS_DONE;
            break;
        default:
            scl_release();
            sda_release();
            transfer_status = BSP_I2C_STATUS_ERROR;
            break;
    }
}
