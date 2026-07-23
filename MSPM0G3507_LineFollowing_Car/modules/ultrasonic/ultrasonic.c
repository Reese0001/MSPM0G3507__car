#include "ultrasonic.h"

#include "bsp_ultrasonic.h"
#include "ultrasonic_config.h"

typedef enum {
    ULTRA_IDLE = 0,
    ULTRA_TRIGGER_HIGH,
    ULTRA_WAIT_RISE,
    ULTRA_WAIT_FALL
} UltrasonicState;

static UltrasonicSnapshot snapshot = {0};
static volatile UltrasonicState state = ULTRA_IDLE;
static uint32_t last_trigger_us = 0U;
static uint32_t trigger_high_us = 0U;
static uint32_t measure_start_us = 0U;
static volatile uint32_t echo_rise_us = 0U;
static volatile uint32_t captured_pulse_us = 0U;
static volatile bool captured_ready = false;

static uint32_t elapsed_us(uint32_t now_us, uint32_t start_us)
{
    return (uint32_t)(now_us - start_us);
}

static void publish_invalid(uint32_t now_us, ModuleHealth health)
{
    snapshot.status.timestamp_ms = now_us / 1000U;
    snapshot.status.sequence++;
    snapshot.status.valid = false;
    snapshot.status.health = health;
    snapshot.pulse_us = 0U;
    snapshot.distance_mm = 0U;
}

static void publish_valid(uint32_t now_us, uint32_t pulse_us, uint16_t distance_mm)
{
    snapshot.status.timestamp_ms = now_us / 1000U;
    snapshot.status.sequence++;
    snapshot.status.valid = true;
    snapshot.status.health = MODULE_HEALTH_OK;
    snapshot.pulse_us = pulse_us;
    snapshot.distance_mm = distance_mm;
}

bool Ultrasonic_PulseUsToMm(uint32_t pulse_us, uint16_t *distance_mm)
{
    if (distance_mm == 0 ||
        pulse_us < ULTRASONIC_MIN_PULSE_US ||
        pulse_us > ULTRASONIC_MAX_PULSE_US) {
        return false;
    }

    *distance_mm = (uint16_t)((pulse_us * 343U + 1000U) / 2000U);
    return true;
}

bool Ultrasonic_GetSnapshot(UltrasonicSnapshot *out)
{
    if (out == 0) {
        return false;
    }

    *out = snapshot;
    return true;
}

void Ultrasonic_Init(void)
{
    uint32_t now_us;

    BSP_Ultrasonic_Init();
    BSP_Ultrasonic_SetTrig(false);
    now_us = BSP_Ultrasonic_NowUs();
    state = ULTRA_IDLE;
    last_trigger_us = now_us;
    trigger_high_us = now_us;
    measure_start_us = now_us;
    echo_rise_us = 0U;
    captured_pulse_us = 0U;
    captured_ready = false;
    snapshot.status.timestamp_ms = now_us / 1000U;
    snapshot.status.sequence = 0U;
    snapshot.status.valid = false;
    snapshot.status.health = MODULE_HEALTH_UNKNOWN;
    snapshot.pulse_us = 0U;
    snapshot.distance_mm = 0U;
}

void Ultrasonic_Service(uint32_t now_us)
{
    uint32_t pulse_us;
    uint16_t distance_mm;

    if (captured_ready) {
        pulse_us = captured_pulse_us;
        captured_ready = false;
        state = ULTRA_IDLE;
        if (Ultrasonic_PulseUsToMm(pulse_us, &distance_mm)) {
            publish_valid(now_us, pulse_us, distance_mm);
        } else {
            publish_invalid(now_us, MODULE_HEALTH_DEGRADED);
        }
        return;
    }

    if (state == ULTRA_IDLE &&
        elapsed_us(now_us, last_trigger_us) >= ULTRASONIC_TRIGGER_PERIOD_US) {
        BSP_Ultrasonic_SetTrig(true);
        last_trigger_us = now_us;
        trigger_high_us = now_us;
        state = ULTRA_TRIGGER_HIGH;
    } else if (state == ULTRA_TRIGGER_HIGH &&
               elapsed_us(now_us, trigger_high_us) >= 10U) {
        BSP_Ultrasonic_SetTrig(false);
        measure_start_us = now_us;
        state = ULTRA_WAIT_RISE;
    } else if ((state == ULTRA_WAIT_RISE || state == ULTRA_WAIT_FALL) &&
               elapsed_us(now_us, measure_start_us) >=
                   ULTRASONIC_ECHO_TIMEOUT_US) {
        publish_invalid(now_us, MODULE_HEALTH_DEGRADED);
        state = ULTRA_IDLE;
    }
}

void Ultrasonic_OnEchoEdge(bool high, uint32_t timestamp_us)
{
    if (high && state == ULTRA_WAIT_RISE) {
        echo_rise_us = timestamp_us;
        state = ULTRA_WAIT_FALL;
    } else if (!high && state == ULTRA_WAIT_FALL) {
        captured_pulse_us = elapsed_us(timestamp_us, echo_rise_us);
        state = ULTRA_IDLE;
        captured_ready = true;
    }
}
