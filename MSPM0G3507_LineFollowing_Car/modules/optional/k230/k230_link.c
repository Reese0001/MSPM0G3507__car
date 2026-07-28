#include "k230_link.h"

#include <limits.h>
#include <string.h>

#include "../../../bsp/bsp_k230_uart.h"
#include "k230_config.h"
#include "k230_protocol.h"

static volatile uint8_t rx_buffer[K230_RX_BUFFER_LEN];
static volatile uint8_t rx_head = 0U;
static volatile uint8_t rx_tail = 0U;
static K230VisionSnapshot latest_snapshot = {0};
static K230LinkDiagnostics diagnostics = {0};
static volatile uint32_t rx_overflow_count = 0U;

static bool to_int16(int32_t value, int16_t *out)
{
    if (value < INT16_MIN || value > INT16_MAX) {
        return false;
    }
    *out = (int16_t)value;
    return true;
}

static bool publish_frame(const K230Frame *frame, uint32_t now_ms)
{
    K230VisionSnapshot candidate = {0};

    if (frame == 0 || frame->field_count < 1U ||
        frame->field_count > 5U || frame->fields[0] < 0L ||
        frame->fields[0] > 100L) {
        return false;
    }
    candidate.event_id = frame->id;
    candidate.confidence = (uint8_t)frame->fields[0];
    if ((frame->field_count > 1U &&
         !to_int16(frame->fields[1], &candidate.x)) ||
        (frame->field_count > 2U &&
         !to_int16(frame->fields[2], &candidate.y)) ||
        (frame->field_count > 3U &&
         !to_int16(frame->fields[3], &candidate.width)) ||
        (frame->field_count > 4U &&
         !to_int16(frame->fields[4], &candidate.height))) {
        return false;
    }
    memcpy(candidate.text, frame->text, sizeof(candidate.text));
    candidate.status.timestamp_ms = now_ms;
    candidate.status.sequence = (uint16_t)(latest_snapshot.status.sequence + 1U);
    candidate.status.valid = true;
    candidate.status.health = MODULE_HEALTH_OK;
    latest_snapshot = candidate;
    return true;
}

void K230Link_Init(void)
{
    rx_head = 0U;
    rx_tail = 0U;
    memset(&latest_snapshot, 0, sizeof(latest_snapshot));
    memset(&diagnostics, 0, sizeof(diagnostics));
    rx_overflow_count = 0U;
    K230Protocol_Init();
    BSP_K230_UartInit();
}

void K230Link_OnRxByteFromISR(uint8_t byte)
{
    uint8_t next = (uint8_t)((rx_head + 1U) % K230_RX_BUFFER_LEN);

    if (next == rx_tail) {
        rx_overflow_count++;
        return;
    }
    rx_buffer[rx_head] = byte;
    rx_head = next;
}

void K230Link_Service(uint32_t now_ms)
{
    while (rx_tail != rx_head) {
        K230Frame frame;
        uint8_t byte = rx_buffer[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1U) % K230_RX_BUFFER_LEN);
        K230Protocol_ConsumeByte(byte);
        if (K230Protocol_TakeFrame(&frame)) {
            if (publish_frame(&frame, now_ms)) {
                diagnostics.accepted_frames++;
            } else {
                diagnostics.rejected_events++;
            }
        }
    }
}

bool K230Link_GetSnapshot(uint32_t now_ms, K230VisionSnapshot *out)
{
    if (out == 0 || !latest_snapshot.status.valid) {
        return false;
    }
    *out = latest_snapshot;
    if (!ModuleStatus_IsFresh(&out->status, now_ms,
                              K230_VISION_STALE_MS)) {
        out->status.health = MODULE_HEALTH_DEGRADED;
        return false;
    }
    return true;
}

void K230Link_GetDiagnostics(K230LinkDiagnostics *out)
{
    if (out != 0) {
        *out = diagnostics;
        out->rejected_frames = K230Protocol_GetRejectedCount();
        out->rx_overflows = rx_overflow_count;
    }
}

void K230_UART_IRQHandler(void)
{
    while (!BSP_K230_UartRxEmpty()) {
        K230Link_OnRxByteFromISR(BSP_K230_UartReadByte());
    }
}
