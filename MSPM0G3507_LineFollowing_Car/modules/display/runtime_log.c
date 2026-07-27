#include "runtime_log.h"

#include <stdio.h>
#include <string.h>

#include "ssd1306/ssd1306.h"

static char lines[RUNTIME_LOG_CAPACITY][RUNTIME_LOG_LINE_BUFFER];
static uint8_t head;
static uint8_t count;
static char last_payload[18];

static bool payload_is_ascii(const char *payload)
{
    const unsigned char *character = (const unsigned char *)payload;

    while (*character != '\0') {
        if ((*character < 0x20U) || (*character > 0x7EU)) {
            return false;
        }
        character++;
    }
    return true;
}

static void push_line(uint32_t now_ms, const char *payload)
{
    uint8_t write_index = (uint8_t)((head + count) % RUNTIME_LOG_CAPACITY);

    if (count == RUNTIME_LOG_CAPACITY) {
        write_index = head;
        head = (uint8_t)((head + 1U) % RUNTIME_LOG_CAPACITY);
    } else {
        count++;
    }
    (void)snprintf(lines[write_index], RUNTIME_LOG_LINE_BUFFER, "%04lu %s",
                   (unsigned long)(now_ms % 10000U), payload);
}

void RuntimeLog_Init(void)
{
    (void)memset(lines, 0, sizeof(lines));
    (void)memset(last_payload, 0, sizeof(last_payload));
    head = 0U;
    count = 0U;
}

bool RuntimeLog_Push(uint32_t now_ms, const char *event)
{
    char payload[sizeof(last_payload)];

    if (event == NULL) {
        return false;
    }
    if (!payload_is_ascii(event)) {
        return false;
    }
    (void)snprintf(payload, sizeof(payload), "%s", event);
    if (strcmp(last_payload, payload) == 0) {
        return false;
    }

    (void)snprintf(last_payload, sizeof(last_payload), "%s", payload);
    push_line(now_ms, last_payload);
    return true;
}

bool RuntimeLog_PushMotor(uint32_t now_ms, int16_t left, int16_t right)
{
    char payload[18];
    int left_value = left;
    int right_value = right;

    if (left_value < -999) {
        left_value = -999;
    } else if (left_value > 999) {
        left_value = 999;
    }
    if (right_value < -999) {
        right_value = -999;
    } else if (right_value > 999) {
        right_value = 999;
    }

    (void)snprintf(payload, sizeof(payload), "TX L%03d R%03d", left_value,
                   right_value);
    return RuntimeLog_Push(now_ms, payload);
}

uint8_t RuntimeLog_Snapshot(
    char out[RUNTIME_LOG_CAPACITY][RUNTIME_LOG_LINE_BUFFER])
{
    uint8_t index;

    for (index = 0U; index < RUNTIME_LOG_CAPACITY; ++index) {
        out[index][0] = '\0';
    }
    for (index = 0U; index < count; ++index) {
        uint8_t read_index = (uint8_t)((head + index) % RUNTIME_LOG_CAPACITY);

        (void)snprintf(out[index], RUNTIME_LOG_LINE_BUFFER, "%s",
                       lines[read_index]);
    }
    return count;
}

void RuntimeLog_Draw(void)
{
    char snapshot[RUNTIME_LOG_CAPACITY][RUNTIME_LOG_LINE_BUFFER];
    uint8_t page;

    (void)RuntimeLog_Snapshot(snapshot);
    for (page = 0U; page < RUNTIME_LOG_CAPACITY; ++page) {
        Ssd1306_DrawText(page, 0U, snapshot[page]);
    }
}
