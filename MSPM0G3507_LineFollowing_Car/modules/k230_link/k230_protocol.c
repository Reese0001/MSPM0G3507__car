#include "k230_protocol.h"

#include <string.h>

#include "k230_config.h"

typedef enum {
    WAIT_DOLLAR = 0,
    READ_FRAME,
    WAIT_LF,
    DISCARD
} K230ParserState;

static K230ParserState parser_state = WAIT_DOLLAR;
static char frame_buffer[K230_FRAME_MAX_LEN + 1U];
static uint16_t frame_length = 0U;
static K230Frame pending_frame = {0};
static bool frame_ready = false;

static bool parse_unsigned(const char *text, uint16_t length, uint16_t *out)
{
    uint32_t value = 0U;
    uint16_t index;

    if (text == 0 || out == 0 || length == 0U) {
        return false;
    }
    for (index = 0U; index < length; index++) {
        uint8_t digit;
        if (text[index] < '0' || text[index] > '9') {
            return false;
        }
        digit = (uint8_t)(text[index] - '0');
        value = value * 10U + digit;
        if (value > K230_FRAME_MAX_LEN) {
            return false;
        }
    }
    *out = (uint16_t)value;
    return true;
}

static bool token_is_integer(const char *text, uint16_t length)
{
    uint16_t index = 0U;

    if (length == 0U) {
        return false;
    }
    if (text[0] == '-') {
        index = 1U;
        if (length == 1U) {
            return false;
        }
    }
    for (; index < length; index++) {
        if (text[index] < '0' || text[index] > '9') {
            return false;
        }
    }
    return true;
}

static bool parse_int32(const char *text, uint16_t length, int32_t *out)
{
    bool negative = false;
    uint16_t index = 0U;
    uint32_t value = 0U;
    uint32_t limit;

    if (text == 0 || out == 0 || !token_is_integer(text, length)) {
        return false;
    }
    if (text[0] == '-') {
        negative = true;
        index = 1U;
    }
    limit = negative ? 2147483648UL : 2147483647UL;
    for (; index < length; index++) {
        uint32_t digit = (uint32_t)(text[index] - '0');
        if (value > (limit - digit) / 10U) {
            return false;
        }
        value = value * 10U + digit;
    }
    if (negative && value == 2147483648UL) {
        *out = (-2147483647L - 1L);
    } else if (negative) {
        *out = -(int32_t)value;
    } else {
        *out = (int32_t)value;
    }
    return true;
}

static bool copy_text_field(K230Frame *frame,
                            const char *text,
                            uint16_t length)
{
    uint16_t index;

    if (frame->text[0] != '\0' || length == 0U ||
        length >= K230_PROTOCOL_TEXT_LEN) {
        return false;
    }
    for (index = 0U; index < length; index++) {
        if (text[index] < 32 || text[index] > 126 ||
            text[index] == '$' || text[index] == '#') {
            return false;
        }
    }
    memcpy(frame->text, text, length);
    frame->text[length] = '\0';
    return true;
}

static bool parse_frame(K230Frame *out)
{
    K230Frame candidate = {0};
    uint16_t declared_length;
    uint16_t index = 1U;
    uint16_t length_start = index;
    uint16_t token_start;
    uint16_t hash_index;
    uint8_t id_tens;
    uint8_t id_ones;

    if (out == 0 || frame_length < 8U || frame_buffer[0] != '$' ||
        frame_buffer[frame_length - 1U] != '#') {
        return false;
    }
    hash_index = frame_length - 1U;
    while (index < hash_index && frame_buffer[index] != ',') {
        index++;
    }
    if (index >= hash_index ||
        !parse_unsigned(&frame_buffer[length_start],
                        (uint16_t)(index - length_start),
                        &declared_length) ||
        declared_length != frame_length) {
        return false;
    }

    index++;
    if ((uint16_t)(index + 2U) >= hash_index ||
        frame_buffer[index] < '0' || frame_buffer[index] > '9' ||
        frame_buffer[index + 1U] < '0' ||
        frame_buffer[index + 1U] > '9' ||
        frame_buffer[index + 2U] != ',') {
        return false;
    }
    id_tens = (uint8_t)(frame_buffer[index] - '0');
    id_ones = (uint8_t)(frame_buffer[index + 1U] - '0');
    candidate.id = (uint8_t)(id_tens * 10U + id_ones);
    if (candidate.id < K230_VENDOR_ID_MIN ||
        candidate.id > K230_VENDOR_ID_MAX ||
        !K230_Config_IsIdAllowed(candidate.id)) {
        return false;
    }

    token_start = index + 3U;
    for (index = token_start; index <= hash_index; index++) {
        if (index == hash_index || frame_buffer[index] == ',') {
            uint16_t token_length = (uint16_t)(index - token_start);
            if (token_length == 0U) {
                return false;
            }
            if (token_is_integer(&frame_buffer[token_start], token_length)) {
                if (candidate.field_count >= K230_PROTOCOL_MAX_FIELDS ||
                    !parse_int32(&frame_buffer[token_start],
                                 token_length,
                                 &candidate.fields[candidate.field_count])) {
                    return false;
                }
                candidate.field_count++;
            } else if (!copy_text_field(&candidate,
                                        &frame_buffer[token_start],
                                        token_length)) {
                return false;
            }
            token_start = index + 1U;
        }
    }

    *out = candidate;
    return true;
}

static void reset_parser(void)
{
    parser_state = WAIT_DOLLAR;
    frame_length = 0U;
}

void K230Protocol_Init(void)
{
    reset_parser();
    frame_ready = false;
    memset(frame_buffer, 0, sizeof(frame_buffer));
    memset(&pending_frame, 0, sizeof(pending_frame));
}

void K230Protocol_ConsumeByte(uint8_t byte)
{
    if (byte == '$') {
        frame_buffer[0] = '$';
        frame_length = 1U;
        parser_state = READ_FRAME;
        return;
    }

    if (parser_state == WAIT_DOLLAR) {
        return;
    }
    if (parser_state == DISCARD) {
        if (byte == '\n') {
            reset_parser();
        }
        return;
    }
    if (parser_state == WAIT_LF) {
        if (byte == '\n') {
            K230Frame candidate;
            frame_buffer[frame_length] = '\0';
            if (!frame_ready && parse_frame(&candidate)) {
                pending_frame = candidate;
                frame_ready = true;
            }
            reset_parser();
        } else {
            parser_state = DISCARD;
        }
        return;
    }

    if (byte < 32U || byte > 126U ||
        frame_length >= K230_FRAME_MAX_LEN) {
        parser_state = DISCARD;
        return;
    }
    frame_buffer[frame_length++] = (char)byte;
    if (byte == '#') {
        parser_state = WAIT_LF;
    }
}

bool K230Protocol_TakeFrame(K230Frame *out)
{
    if (out == 0 || !frame_ready) {
        return false;
    }
    *out = pending_frame;
    frame_ready = false;
    return true;
}
