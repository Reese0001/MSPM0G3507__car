#include "dashboard.h"

#include "../../modules/display/ssd1306.h"

static char digit_char(uint8_t value)
{
    return (char)('0' + value);
}

static void format_i16(char *out, int16_t value)
{
    uint16_t magnitude;

    /* Fixed sign + three digits keeps every field a constant width. */
    out[0] = value < 0 ? '-' : '+';
    magnitude = value < 0 ? (uint16_t)-value : (uint16_t)value;
    if (magnitude > 999U) {
        magnitude = 999U;
    }
    out[1] = digit_char((uint8_t)(magnitude / 100U));
    out[2] = digit_char((uint8_t)((magnitude / 10U) % 10U));
    out[3] = digit_char((uint8_t)(magnitude % 10U));
    out[4] = '\0';
}

static void format_i8(char *out, int8_t value)
{
    uint8_t magnitude = value < 0 ? (uint8_t)-value : (uint8_t)value;

    out[0] = value < 0 ? '-' : '+';
    out[1] = digit_char((uint8_t)(magnitude % 10U));
    out[2] = '\0';
}

static void render_bits(const AppDiagnostics *data)
{
    char text[14];
    uint8_t bit;

    text[0] = 'B';
    text[1] = ':';
    for (bit = 0U; bit < 8U; bit++) {
        /* Sensor 1 (bit 0, leftmost) prints first. */
        text[2U + bit] =
            (data->black_bits & (uint8_t)(1U << bit)) != 0U ? '1' : '0';
    }
    text[10] = ' ';
    text[11] = 'T';
    text[12] = digit_char(data->pattern_type);
    text[13] = '\0';
    Ssd1306_DrawText(1U, 0U, text);
}

static void render_position(const AppDiagnostics *data)
{
    char text[16];
    char value[5];

    text[0] = 'P';
    text[1] = ':';
    format_i8(&text[2], data->stable_position);
    text[4] = ' ';
    text[5] = 'C';
    text[6] = ':';
    format_i8(&text[7], data->candidate_position);
    text[9] = '\0';
    Ssd1306_DrawText(2U, 0U, text);

    text[0] = 'L';
    text[1] = ':';
    format_i16(value, data->left_command);
    text[2] = value[0];
    text[3] = value[1];
    text[4] = value[2];
    text[5] = value[3];
    text[6] = ' ';
    text[7] = 'R';
    text[8] = ':';
    format_i16(value, data->right_command);
    text[9] = value[0];
    text[10] = value[1];
    text[11] = value[2];
    text[12] = value[3];
    text[13] = '\0';
    Ssd1306_DrawText(3U, 0U, text);
}

static const char *fault_label(uint8_t fault_code)
{
    switch (fault_code) {
    case APP_FAULT_CORNER_SEARCH:
        return "C-SEARCH";
    case APP_FAULT_LINE_LOST:
        return "L-LOST";
    case APP_FAULT_OLED_I2C:
        return "OLED-I2C";
    case APP_FAULT_MOTOR_UART:
        return "M-UART";
    case APP_FAULT_CONTROL_HEARTBEAT:
        return "CTRL-HB";
    case APP_FAULT_SENSOR_HEARTBEAT:
        return "SENS-HB";
    default:
        return "OK";
    }
}

static void render_states(const AppDiagnostics *data)
{
    char text[16];

    text[0] = 'S';
    text[1] = digit_char(data->safety_state);
    text[2] = ' ';
    text[3] = 'R';
    text[4] = digit_char(data->recovery_state);
    text[5] = ' ';
    text[6] = 'M';
    text[7] = digit_char(data->mpu_state);
    text[8] = '\0';
    Ssd1306_DrawText(4U, 0U, text);
    Ssd1306_DrawText(5U, 0U, "F:        ");
    Ssd1306_DrawText(5U, 2U, fault_label(data->fault_code));
}

void Dashboard_Render(const AppDiagnostics *data)
{
    if (data == 0) {
        return;
    }
    Ssd1306_DrawText(0U, 0U, "LINE CAR DIAG");
    render_bits(data);
    render_position(data);
    render_states(data);
}
