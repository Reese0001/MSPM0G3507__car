#include "dashboard.h"

#include <stdio.h>

#include "ssd1306/ssd1306.h"

static int16_t round_float(float value)
{
    return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

/* The compact page prioritizes sensor/fusion/motor values during tuning. */
void Dashboard_Render(const AppDiagnostics *data)
{
    char text[22];

    if (data == 0) {
        return;
    }
    Ssd1306_ClearBuffer();
    if (data->lap.state == LAP_FINISHED) {
        (void)snprintf(text, sizeof(text), "STOP %03lu.%03lus",
                       (unsigned long)(data->lap.elapsed_ms / 1000U),
                       (unsigned long)(data->lap.elapsed_ms % 1000U));
    } else if (data->run_started) {
        (void)snprintf(text, sizeof(text), "RUN %03lu.%03lus",
                       (unsigned long)(data->lap.elapsed_ms / 1000U),
                       (unsigned long)(data->lap.elapsed_ms % 1000U));
    } else {
        (void)snprintf(text, sizeof(text), "WAIT K1 SAFE");
    }
    Ssd1306_DrawText(0U, 0U, text);
    (void)snprintf(text, sizeof(text), "X1 %d X2 %d",
                   (data->raw_x_bits & 0x02U) != 0U,
                   (data->raw_x_bits & 0x01U) != 0U);
    Ssd1306_DrawText(1U, 0U, text);
    (void)snprintf(text, sizeof(text), "X3 %d X4 %d",
                   (data->raw_x_bits & 0x04U) != 0U,
                   (data->raw_x_bits & 0x08U) != 0U);
    Ssd1306_DrawText(2U, 0U, text);
    (void)snprintf(text, sizeof(text), "LINE B%02X P%+d",
                   (unsigned int)data->line.black_bits,
                   (int)data->line.position);
    Ssd1306_DrawText(3U, 0U, text);
    (void)snprintf(text, sizeof(text), "ERR%+03d TRE%+03d",
                   (int)round_float(data->line.error * 10.0f),
                   (int)round_float(data->line.trend * 0.01f));
    Ssd1306_DrawText(4U, 0U, text);
    (void)snprintf(text, sizeof(text), "SPD%03d DIF%+03d",
                   (int)((data->feedback.left_speed_mm_s +
                          data->feedback.right_speed_mm_s) * 0.5f),
                   (int)(data->feedback.right_speed_mm_s -
                         data->feedback.left_speed_mm_s));
    Ssd1306_DrawText(5U, 0U, text);
    (void)snprintf(text, sizeof(text), "DST%04d ANG%+03d",
                   (int)(data->distance_mm / 10.0f),
                   (int)round_float(data->line.yaw_angle_deg));
    Ssd1306_DrawText(6U, 0U, text);
    (void)snprintf(text, sizeof(text), "YAW%+03d TURN%+03d",
                   (int)round_float(data->line.yaw_rate_dps),
                   (int)data->line.turn_command);
    Ssd1306_DrawText(6U, 0U, text);
    switch (data->wifi_state) {
    case WIFI_AT_PROBE_OK:
        (void)snprintf(text, sizeof(text), "ESP OK");
        break;
    case WIFI_AT_PROBE_TIMEOUT:
        (void)snprintf(text, sizeof(text), "ESP TIMEOUT");
        break;
    default:
        (void)snprintf(text, sizeof(text), "ESP WAIT");
        break;
    }
    Ssd1306_DrawText(7U, 0U, text);
}
