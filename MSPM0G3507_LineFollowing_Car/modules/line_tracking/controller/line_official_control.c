#include "line_official_control.h"

#include "../../../config/line_lookup_config.h"
#include "line_lookup_control.h"

static LineOfficialControlDiagnostics diagnostics;
static LineOfficialControlResult last_result;
static uint32_t noise_started_ms;
static bool noise_active;
static bool have_last_result;

static float absolute_float(float value)
{
    return value < 0.0f ? -value : value;
}

static int16_t clamp_i16(int16_t value, int16_t low, int16_t high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static int16_t damping_from_rate(float yaw_rate_dps, bool fresh)
{
    float scaled;
    int16_t damping;

    diagnostics.yaw_rate_dps = fresh ? yaw_rate_dps : 0.0f;
    diagnostics.imu_used = fresh &&
                           absolute_float(yaw_rate_dps) >
                               LINE_YAW_RATE_DEADBAND_DPS;
    if (!diagnostics.imu_used) {
        return 0;
    }
    scaled = yaw_rate_dps * LINE_YAW_DAMPING_GAIN;
    damping = (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
    return clamp_i16(damping, -LINE_YAW_DAMPING_LIMIT,
                     LINE_YAW_DAMPING_LIMIT);
}

static void set_status(LineOfficialControlResult *out,
                       uint16_t sequence,
                       uint32_t now_ms)
{
    out->estimate.status.timestamp_ms = now_ms;
    out->estimate.status.sequence = sequence;
    out->estimate.status.valid = true;
    out->estimate.status.health = MODULE_HEALTH_OK;
}

static void set_lost(LineOfficialControlResult *out,
                     uint16_t sequence,
                     uint32_t now_ms)
{
    *out = (LineOfficialControlResult){0};
    set_status(out, sequence, now_ms);
    out->estimate.event = LINE_EVENT_LOST;
    out->recovery_direction = diagnostics.direction;
}

static void build_follow(int8_t control_position,
                         LineEvent event,
                         uint16_t sequence,
                         uint32_t now_ms,
                         float yaw_rate_dps,
                         bool imu_fresh,
                         LineOfficialControlResult *out)
{
    LineLookupCommand lookup = LineLookupControl_Step(control_position);
    int16_t max_turn;
    int16_t turn;

    *out = (LineOfficialControlResult){0};
    set_status(out, sequence, now_ms);
    out->estimate.error = (float)control_position;
    out->estimate.predicted_error = (float)control_position;
    out->estimate.confidence = event == LINE_EVENT_NONE ? 60U : 50U;
    out->estimate.event = event;
    out->follow.forward = (int16_t)((lookup.left + lookup.right) / 2);
    turn = (int16_t)((lookup.right - lookup.left) / 2);
    diagnostics.damping_command = damping_from_rate(yaw_rate_dps, imu_fresh);
    turn = (int16_t)(turn - diagnostics.damping_command);
    max_turn = out->follow.forward;
    if ((LINE_LOOKUP_COMMAND_LIMIT - out->follow.forward) < max_turn) {
        max_turn = (int16_t)(LINE_LOOKUP_COMMAND_LIMIT - out->follow.forward);
    }
    out->follow.turn = clamp_i16(turn, (int16_t)-max_turn, max_turn);
    out->follow.valid = lookup.valid;
    out->recovery_direction = diagnostics.direction;
    out->imu_used = diagnostics.imu_used;
}

void LineOfficialControl_Init(void)
{
    diagnostics = (LineOfficialControlDiagnostics){0};
    diagnostics.direction = 1;
    last_result = (LineOfficialControlResult){0};
    noise_started_ms = 0U;
    noise_active = false;
    have_last_result = false;
}

bool LineOfficialControl_Step(const LinePositionResult *position,
                              uint16_t sequence,
                              uint32_t now_ms,
                              float yaw_rate_dps,
                              bool imu_fresh,
                              LineOfficialControlResult *out)
{
    int8_t control_position;
    LineEvent event;

    if (position == 0 || out == 0 ||
        (unsigned int)position->type > (unsigned int)LINE_PATTERN_NOISE) {
        return false;
    }
    if (position->type == LINE_PATTERN_LOST) {
        noise_active = false;
        set_lost(out, sequence, now_ms);
        return true;
    }
    if (position->type == LINE_PATTERN_NOISE) {
        if (!noise_active) {
            noise_active = true;
            noise_started_ms = now_ms;
        }
        if (have_last_result &&
            (uint32_t)(now_ms - noise_started_ms) <= LINE_NOISE_HOLD_MS) {
            *out = last_result;
            set_status(out, sequence, now_ms);
            return true;
        }
        set_lost(out, sequence, now_ms);
        return true;
    }

    noise_active = false;
    control_position = position->type == LINE_PATTERN_WIDE ?
                           position->candidate_position :
                           position->stable_position;
    if (control_position != 0) {
        diagnostics.direction = control_position < 0 ? (int8_t)-1 : (int8_t)1;
    }
    event = position->type == LINE_PATTERN_WIDE ? LINE_EVENT_WIDE_BLACK :
                                                  LINE_EVENT_NONE;
    build_follow(control_position, event, sequence, now_ms,
                 yaw_rate_dps, imu_fresh, out);
    last_result = *out;
    have_last_result = out->follow.valid;
    return true;
}

void LineOfficialControl_GetDiagnostics(LineOfficialControlDiagnostics *out)
{
    if (out != 0) {
        *out = diagnostics;
    }
}
