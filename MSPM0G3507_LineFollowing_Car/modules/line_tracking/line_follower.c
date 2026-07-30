#include "line_follower.h"

#include "line_tracking_config.h"

#define LINE_COMMAND_LIMIT       (DRIVE_COMMAND_MAX)
#define LINE_SEEK_COMMAND        (100)
#define LINE_REACQUIRE_FRAMES    (3U)
#define LINE_YAW_FILTER_ALPHA    (0.50f)
#define LINE_PREDICT_LIMIT       (3.5f)
#define RAD_TO_DEG               (57.2957795f)

static LineFollowerMode mode;
static int8_t direction;
static uint8_t reacquire_frames;
static MotionRequest last_follow;
static uint32_t last_follow_ms;
static float history_error[LINE_HISTORY_FRAMES];
static uint32_t history_time[LINE_HISTORY_FRAMES];
static uint8_t history_count;
static uint8_t history_head;
static float filtered_error;
static float error_rate;
static float predicted_error;
static float filtered_yaw_rate;
static int16_t last_turn_command;
static float heading_target_deg;
static float heading_error_deg;
static uint16_t centered_frames;
static bool heading_hold;

static float absolute_float(float value)
{
    return value < 0.0f ? -value : value;
}

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int16_t clamp_command(int32_t value)
{
    if (value < 0) {
        return 0;
    }
    if (value > LINE_COMMAND_LIMIT) {
        return LINE_COMMAND_LIMIT;
    }
    return (int16_t)value;
}

static int16_t round_i16(float value)
{
    return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static float wrap_angle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static void reset_history(void)
{
    uint8_t index;

    for (index = 0U; index < LINE_HISTORY_FRAMES; index++) {
        history_error[index] = 0.0f;
        history_time[index] = 0U;
    }
    history_count = 0U;
    history_head = 0U;
    filtered_error = 0.0f;
    error_rate = 0.0f;
    predicted_error = 0.0f;
}

static void push_history(float error, uint32_t now_ms)
{
    uint8_t old_index;
    uint32_t elapsed_ms;

    if (history_count == 0U ||
        (filtered_error < 0.0f && error > 0.0f) ||
        (filtered_error > 0.0f && error < 0.0f)) {
        filtered_error = error;
    } else {
        filtered_error += LINE_FILTER_ALPHA * (error - filtered_error);
    }

    history_error[history_head] = error;
    history_time[history_head] = now_ms;
    if (history_count < LINE_HISTORY_FRAMES) {
        history_count++;
    }
    history_head = (uint8_t)((history_head + 1U) % LINE_HISTORY_FRAMES);

    error_rate = 0.0f;
    if (history_count > LINE_HISTORY_RATE_LAG) {
        old_index = (uint8_t)((history_head + LINE_HISTORY_FRAMES -
                               1U - LINE_HISTORY_RATE_LAG) %
                              LINE_HISTORY_FRAMES);
        elapsed_ms = now_ms - history_time[old_index];
        if (elapsed_ms != 0U) {
            error_rate = (error - history_error[old_index]) /
                         ((float)elapsed_ms / 1000.0f);
        }
    }
    predicted_error = clamp_float(
        filtered_error + LINE_PREDICTION_HORIZON_S * error_rate,
        -LINE_PREDICT_LIMIT, LINE_PREDICT_LIMIT);
}

static int16_t turn_from_state(float error,
                               float trend,
                               float yaw_rate_dps,
                               float heading_error,
                               bool use_heading)
{
    float control = LINE_KP * error + LINE_KD * trend -
                    LINE_KYAW * yaw_rate_dps;
    if (use_heading) {
        control += LINE_KTHETA * heading_error;
    }
    float turn = (float)LINE_MOTOR_TURN_SIGN * control;

    turn = clamp_float(turn, -LINE_TURN_LIMIT, LINE_TURN_LIMIT);
    return round_i16(turn);
}

static int16_t base_speed(uint8_t confidence, float error)
{
    float magnitude = absolute_float(error);
    float speed;

    if (confidence < 60U || magnitude >= 2.5f) {
        return round_i16(LINE_SHARP_CURVE_SPEED_MM_S);
    }
    if (magnitude >= 1.5f) {
        return round_i16(LINE_CURVE_SPEED_MM_S);
    }
    if (magnitude >= 0.5f) {
        speed = LINE_CURVE_SPEED_MM_S +
                (LINE_STRAIGHT_SPEED_MM_S - LINE_CURVE_SPEED_MM_S) *
                    (1.5f - magnitude);
        return round_i16(speed);
    }
    return round_i16(LINE_STRAIGHT_SPEED_MM_S);
}

static void seek_request(uint32_t now_ms, MotionRequest *request)
{
    int16_t search_turn = (int16_t)(direction * LINE_MOTOR_TURN_SIGN *
                                    LINE_SEEK_COMMAND);

    request->left_speed = search_turn > 0 ? LINE_SEEK_COMMAND : 0;
    request->right_speed = search_turn > 0 ? 0 : LINE_SEEK_COMMAND;
    request->timestamp_ms = now_ms;
    request->valid = true;
    last_turn_command = search_turn;
    mode = direction < 0 ? LINE_FOLLOWER_SEEK_LEFT :
                           LINE_FOLLOWER_SEEK_RIGHT;
}

static void follow_request(const LinePositionResult *position,
                           float yaw_rate_dps,
                           float heading_error,
                           uint32_t now_ms,
                           MotionRequest *request)
{
    int16_t base = base_speed(position->confidence, predicted_error);
    int16_t turn = turn_from_state(predicted_error, error_rate,
                                   yaw_rate_dps, heading_error,
                                   heading_hold);

    request->left_speed = clamp_command((int32_t)base + turn);
    request->right_speed = clamp_command((int32_t)base - turn);
    request->timestamp_ms = now_ms;
    request->valid = true;
    last_follow = *request;
    last_follow_ms = now_ms;
    last_turn_command = turn;
}

void LineFollower_Init(void)
{
    mode = LINE_FOLLOWER_FOLLOW;
    direction = 1;
    reacquire_frames = 0U;
    last_follow = (MotionRequest){0};
    last_follow_ms = 0U;
    filtered_yaw_rate = 0.0f;
    last_turn_command = 0;
    heading_target_deg = 0.0f;
    heading_error_deg = 0.0f;
    centered_frames = 0U;
    heading_hold = false;
    reset_history();
}

bool LineFollower_Step(const AppLineSample *line,
                       const Mpu6050Snapshot *imu,
                       bool imu_fresh,
                       uint32_t now_ms,
                       MotionRequest *request,
                       LineFollowerStatus *status)
{
    float yaw_rate_dps = 0.0f;
    float yaw_angle_deg = 0.0f;
    int16_t yaw_correction = 0;
    int8_t position;
    bool centered;

    if (line == 0 || request == 0 || status == 0) {
        return false;
    }
    *request = (MotionRequest){0, 0, now_ms, false};

    if (imu_fresh && imu != 0) {
        float raw_yaw_rate = imu->gyro_rad_s[1] * RAD_TO_DEG;

        filtered_yaw_rate += LINE_YAW_FILTER_ALPHA *
                             (raw_yaw_rate - filtered_yaw_rate);
        yaw_rate_dps = filtered_yaw_rate;
        yaw_angle_deg = imu->yaw_angle_deg;
        yaw_correction = round_i16(-LINE_KYAW * yaw_rate_dps);
    } else {
        filtered_yaw_rate = 0.0f;
    }

    if (line->position.reliable) {
        push_history(line->position.weighted_error, now_ms);
        if (absolute_float(line->position.weighted_error) >=
            LINE_TREND_EPSILON) {
            direction = line->position.weighted_error < 0.0f ? -1 : 1;
        }
    }
    position = line->position.type == LINE_PATTERN_WIDE ?
                   line->position.candidate_position :
                   line->position.stable_position;
    centered = line->position.type == LINE_PATTERN_POSITION &&
               line->position.reliable &&
               absolute_float(predicted_error) < 0.4f &&
               absolute_float(error_rate) < 50.0f;
    if (centered) {
        if (centered_frames < 0xFFFFU) {
            centered_frames++;
        }
        if (!heading_hold && centered_frames >= 40U && imu_fresh &&
            imu != 0) {
            heading_target_deg = yaw_angle_deg;
            heading_hold = true;
        }
    } else {
        centered_frames = 0U;
        if (absolute_float(predicted_error) > 0.8f ||
            line->position.type != LINE_PATTERN_POSITION) {
            heading_hold = false;
        }
    }
    heading_error_deg = heading_hold && imu_fresh ?
                        wrap_angle(heading_target_deg - yaw_angle_deg) :
                        0.0f;

    if (line->position.type == LINE_PATTERN_NOISE) {
        if (last_follow.valid &&
            (uint32_t)(now_ms - last_follow_ms) <= LINE_NOISE_HOLD_MS) {
            *request = last_follow;
            request->timestamp_ms = now_ms;
        } else {
            seek_request(now_ms, request);
        }
    } else if (line->position.type == LINE_PATTERN_LOST) {
        reacquire_frames = 0U;
        seek_request(now_ms, request);
    } else if (mode != LINE_FOLLOWER_FOLLOW) {
        if (line->position.reliable) {
            reacquire_frames++;
        } else {
            reacquire_frames = 0U;
        }
        if (line->position.reliable &&
            reacquire_frames >= LINE_REACQUIRE_FRAMES) {
            mode = LINE_FOLLOWER_FOLLOW;
            follow_request(&line->position,
                           yaw_rate_dps, heading_error_deg,
                           now_ms, request);
        } else {
            seek_request(now_ms, request);
        }
    } else {
        follow_request(&line->position,
                       yaw_rate_dps, heading_error_deg,
                       now_ms, request);
    }

    status->mode = mode;
    status->imu_state = !imu_fresh ? LINE_IMU_OFF :
                        yaw_correction == 0 ? LINE_IMU_OK : LINE_IMU_USED;
    status->direction = direction;
    status->position = position;
    status->black_bits = line->position.black_bits;
    status->yaw_rate_dps = yaw_rate_dps;
    status->yaw_angle_deg = yaw_angle_deg;
    status->heading_error = heading_error_deg;
    status->heading_hold = heading_hold;
    status->error = filtered_error;
    status->predicted_error = predicted_error;
    status->trend = error_rate;
    status->turn_command = last_turn_command;
    status->imu_correction = yaw_correction;
    return request->valid;
}
