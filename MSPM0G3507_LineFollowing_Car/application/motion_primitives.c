#include "motion_primitives.h"

#include "config/line_control_config.h"
#include "config/line_recovery_config.h"
#include "config/motion_primitives_config.h"
#include "line_recovery.h"

static MotionPrimitiveType active_type = MOTION_PRIMITIVE_FOLLOW_LINE;
static MotionPrimitiveParams active_params = {0};
static uint32_t primitive_started_ms = 0U;
static int32_t start_distance_mm = 0;
static float start_yaw_deg = 0.0f;
static bool reference_captured = false;
static bool primitive_active = false;

static float absolute_float(float value)
{
    return value < 0.0f ? -value : value;
}

static uint64_t distance_magnitude(int64_t value)
{
    return value < 0 ? (uint64_t)-value : (uint64_t)value;
}

static float relative_yaw(float current, float start)
{
    float delta = current - start;
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }
    return delta;
}

static void stop_request(uint32_t now_ms, MotionRequest *request)
{
    request->left_speed = 0;
    request->right_speed = 0;
    request->timestamp_ms = now_ms;
    request->valid = false;
}

static MotionResult finish_primitive(MotionResult result,
                                     uint32_t now_ms,
                                     MotionRequest *request)
{
    primitive_active = false;
    stop_request(now_ms, request);
    return result;
}

static uint32_t active_timeout_ms(void)
{
    switch (active_type) {
        case MOTION_PRIMITIVE_FOLLOW_LINE:
            return active_params.follow_line.timeout_ms;
        case MOTION_PRIMITIVE_DRIVE_DISTANCE:
            return active_params.drive_distance.timeout_ms;
        case MOTION_PRIMITIVE_TURN_RELATIVE:
            return active_params.turn_relative.timeout_ms;
        case MOTION_PRIMITIVE_SEARCH_LINE:
            return active_params.search_line.timeout_ms;
        case MOTION_PRIMITIVE_STOP_AT_MARKER:
            return active_params.stop_at_marker.timeout_ms;
        case MOTION_PRIMITIVE_WAIT_VISION:
            return active_params.wait_vision.timeout_ms;
        default:
            return 0U;
    }
}

static bool step_follow(uint32_t now_ms, const MotionContext *context,
                        MotionRequest *request)
{
    LineControlOutput follow;

    (void)LineController_Step(
        &context->line, &context->line_trend, context->yaw_rate_dps,
        context->yaw_fresh, now_ms, &follow);
    return LineRecovery_Step(
        &context->line, &context->line_trend, &follow, context->yaw_deg,
        context->yaw_fresh, context->emergency_stop, now_ms, request);
}

bool MotionPrimitive_Init(void)
{
    LineControlConfig config = LineControlConfig_Default();

    primitive_active = false;
    reference_captured = false;
    LineRecovery_Init();
    return LineController_Init(&config);
}

bool MotionPrimitive_Start(MotionPrimitiveType type,
                           const MotionPrimitiveParams *params,
                           uint32_t now_ms)
{
    if (params == 0 || type > MOTION_PRIMITIVE_WAIT_VISION) {
        return false;
    }
    if ((type == MOTION_PRIMITIVE_DRIVE_DISTANCE &&
         (params->drive_distance.distance_mm == 0 ||
          params->drive_distance.speed <= 0 ||
          params->drive_distance.speed > 300 ||
          params->drive_distance.timeout_ms == 0U)) ||
        (type == MOTION_PRIMITIVE_TURN_RELATIVE &&
         (absolute_float(params->turn_relative.angle_deg) < 1.0f ||
          absolute_float(params->turn_relative.angle_deg) > 180.0f ||
          params->turn_relative.timeout_ms == 0U)) ||
        (type == MOTION_PRIMITIVE_SEARCH_LINE &&
         params->search_line.timeout_ms == 0U) ||
        (type == MOTION_PRIMITIVE_STOP_AT_MARKER &&
         params->stop_at_marker.timeout_ms == 0U) ||
        (type == MOTION_PRIMITIVE_WAIT_VISION &&
         (params->wait_vision.event_id < 1U ||
          params->wait_vision.event_id > 17U ||
          params->wait_vision.timeout_ms == 0U))) {
        return false;
    }
    active_type = type;
    active_params = *params;
    primitive_started_ms = now_ms;
    reference_captured = false;
    primitive_active = true;
    LineController_Reset();
    LineRecovery_Reset();
    return true;
}

MotionResult MotionPrimitive_StepWithContext(uint32_t now_ms,
                                             const MotionContext *context,
                                             MotionRequest *request)
{
    uint32_t timeout_ms;

    if (request == 0) {
        return MOTION_FAILED;
    }
    stop_request(now_ms, request);
    if (!primitive_active || context == 0 || context->emergency_stop) {
        return finish_primitive(MOTION_FAILED, now_ms, request);
    }
    timeout_ms = active_timeout_ms();
    if (timeout_ms != 0U &&
        (uint32_t)(now_ms - primitive_started_ms) >= timeout_ms) {
        return finish_primitive(MOTION_FAILED, now_ms, request);
    }

    if (active_type == MOTION_PRIMITIVE_FOLLOW_LINE) {
        if (!step_follow(now_ms, context, request) &&
            LineRecovery_GetState() == LINE_RECOVERY_FAULT) {
            return finish_primitive(MOTION_FAILED, now_ms, request);
        }
        return MOTION_RUNNING;
    }

    if (active_type == MOTION_PRIMITIVE_DRIVE_DISTANCE) {
        int16_t command;
        if (!context->odometry_fresh) {
            return finish_primitive(MOTION_FAILED, now_ms, request);
        }
        if (!reference_captured) {
            start_distance_mm = context->distance_mm;
            reference_captured = true;
        }
        if (distance_magnitude((int64_t)context->distance_mm -
                               (int64_t)start_distance_mm) >=
            distance_magnitude(active_params.drive_distance.distance_mm)) {
            return finish_primitive(MOTION_COMPLETE, now_ms, request);
        }
        command = active_params.drive_distance.speed;
        if (active_params.drive_distance.distance_mm < 0) {
            command = (int16_t)-command;
        }
        request->left_speed = command;
        request->right_speed = command;
        request->timestamp_ms = now_ms;
        request->valid = true;
        return MOTION_RUNNING;
    }

    if (active_type == MOTION_PRIMITIVE_TURN_RELATIVE) {
        float progress;
        float target = active_params.turn_relative.angle_deg;
        if (!context->yaw_fresh) {
            return finish_primitive(MOTION_FAILED, now_ms, request);
        }
        if (!reference_captured) {
            start_yaw_deg = context->yaw_deg;
            reference_captured = true;
        }
        progress = relative_yaw(context->yaw_deg, start_yaw_deg);
        if ((target > 0.0f && progress >= target) ||
            (target < 0.0f && progress <= target)) {
            return finish_primitive(MOTION_COMPLETE, now_ms, request);
        }
        if (target < 0.0f) {
            request->left_speed = LINE_SEARCH_INNER_COMMAND;
            request->right_speed = LINE_SEARCH_OUTER_COMMAND;
        } else {
            request->left_speed = LINE_SEARCH_OUTER_COMMAND;
            request->right_speed = LINE_SEARCH_INNER_COMMAND;
        }
        request->timestamp_ms = now_ms;
        request->valid = true;
        return MOTION_RUNNING;
    }

    if (active_type == MOTION_PRIMITIVE_SEARCH_LINE) {
        if (!context->yaw_fresh) {
            return finish_primitive(MOTION_FAILED, now_ms, request);
        }
        if (ModuleStatus_IsFresh(&context->line.status, now_ms,
                                 LINE_ESTIMATE_STALE_MS) &&
            context->line.event != LINE_EVENT_LOST &&
            context->line.confidence >= 40U) {
            return finish_primitive(MOTION_COMPLETE, now_ms, request);
        }
        if (!reference_captured) {
            start_yaw_deg = context->yaw_deg;
            reference_captured = true;
        }
        if (absolute_float(relative_yaw(context->yaw_deg, start_yaw_deg)) >=
            MOTION_SEARCH_LINE_MAX_YAW_DEG) {
            return finish_primitive(MOTION_FAILED, now_ms, request);
        }
        if (active_params.search_line.prefer_left) {
            request->left_speed = LINE_SEARCH_INNER_COMMAND;
            request->right_speed = LINE_SEARCH_OUTER_COMMAND;
        } else {
            request->left_speed = LINE_SEARCH_OUTER_COMMAND;
            request->right_speed = LINE_SEARCH_INNER_COMMAND;
        }
        request->timestamp_ms = now_ms;
        request->valid = true;
        return MOTION_RUNNING;
    }

    if (active_type == MOTION_PRIMITIVE_STOP_AT_MARKER) {
        if (ModuleStatus_IsFresh(&context->line.status, now_ms,
                                 LINE_ESTIMATE_STALE_MS) &&
            context->line.event == LINE_EVENT_WIDE_BLACK) {
            return finish_primitive(MOTION_COMPLETE, now_ms, request);
        }
        if (!step_follow(now_ms, context, request) &&
            LineRecovery_GetState() == LINE_RECOVERY_FAULT) {
            return finish_primitive(MOTION_FAILED, now_ms, request);
        }
        return MOTION_RUNNING;
    }

    if (active_type == MOTION_PRIMITIVE_WAIT_VISION) {
        if (context->vision_fresh &&
            context->vision_event == active_params.wait_vision.event_id) {
            return finish_primitive(MOTION_COMPLETE, now_ms, request);
        }
        return MOTION_RUNNING;
    }

    return finish_primitive(MOTION_FAILED, now_ms, request);
}

void MotionPrimitive_Cancel(void)
{
    primitive_active = false;
    reference_captured = false;
    LineController_Reset();
    LineRecovery_Reset();
}
