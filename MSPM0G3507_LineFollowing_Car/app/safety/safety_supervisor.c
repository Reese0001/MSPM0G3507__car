#include "safety_supervisor.h"

#include "config/safety_config.h"

static SafetySupervisorState supervisor_state = SAFETY_BOOT_SAFE;
static uint8_t clear_sample_count = 0U;
static uint16_t last_ultrasonic_sequence = 0U;
static uint16_t latched_reason = SAFETY_REASON_NONE;

static void reject(SafetyDecision *decision, uint16_t reason)
{
    decision->approved = false;
    decision->left_speed = 0;
    decision->right_speed = 0;
    decision->reason = reason;
    decision->state = supervisor_state;
}

static bool speed_within_limit(int16_t speed, int16_t limit)
{
    int32_t value = speed;
    if (value < 0) {
        value = -value;
    }
    return value <= limit;
}

static bool validate_request(const MotionRequest *request,
                             uint32_t now_ms,
                             int16_t speed_limit)
{
    return request != 0 && request->valid &&
           (uint32_t)(now_ms - request->timestamp_ms) <=
               MOTION_REQUEST_MAX_AGE_MS &&
           speed_within_limit(request->left_speed, speed_limit) &&
           speed_within_limit(request->right_speed, speed_limit);
}

void SafetySupervisor_Init(void)
{
    supervisor_state = SAFETY_BOOT_SAFE;
    clear_sample_count = 0U;
    last_ultrasonic_sequence = 0U;
    latched_reason = SAFETY_REASON_NONE;
}

void SafetySupervisor_Reinitialize(void)
{
    SafetySupervisor_Init();
}

SafetySupervisorState SafetySupervisor_GetState(void)
{
    return supervisor_state;
}

bool SafetySupervisor_Step(const SafetyInputs *inputs,
                           const MotionRequest *mission_request,
                           uint32_t now_ms,
                           SafetyDecision *decision)
{
    bool ultrasonic_fresh;
    bool new_ultrasonic_sample;
    int16_t speed_limit;

    if (decision == 0) {
        return false;
    }
    reject(decision, SAFETY_REASON_BOOT_GATE);
    if (inputs == 0) {
        supervisor_state = SAFETY_FAULT;
        latched_reason = SAFETY_REASON_SENSOR_STALE;
        reject(decision, SAFETY_REASON_SENSOR_STALE);
        return false;
    }

    if (inputs->motor_fault) {
        supervisor_state = SAFETY_FAULT;
        latched_reason = SAFETY_REASON_MOTOR_FAULT;
        reject(decision, SAFETY_REASON_MOTOR_FAULT);
        return false;
    }
    if (supervisor_state == SAFETY_FAULT) {
        reject(decision, latched_reason);
        return false;
    }
    if (!inputs->power_qualified) {
        if (supervisor_state != SAFETY_STOP_LATCHED) {
            supervisor_state = SAFETY_BOOT_SAFE;
        }
        reject(decision, SAFETY_REASON_POWER);
        return false;
    }

    ultrasonic_fresh = !inputs->ultrasonic_required ||
        ModuleStatus_IsFresh(
            &inputs->ultrasonic.status, now_ms, SAFETY_ULTRASONIC_STALE_MS);
    new_ultrasonic_sample = inputs->ultrasonic_required &&
        ultrasonic_fresh &&
        inputs->ultrasonic.status.sequence != last_ultrasonic_sequence;
    if (new_ultrasonic_sample) {
        last_ultrasonic_sequence = inputs->ultrasonic.status.sequence;
    }
    if (inputs->ultrasonic_required && !ultrasonic_fresh) {
        if (supervisor_state == SAFETY_RUNNING ||
            supervisor_state == SAFETY_LIMITED) {
            supervisor_state = SAFETY_FAULT;
            latched_reason = SAFETY_REASON_SENSOR_STALE;
        }
        clear_sample_count = 0U;
        reject(decision, SAFETY_REASON_SENSOR_STALE);
        return false;
    }

    if (inputs->ultrasonic_required &&
        inputs->ultrasonic.distance_mm <= SAFETY_OBSTACLE_STOP_MM) {
        supervisor_state = SAFETY_STOP_LATCHED;
        clear_sample_count = 0U;
        reject(decision, SAFETY_REASON_OBSTACLE);
        return false;
    }

    if (inputs->imu_required &&
        !ModuleStatus_IsFresh(&inputs->imu, now_ms,
                              SAFETY_IMU_STALE_MS)) {
        supervisor_state = SAFETY_FAULT;
        latched_reason = SAFETY_REASON_SENSOR_STALE;
        reject(decision, SAFETY_REASON_SENSOR_STALE);
        return false;
    }
    if (inputs->vision_required &&
        !ModuleStatus_IsFresh(&inputs->vision, now_ms,
                              SAFETY_VISION_STALE_MS)) {
        supervisor_state = SAFETY_FAULT;
        latched_reason = SAFETY_REASON_SENSOR_STALE;
        reject(decision, SAFETY_REASON_SENSOR_STALE);
        return false;
    }

    if (supervisor_state == SAFETY_STOP_LATCHED) {
        if (new_ultrasonic_sample &&
            inputs->ultrasonic.distance_mm > SAFETY_OBSTACLE_CLEAR_MM) {
            if (clear_sample_count < UINT8_MAX) {
                clear_sample_count++;
            }
        } else if (new_ultrasonic_sample) {
            clear_sample_count = 0U;
        }
        if (inputs->reset_pressed &&
            clear_sample_count >= SAFETY_CLEAR_SAMPLE_COUNT) {
            supervisor_state = SAFETY_READY;
            clear_sample_count = 0U;
        }
        reject(decision, SAFETY_REASON_OBSTACLE);
        return false;
    }

    if (supervisor_state == SAFETY_BOOT_SAFE) {
        supervisor_state = SAFETY_READY;
        reject(decision, SAFETY_REASON_BOOT_GATE);
        return false;
    }
    if (supervisor_state == SAFETY_READY) {
        if (inputs->start_pressed) {
            supervisor_state = SAFETY_RUNNING;
        }
        reject(decision, SAFETY_REASON_BOOT_GATE);
        return false;
    }

    if (inputs->ultrasonic_required &&
        inputs->ultrasonic.distance_mm <= SAFETY_OBSTACLE_LIMIT_MM) {
        supervisor_state = SAFETY_LIMITED;
        speed_limit = SAFETY_LIMITED_SPEED_LIMIT;
    } else {
        supervisor_state = SAFETY_RUNNING;
        speed_limit = SAFETY_RUNNING_SPEED_LIMIT;
    }

    if (!validate_request(mission_request, now_ms, speed_limit)) {
        reject(decision, SAFETY_REASON_REQUEST_INVALID);
        return false;
    }
    decision->approved = true;
    decision->left_speed = mission_request->left_speed;
    decision->right_speed = mission_request->right_speed;
    decision->reason = SAFETY_REASON_NONE;
    decision->state = supervisor_state;
    return true;
}
