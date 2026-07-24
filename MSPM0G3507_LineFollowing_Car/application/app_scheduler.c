#include "app_scheduler.h"

#include "config/line_control_config.h"
#include "config/line_following_profile.h"
#include "corner_maneuver.h"
#include "line_recovery.h"
#include "safety_supervisor.h"
#include "../bsp/time/timer.h"
#include "../modules/line_tracking/line_controller.h"
#include "../modules/line_tracking/line_estimator.h"
#include "../modules/line_tracking/line_event_classifier.h"
#include "../modules/line_tracking/line_features.h"
#include "../modules/line_tracking/line_scanner.h"
#include "../modules/line_tracking/line_trend_detector.h"
#include "../modules/led/led.h"
#include "../modules/motor/motor_adapter.h"
#include "../modules/motor/motor_safety.h"

static MotionRequest mission_request = {0};
static LineEstimate line_estimate = {0};
static LineFeatures line_features = {0};
static LineTrendResult line_trend = {0};
static LinePathEvent path_event = {0};
static LineControlOutput line_control = {0};
static CornerManeuverOutput corner_output = {0};
static bool start_requested = false;
static bool control_fault_latched = false;

static void AppScheduler_ResetLineControlHistory(void)
{
    LineFeatureExtractor_Reset();
    LineEventClassifier_Reset();
    LineTrendDetector_Reset();
    LineController_Reset();
    LineRecovery_Reset();
    CornerManeuver_Reset();
}

static void AppScheduler_Start(void)
{
    LineFeatureExtractor_Reset();
    LineEventClassifier_Reset();
    LineTrendDetector_Reset();
    LineController_Reset();
    LineRecovery_Reset();
    CornerManeuver_Reset();
    SafetySupervisor_Reinitialize();
    Motor_Safety_Arm();
    LED_OFF();
    start_requested = true;
    control_fault_latched = false;
}

static void AppScheduler_RunLineControl(uint32_t now_ms)
{
    LineSensorSnapshot scanner = {0};
    bool feature_ready = false;
    bool estimate_ready = false;
    bool trend_ready = false;
    bool event_ready = false;
    bool corner_fault =
        CornerManeuver_GetState() == CORNER_MANEUVER_FAULT;
    bool recovery_fault =
        LineRecovery_GetState() == LINE_RECOVERY_FAULT;

    if (LineScanner_GetSnapshot(&scanner)) {
        feature_ready = LineFeatureExtractor_Update(&scanner, now_ms,
                                                    &line_features);
        if (!feature_ready) {
            (void)LineEstimator_Update(0, now_ms);
        }
    }
    if (feature_ready) {
        estimate_ready = LineEstimator_Update(&line_features, now_ms) &&
                         LineEstimator_Get(&line_estimate);
    }
    if (estimate_ready) {
        trend_ready = LineTrendDetector_Update(
            &line_estimate, &scanner, now_ms, &line_trend);
    }
    if (trend_ready) {
        event_ready = LineEventClassifier_Update(
            &line_features, &line_estimate, &line_trend, now_ms,
            &path_event);
    }
    if (event_ready) {
        (void)LineController_Step(
            &line_estimate, &line_trend, 0.0F, false, now_ms, &line_control);
        (void)CornerManeuver_Step(
            &line_features, &path_event, &line_control, false,
            now_ms, &corner_output);
        if (corner_output.owns_motion) {
            mission_request = corner_output.request;
        } else {
            (void)LineRecovery_Step(
                &line_estimate, &line_trend, &line_control, 0.0F, false,
                false, now_ms, &mission_request);
        }
        corner_fault = corner_fault || corner_output.fault ||
            CornerManeuver_GetState() == CORNER_MANEUVER_FAULT;
        recovery_fault = recovery_fault ||
            LineRecovery_GetState() == LINE_RECOVERY_FAULT;
    } else {
        mission_request.left_speed = 0;
        mission_request.right_speed = 0;
        mission_request.valid = false;
        mission_request.timestamp_ms = now_ms;
    }

    if (corner_output.completed) {
        LineFeatureExtractor_Reset();
        LineEventClassifier_Reset();
        LineTrendDetector_Reset();
        LineController_Reset();
        LineRecovery_Reset();
        CornerManeuver_Reset();
        corner_output = (CornerManeuverOutput){0};
    }
    if (corner_fault || recovery_fault) {
        control_fault_latched = true;
        AppScheduler_ResetLineControlHistory();
        corner_output = (CornerManeuverOutput){0};
    }
    if (control_fault_latched) {
        mission_request.left_speed = 0;
        mission_request.right_speed = 0;
        mission_request.timestamp_ms = now_ms;
        mission_request.valid = false;
        LED_ON();
    }
}

static void AppScheduler_RunSafety(uint32_t now_ms)
{
    SafetyInputs inputs = {0};
    SafetyDecision decision = {0};

    inputs.ultrasonic_required = LINE_FOLLOWING_USE_ULTRASONIC != 0;
    inputs.imu_required = LINE_FOLLOWING_USE_IMU != 0;
    inputs.vision_required = LINE_FOLLOWING_USE_VISION != 0;
    inputs.start_pressed = start_requested;
    inputs.reset_pressed = false;
    inputs.power_qualified = LINE_FOLLOWING_POWER_QUALIFIED != 0;
    inputs.motor_fault = Motor_Safety_IsFaultLatched() != 0U;

    (void)SafetySupervisor_Step(
        &inputs, &mission_request, now_ms, &decision);
    MotorAdapter_Apply(&decision);

    if (SafetySupervisor_GetState() == SAFETY_RUNNING ||
        SafetySupervisor_GetState() == SAFETY_LIMITED) {
        start_requested = false;
    }
}

static AppTask app_tasks[] = {
    {1U, 200U, 0U, 0U, 0U, AppScheduler_RunSafety},
    {5U, 500U, 0U, 0U, 0U, AppScheduler_RunLineControl},
};

void AppScheduler_Init(uint32_t now_ms)
{
    uint32_t index;
    LineControlConfig line_config = LineControlConfig_Default();

    LineScanner_Init();
    LineFeatureExtractor_Init();
    LineEstimator_Init();
    LineTrendDetector_Init();
    LineEventClassifier_Init();
    (void)LineController_Init(&line_config);
    LineRecovery_Init();
    CornerManeuver_Init();
    SafetySupervisor_Init();

    mission_request = (MotionRequest){0};
    line_estimate = (LineEstimate){0};
    line_features = (LineFeatures){0};
    line_trend = (LineTrendResult){0};
    path_event = (LinePathEvent){0};
    line_control = (LineControlOutput){0};
    corner_output = (CornerManeuverOutput){0};
    start_requested = false;
    control_fault_latched = false;

    for (index = 0U;
         index < (uint32_t)(sizeof(app_tasks) / sizeof(app_tasks[0]));
         index++) {
        app_tasks[index].last_ms = now_ms;
        app_tasks[index].max_runtime_us = 0U;
        app_tasks[index].deadline_miss_count = 0U;
    }

    /* Reset is the only start command; safety supervision still gates output. */
    AppScheduler_Start();
}

void AppScheduler_Run(uint32_t now_ms)
{
    uint32_t index;
    uint32_t now_us = BSP_Time_GetUs();

    LineScanner_Service(now_us);

    for (index = 0U;
         index < (uint32_t)(sizeof(app_tasks) / sizeof(app_tasks[0]));
         index++) {
        AppTask *task = &app_tasks[index];
        uint32_t elapsed_ms = (uint32_t)(now_ms - task->last_ms);

        if (elapsed_ms >= task->period_ms) {
            uint32_t started_us;
            uint32_t runtime_us;

            if (elapsed_ms >= (uint32_t)task->period_ms * 2U) {
                task->deadline_miss_count +=
                    elapsed_ms / task->period_ms - 1U;
            }
            started_us = BSP_Time_GetUs();
            task->run(now_ms);
            runtime_us = (uint32_t)(BSP_Time_GetUs() - started_us);
            if (runtime_us > task->max_runtime_us) {
                task->max_runtime_us = runtime_us;
            }
            if (runtime_us > task->budget_us) {
                task->deadline_miss_count++;
            }
            task->last_ms = now_ms;
        }
    }
}

bool AppScheduler_GetDiagnostics(uint32_t index, AppTaskDiagnostics *out)
{
    const uint32_t task_count =
        (uint32_t)(sizeof(app_tasks) / sizeof(app_tasks[0]));

    if (out == 0 || index >= task_count) {
        return false;
    }
    out->period_ms = app_tasks[index].period_ms;
    out->budget_us = app_tasks[index].budget_us;
    out->max_runtime_us = app_tasks[index].max_runtime_us;
    out->deadline_miss_count = app_tasks[index].deadline_miss_count;
    return true;
}
