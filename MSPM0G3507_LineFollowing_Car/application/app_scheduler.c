#include "app_scheduler.h"

#include "app_motor.h"
#include "config/line_control_config.h"
#include "questions.h"
#include "safety_supervisor.h"
#include "../bsp/time/timer.h"
#include "../modules/k230_link/k230_link.h"
#include "../modules/line_tracking/line_controller.h"
#include "../modules/line_tracking/line_estimator.h"
#include "../modules/line_tracking/line_scanner.h"
#include "../modules/motor/motor_adapter.h"
#include "../modules/ultrasonic/ultrasonic.h"
#include "../modules/ybimu/ybimu.h"

static MotionRequest mission_request = {0};
static LineEstimate line_estimate = {0};
static LineControlOutput line_control = {0};

static void AppScheduler_RunSafety(uint32_t now_ms)
{
    SafetyInputs inputs = {0};
    SafetyDecision decision = {0};

    (void)Ultrasonic_GetSnapshot(&inputs.ultrasonic);
    (void)YbImu_GetSnapshot(&inputs.imu);
    (void)K230Link_GetSnapshot(now_ms, &inputs.vision);

    inputs.imu_required = false;
    inputs.vision_required = false;
    inputs.start_pressed = false;
    inputs.reset_pressed = false;
    inputs.power_qualified = false;
    inputs.motor_fault = false;

    (void)SafetySupervisor_Step(
        &inputs, &mission_request, now_ms, &decision);
    MotorAdapter_Apply(&decision);
}

static void AppScheduler_RunLineControl(uint32_t now_ms)
{
    LineSensorSnapshot scanner = {0};
    YbImuSnapshot imu = {0};
    bool yaw_fresh = YbImu_GetSnapshot(&imu);
    const float radians_to_degrees = 57.2957795F;

    if (LineScanner_GetSnapshot(&scanner)) {
        (void)LineEstimator_Update(&scanner, now_ms);
    }
    if (LineEstimator_Get(&line_estimate)) {
        (void)LineController_Step(
            &line_estimate,
            yaw_fresh ? imu.gyro_rad_s[2] * radians_to_degrees : 0.0F,
            yaw_fresh,
            now_ms,
            &line_control);
    }
}

static void AppScheduler_RunYbImu(uint32_t now_ms)
{
    YbImu_Service(now_ms);
}

static void AppScheduler_RunK230(uint32_t now_ms)
{
    K230Link_Service(now_ms);
}

static void AppScheduler_RunKey(uint32_t now_ms)
{
    (void)now_ms;
    Legacy_Questions_HandleKey();
}

static void AppScheduler_RunOdometry(uint32_t now_ms)
{
    (void)now_ms;
    Get_Odometry();
}

static AppTask app_tasks[] = {
    {1U, 200U, 0U, 0U, 0U, AppScheduler_RunSafety},
    {1U, 300U, 0U, 0U, 0U, AppScheduler_RunK230},
    {5U, 500U, 0U, 0U, 0U, AppScheduler_RunLineControl},
    {10U, 500U, 0U, 0U, 0U, AppScheduler_RunYbImu},
    {100U, 200U, 0U, 0U, 0U, AppScheduler_RunKey},
    {15U, 200U, 0U, 0U, 0U, AppScheduler_RunOdometry},
};

void AppScheduler_Init(uint32_t now_ms)
{
    uint32_t index;
    LineControlConfig line_config = LineControlConfig_Default();

    LineScanner_Init();
    LineEstimator_Init();
    (void)LineController_Init(&line_config);
    Ultrasonic_Init();
    YbImu_Init(now_ms);
    K230Link_Init();
    SafetySupervisor_Init();

    mission_request = (MotionRequest){0};
    line_estimate = (LineEstimate){0};
    line_control = (LineControlOutput){0};

    for (index = 0U; index < (uint32_t)(sizeof(app_tasks) / sizeof(app_tasks[0])); index++) {
        app_tasks[index].last_ms = now_ms;
        app_tasks[index].max_runtime_us = 0U;
        app_tasks[index].deadline_miss_count = 0U;
    }
}

void AppScheduler_Run(uint32_t now_ms)
{
    uint32_t index;
    uint32_t now_us = BSP_Time_GetUs();

    /* Fast cooperative services contain no blocking waits. */
    LineScanner_Service(now_us);
    Ultrasonic_Service(now_us);

    for (index = 0U; index < (uint32_t)(sizeof(app_tasks) / sizeof(app_tasks[0])); index++) {
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
