#include "app_tasks.h"

#include <stdbool.h>

#include "../boot/app_boot.h"
#include "../../bsp/bsp_i2c.h"
#include "../../modules/diagnostics/boot_trace.h"
#include "../../modules/display/dashboard.h"
#include "../../modules/display/ssd1306/ssd1306.h"
#include "../../modules/key/key.h"
#include "../../modules/led/led.h"
#include "../../modules/line_tracking/decoder/line_position.h"
#include "../../modules/line_tracking/line_follower.h"
#include "../../modules/line_tracking/lap_tracker.h"
#include "../../modules/line_tracking/line_tracking_config.h"
#include "../../modules/line_tracking/scanner/four_line_scanner.h"
#include "../../modules/motor/drive.h"
#include "../../modules/motor/feedback/motor_feedback.h"
#include "../../modules/motor/feedback/stop_controller.h"
#include "../../modules/motor/uart/motor_uart.h"
#include "../../modules/imu/mpu6050.h"
#include "../../modules/time/timer.h"
#include "../../modules/wifi/wifi_uart.h"
#include "../../shared/module_status.h"

#define DISPLAY_PERIOD_MS  (200U)
#define DISPLAY_FLUSH_PERIOD_MS (2U)

static uint32_t last_scanner_ms;
static uint32_t last_display_ms;
static uint32_t last_flush_ms;
static uint16_t last_scanner_sequence;
static uint32_t last_line_frame_ms;
static bool run_started;
static bool lap_finished;
static StopController stop_controller;
static bool stop_controller_started;
static StopControllerStatus stop_status;
static bool display_ready;
static AppDiagnostics diagnostics;

static bool due(uint32_t now_ms, uint32_t last_ms, uint32_t period_ms)
{
    return (uint32_t)(now_ms - last_ms) >= period_ms;
}

static bool read_imu(uint32_t now_ms, Mpu6050Snapshot *imu)
{
    return Mpu6050_GetSnapshot(imu) &&
           imu->status.health == MODULE_HEALTH_OK &&
           ModuleStatus_IsFresh(&imu->status, now_ms,
                                MPU6050_STALE_TIMEOUT_MS);
}

static void poll_key(uint32_t now_ms)
{
    KeyEvent event = Key_PollEvent();

    if (lap_finished && event == KEY_EVENT_LONG) {
        run_started = false;
        lap_finished = false;
        LapTracker_Reset();
        LineFollower_Init();
        StopController_Init(&stop_controller);
        stop_controller_started = false;
        stop_status = (StopControllerStatus){0};
        Mpu6050_ResetYawReference();
        MotorFeedback_ResetDistance();
        diagnostics.request = (MotionRequest){0, 0, now_ms, false};
        Drive_SetTarget(&diagnostics.request);
        return;
    }
    if (!run_started && AppBoot_IsMotorConfigured() &&
        event == KEY_EVENT_PRESS) {
        run_started = true;
        lap_finished = false;
        LapTracker_Start(now_ms);
        StopController_Init(&stop_controller);
        stop_controller_started = false;
        stop_status = (StopControllerStatus){0};
        Mpu6050_ResetYawReference();
        MotorFeedback_ResetDistance();
        Drive_Start();
    }
}

static void sample_and_control(uint32_t now_ms)
{
    LineSensorSnapshot raw;
    AppLineSample line;
    Mpu6050Snapshot imu = {0};
    bool imu_fresh;

    if (!FourLineScanner_GetSnapshot(&raw) ||
        raw.status.sequence == last_scanner_sequence) {
        return;
    }
    last_scanner_sequence = raw.status.sequence;
    last_line_frame_ms = raw.status.timestamp_ms;
    line.position = LinePosition_Update(raw.black_bits);
    line.sequence = raw.status.sequence;
    line.timestamp_ms = raw.status.timestamp_ms;
    diagnostics.raw_x_bits = raw.black_bits;
    diagnostics.line.black_bits = raw.black_bits;
    diagnostics.line.position = line.position.stable_position;
    if (!run_started || lap_finished) {
        return;
    }
    if (LapTracker_Update(raw.black_bits,
                          (uint32_t)MotorFeedback_GetDistanceMm(),
                          now_ms)) {
        StopController_Start(&stop_controller,
                             LapTracker_GetStopTargetMm(), now_ms);
        stop_controller_started = true;
    }
    imu_fresh = read_imu(now_ms, &imu);
    if (LineFollower_Step(&line, &imu, imu_fresh, now_ms,
                          &diagnostics.request, &diagnostics.line)) {
        if (stop_controller_started) {
            MotorFeedbackSnapshot feedback;
            float speed = 0.0f;

            if (MotorFeedback_GetSnapshot(&feedback, now_ms)) {
                speed = (feedback.left_speed_mm_s +
                         feedback.right_speed_mm_s) * 0.5f;
            } else {
                speed = (float)(diagnostics.request.left_speed +
                                diagnostics.request.right_speed) * 0.5f;
            }
            StopController_Update(&stop_controller,
                                  (uint32_t)MotorFeedback_GetDistanceMm(),
                                  speed, now_ms, &stop_status);
            diagnostics.request.left_speed =
                (int16_t)stop_status.speed_command_mm_s;
            diagnostics.request.right_speed =
                (int16_t)stop_status.speed_command_mm_s;
            diagnostics.request.timestamp_ms = now_ms;
            diagnostics.request.valid = !stop_status.done;
            if (stop_status.done) {
                stop_controller_started = false;
                LapTracker_NotifyStopped(now_ms);
                lap_finished = true;
            }
        }
        Drive_SetTarget(&diagnostics.request);
    }
}

static void refresh_fresh_motion(uint32_t now_ms)
{
    MotionRequest held;

    if (!run_started || lap_finished || !diagnostics.request.valid ||
        (uint32_t)(now_ms - last_line_frame_ms) > FOUR_LINE_STALE_MS) {
        return;
    }
    held = diagnostics.request;
    held.timestamp_ms = now_ms;
    Drive_SetTarget(&held);
}

static void render_dashboard(void)
{
    diagnostics.run_started = run_started;
    diagnostics.motor_ready = AppBoot_IsMotorConfigured();
    diagnostics.wifi_state = WifiUart_GetProbeState();
    diagnostics.timestamp_ms = Get_Time();
    (void)MotorFeedback_GetSnapshot(&diagnostics.feedback, Get_Time());
    diagnostics.distance_mm = MotorFeedback_GetDistanceMm();
    diagnostics.stop = stop_status;
    LapTracker_GetStatus(Get_Time(), &diagnostics.lap);
    Drive_GetStatus(&diagnostics.drive);
    Dashboard_Render(&diagnostics);
}

void AppTasks_Init(void)
{
    uint32_t now_ms = Get_Time();

    WifiUart_Init(now_ms);
    FourLineScanner_Init();
    LinePosition_Reset();
    LineFollower_Init();
    LapTracker_Init();
    MotorFeedback_Init();
    Mpu6050_Init(now_ms);
    diagnostics = (AppDiagnostics){0};
    diagnostics.line.imu_state = LINE_IMU_OFF;
    last_scanner_sequence = 0U;
    last_line_frame_ms = now_ms;
    run_started = false;
    lap_finished = false;
    StopController_Init(&stop_controller);
    stop_controller_started = false;
    stop_status = (StopControllerStatus){0};
    display_ready = AppBoot_IsDisplayReady();
    last_scanner_ms = now_ms - FOUR_LINE_SAMPLE_PERIOD_MS;
    last_display_ms = now_ms - DISPLAY_PERIOD_MS;
    last_flush_ms = now_ms;
    BootTrace_TaskOnline(BOOT_TASK_SAFETY);
    BootTrace_TaskOnline(BOOT_TASK_SENSOR);
    BootTrace_TaskOnline(BOOT_TASK_CONTROL);
    BootTrace_TaskOnline(BOOT_TASK_DISPLAY);
}

void AppTasks_Poll(uint32_t now_ms)
{
    BSP_I2C_Service(BSP_Time_GetUs());
    Motor_Usart_Service(now_ms);
    MotorFeedback_SetCommandSpeed(
        diagnostics.request.valid ?
            (float)(diagnostics.request.left_speed +
                    diagnostics.request.right_speed) * 0.5f : 0.0f);
    MotorFeedback_UpdateOdometry(now_ms);
    Mpu6050_Service(now_ms);
    WifiUart_Service(now_ms);
    poll_key(now_ms);
    if (due(now_ms, last_scanner_ms, FOUR_LINE_SAMPLE_PERIOD_MS)) {
        last_scanner_ms = now_ms;
        FourLineScanner_Sample(now_ms);
        sample_and_control(now_ms);
    }
    refresh_fresh_motion(now_ms);
    Drive_Service(now_ms);
    LED_HeartbeatService(now_ms);
    if (display_ready && due(now_ms, last_display_ms, DISPLAY_PERIOD_MS)) {
        last_display_ms = now_ms;
        render_dashboard();
    }
    if (display_ready &&
        due(now_ms, last_flush_ms, DISPLAY_FLUSH_PERIOD_MS)) {
        last_flush_ms = now_ms;
        display_ready = Ssd1306_FlushNextChunk();
    }
}
