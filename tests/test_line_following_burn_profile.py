from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class LineFollowingBurnProfileContract(unittest.TestCase):
    def test_profile_matches_installed_hardware(self):
        profile = (ROOT / "config/line_following_profile.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "LINE_FOLLOWING_POWER_QUALIFIED (1)",
            "LINE_FOLLOWING_USE_ULTRASONIC (0)",
            "LINE_FOLLOWING_USE_IMU (1)",
            "LINE_FOLLOWING_REQUIRE_IMU (0)",
            "LINE_FOLLOWING_IMU_STARTUP_TIMEOUT_MS (2600U)",
            "LINE_FOLLOWING_IMU_DEGRADED_LIMIT (180)",
            "LINE_FOLLOWING_USE_VISION (0)",
        ):
            self.assertIn(token, profile)
        self.assertNotIn("LINE_KEY_TASK_PERIOD_MS", profile)

    def test_reset_automatically_starts_without_key_gate(self):
        run = (ROOT / "app/run/run_controller.c").read_text(encoding="utf-8")
        safety = (ROOT / "app/safety/safety_runtime.c").read_text(encoding="utf-8")
        self.assertIn("run_requested = true", run)
        self.assertIn("inputs.start_pressed = true", safety)
        self.assertIn("RunController_OnKeyEvent(Key_PollEvent())", safety)
        self.assertNotIn("KEY_EVENT_", safety)

    def test_bootstrap_defers_heartbeat_to_later_runtime(self):
        boot = (ROOT / "app/boot/app_boot.c").read_text(encoding="utf-8")
        safety = (ROOT / "app/safety/safety_runtime.c").read_text(encoding="utf-8")
        led_header = (ROOT / "modules/led/led.h").read_text(encoding="utf-8")
        led_source = (ROOT / "modules/led/led.c").read_text(encoding="utf-8")
        self.assertIn("LED_HeartbeatInit", led_header + led_source)
        self.assertIn("LED_HeartbeatService", led_header + led_source)
        self.assertIn("LED_HEARTBEAT_PERIOD_MS (250U)", led_source)
        self.assertIn("DL_GPIO_togglePins(LED_PORT, LED_D2_PIN)", led_source)
        self.assertNotIn("delay_", led_source)
        self.assertIn("LED_HeartbeatInit();", boot)
        self.assertIn("LED_HeartbeatService(now_ms)", safety)

    def test_tracking_timing_matches_pre_stop_go_baseline(self):
        config = (ROOT / "modules/optional/competition/line_tracking/line_control_config.h").read_text(
            encoding="utf-8"
        )
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        boot = (ROOT / "app/boot/app_boot.c").read_text(encoding="utf-8")
        self.assertIn("LINE_ESTIMATE_STALE_MS (20U)", config)
        self.assertNotIn("Buzzer_RequestBeeps", tasks)
        self.assertNotIn("PWM_Buzzer_Init();", boot)

    def test_line_output_reaches_the_single_motor_authority_path(self):
        control = (ROOT / "app/control/control_runtime.c").read_text(encoding="utf-8")
        safety = (ROOT / "app/safety/safety_runtime.c").read_text(encoding="utf-8")
        self.assertIn("AppLineMotion_BuildRequest", control)
        self.assertIn("AppMailbox_PublishMotionRequest", control)
        self.assertIn("SafetySupervisor_Step", safety)
        self.assertIn("MotorAdapter_Apply", safety)

    def test_ccs_build_excludes_unfitted_and_legacy_sources(self):
        cproject = (ROOT / ".cproject").read_text(encoding="utf-8")
        for excluded in (
            "application",
            "modules/optional/legacy",
            "modules/optional/competition",
            "modules/optional/ybimu",
            "modules/optional/k230",
            "modules/optional/ultrasonic",
        ):
            self.assertIn(excluded, cproject)
        self.assertNotIn("${PROJECT_ROOT}/application", cproject)

    def test_line_loss_timeout_stops_motion_without_disarming_run(self):
        line_motion = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        safety = (ROOT / "app/safety/safety_runtime.c").read_text(encoding="utf-8")
        self.assertIn("LineRecovery_Reset();", line_motion)
        self.assertIn("ClearRequest(now_ms, request)", line_motion)
        self.assertNotIn("Motor_Safety_Disarm", line_motion)
        self.assertNotIn("RunController_Init", line_motion)
        self.assertIn("RunController_BuildRequest", safety)


if __name__ == "__main__":
    unittest.main()

