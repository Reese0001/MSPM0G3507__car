from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class LineFollowingBurnProfileContract(unittest.TestCase):
    def test_profile_matches_installed_hardware(self):
        profile = (ROOT / "application/config/line_following_profile.h").read_text(
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

    def test_reset_automatically_starts_without_key_control(self):
        scheduler = (ROOT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )
        init = scheduler[scheduler.index("void AppScheduler_Init"):]
        init = init[:init.index("void AppScheduler_Run")]
        self.assertIn("AppScheduler_Start();", init)
        self.assertNotIn("Key_PollEvent", scheduler)
        self.assertNotIn("KEY_EVENT_", scheduler)
        self.assertNotIn("AppScheduler_RunKey", scheduler)

    def test_bootstrap_defers_heartbeat_to_a_later_task(self):
        app_main = (ROOT / "application/app_main.c").read_text(encoding="utf-8")
        led_header = (ROOT / "modules/led/led.h").read_text(encoding="utf-8")
        led_source = (ROOT / "modules/led/led.c").read_text(encoding="utf-8")
        self.assertIn("LED_HeartbeatInit", led_header + led_source)
        self.assertIn("LED_HeartbeatService", led_header + led_source)
        self.assertIn("LED_HEARTBEAT_PERIOD_MS (250U)", led_source)
        self.assertIn("DL_GPIO_togglePins(LED_PORT, LED_D2_PIN)", led_source)
        self.assertNotIn("delay_", led_source)
        self.assertIn("LED_HeartbeatInit();", app_main)
        self.assertNotIn("LED_HeartbeatService", app_main)

    def test_tracking_timing_matches_pre_stop_go_baseline(self):
        config = (ROOT / "application/config/line_control_config.h").read_text(
            encoding="utf-8"
        )
        scheduler = (ROOT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )
        app_main = (ROOT / "application/app_main.c").read_text(encoding="utf-8")
        self.assertIn("LINE_ESTIMATE_STALE_MS (20U)", config)
        self.assertNotIn("Buzzer_RequestBeeps", scheduler)
        self.assertNotIn("PWM_Buzzer_Init();", app_main)

    def test_line_output_reaches_the_single_motor_authority_path(self):
        scheduler = (ROOT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )
        positions = [
            scheduler.index("LineController_Step"),
            scheduler.index("LineRecovery_Step"),
            scheduler.index("SafetySupervisor_Step"),
            scheduler.index("MotorAdapter_Apply"),
        ]
        self.assertEqual(positions, sorted(positions))

    def test_ccs_build_excludes_unfitted_and_legacy_sources(self):
        cproject = (ROOT / ".cproject").read_text(encoding="utf-8")
        for excluded in (
            "application/legacy_questions",
            "application/legacy_task",
            "modules/legacy_mpu6050",
            "modules/ybimu",
            "modules/k230_link",
            "modules/ultrasonic",
        ):
            self.assertIn(excluded, cproject)

    def test_line_loss_timeout_stops_motion_without_disarming_run(self):
        scheduler = (ROOT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )
        line_control = scheduler[
            scheduler.index("static void AppScheduler_RunLineControl"):
            scheduler.index("static void AppScheduler_RunSafety")
        ]
        self.assertIn("LineRecovery_Reset();", line_control)
        self.assertIn("mission_request.valid = false;", line_control)
        self.assertNotIn("AppScheduler_Stop", line_control)
        self.assertNotIn("Motor_Safety_Disarm", line_control)


if __name__ == "__main__":
    unittest.main()
