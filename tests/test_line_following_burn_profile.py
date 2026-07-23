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
            "LINE_FOLLOWING_USE_IMU (0)",
            "LINE_FOLLOWING_USE_VISION (0)",
            "LINE_KEY_TASK_PERIOD_MS (10U)",
        ):
            self.assertIn(token, profile)

    def test_key_reports_debounced_press_for_immediate_stop(self):
        header = (ROOT / "modules/key/key.h").read_text(encoding="utf-8")
        source = (ROOT / "modules/key/key.c").read_text(encoding="utf-8")
        self.assertIn("KEY_EVENT_PRESS", header)
        self.assertIn("return KEY_EVENT_PRESS", source)

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


if __name__ == "__main__":
    unittest.main()
