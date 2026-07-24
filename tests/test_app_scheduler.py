from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class AppSchedulerPipelineContract(unittest.TestCase):
    def setUp(self):
        self.source = (ROOT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )

    def test_line_pipeline_order_and_motion_ownership(self):
        calls = [
            "LineFeatureExtractor_Update",
            "LineEstimator_Update",
            "LineTrendDetector_Update",
            "LineEventClassifier_Update",
            "LineController_Step",
            "CornerManeuver_Step",
            "LineRecovery_Step",
        ]
        positions = [self.source.index(call) for call in calls]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("corner_output.owns_motion", self.source)
        self.assertIn("mission_request = corner_output.request", self.source)

    def test_all_new_modules_reset_at_start_and_corner_completion(self):
        for token in (
            "LineFeatureExtractor_Reset",
            "LineEventClassifier_Reset",
            "CornerManeuver_Reset",
            "LineTrendDetector_Reset",
            "LineController_Reset",
            "LineRecovery_Reset",
        ):
            self.assertGreaterEqual(self.source.count(token), 2)

    def test_corner_or_recovery_fault_fails_closed(self):
        self.assertIn("corner_output.fault", self.source)
        self.assertIn(
            "LineRecovery_GetState() == LINE_RECOVERY_FAULT", self.source
        )
        self.assertRegex(
            self.source,
            r"mission_request\.left_speed\s*=\s*0;[\s\S]{0,120}"
            r"mission_request\.right_speed\s*=\s*0;",
        )
        self.assertIn("LED_ON()", self.source)
        self.assertIn("LED_OFF()", self.source)

    def test_latched_faults_are_checked_outside_ready_pipeline(self):
        self.assertIn(
            "CornerManeuver_GetState() == CORNER_MANEUVER_FAULT",
            self.source,
        )

    def test_pipeline_fault_remains_fail_closed_until_start_reset(self):
        self.assertIn("static bool control_fault_latched = false", self.source)
        self.assertRegex(
            self.source,
            r"if\s*\(corner_fault\s*\|\|\s*recovery_fault\)\s*\{"
            r"\s*control_fault_latched\s*=\s*true;",
        )
        self.assertRegex(
            self.source,
            r"if\s*\(control_fault_latched\)\s*\{[\s\S]{0,240}"
            r"mission_request\.valid\s*=\s*false;[\s\S]{0,120}LED_ON\(\);",
        )
        self.assertRegex(
            self.source,
            r"static void AppScheduler_Start\(void\)[\s\S]{0,700}"
            r"control_fault_latched\s*=\s*false;",
        )

    def test_integration_keeps_confirmed_pd_and_speed_limits(self):
        config = (
            ROOT / "application/config/line_control_config.h"
        ).read_text(encoding="utf-8")
        safety = (
            ROOT / "application/config/safety_config.h"
        ).read_text(encoding="utf-8")
        self.assertIn("LINE_CONTROL_KP (28.0f)", config)
        self.assertIn("LINE_MAX_FORWARD (400)", config)
        self.assertIn("SAFETY_RUNNING_SPEED_LIMIT (450)", safety)


if __name__ == "__main__":
    unittest.main()
