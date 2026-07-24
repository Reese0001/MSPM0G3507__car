from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class MotionPrimitiveContract(unittest.TestCase):
    def test_trend_is_injected_through_scheduler_and_motion_context(self):
        header = (ROOT / "application/motion_primitives.h").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "application/motion_primitives.c").read_text(
            encoding="utf-8"
        )
        scheduler = (ROOT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("LineTrendResult line_trend", header)
        self.assertIn("&context->line_trend", source)
        self.assertIn("line_trend_detector.h", scheduler)
        self.assertIn("static LineTrendResult line_trend", scheduler)
        self.assertIn("LineTrendDetector_Init", scheduler)
        self.assertIn("LineTrendDetector_Reset", scheduler)
        self.assertLess(
            scheduler.index("LineTrendDetector_Update"),
            scheduler.index("LineController_Step"),
        )
        self.assertRegex(
            scheduler,
            r"LineController_Step\(\s*&line_estimate,\s*&line_trend,",
        )
        self.assertRegex(
            scheduler,
            r"LineRecovery_Step\(\s*&line_estimate,\s*&line_trend,",
        )

    def test_six_composable_primitives_and_injected_context_exist(self):
        header = (ROOT / "application/motion_primitives.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "MOTION_PRIMITIVE_FOLLOW_LINE",
            "MOTION_PRIMITIVE_DRIVE_DISTANCE",
            "MOTION_PRIMITIVE_TURN_RELATIVE",
            "MOTION_PRIMITIVE_SEARCH_LINE",
            "MOTION_PRIMITIVE_STOP_AT_MARKER",
            "MOTION_PRIMITIVE_WAIT_VISION",
            "MotionContext",
            "LineEstimate line",
            "LineTrendResult line_trend",
            "distance_mm",
            "yaw_deg",
            "vision_event",
            "odometry_fresh",
            "yaw_fresh",
            "vision_fresh",
            "MotionPrimitive_Start",
            "MotionPrimitive_StepWithContext",
            "MotionPrimitive_Cancel",
        ):
            self.assertIn(token, header)

    def test_primitives_are_nonblocking_and_motor_independent(self):
        source = (ROOT / "application/motion_primitives.c").read_text(
            encoding="utf-8"
        )
        for forbidden in (
            "delay_ms",
            "delay_us",
            "Contrl_Speed",
            "Motion_Car_Control",
            "Motor_Safety_Arm",
        ):
            self.assertNotIn(forbidden, source)
        self.assertIn("MotionRequest", source)
        self.assertIn("request->valid = false", source)

    def test_search_line_yaw_limit_is_owned_by_motion_primitives(self):
        source = (ROOT / "application/motion_primitives.c").read_text(
            encoding="utf-8"
        )
        recovery_config = (
            ROOT / "application/config/line_recovery_config.h"
        ).read_text(encoding="utf-8")
        primitive_config = (
            ROOT / "application/config/motion_primitives_config.h"
        ).read_text(encoding="utf-8")
        self.assertIn("MOTION_SEARCH_LINE_MAX_YAW_DEG (45.0f)", primitive_config)
        self.assertIn("MOTION_SEARCH_LINE_MAX_YAW_DEG", source)
        self.assertNotIn("LINE_RECOVERY_MAX_YAW_DEG", source + recovery_config)

    def test_active_scheduler_does_not_call_legacy_blocking_tracking(self):
        scheduler = (ROOT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("LineWalking", scheduler)
        self.assertNotIn("Question_Task_", scheduler)


if __name__ == "__main__":
    unittest.main()
