from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"
OPTIONAL = ROOT / "modules/optional/competition"


class MotionPrimitiveContract(unittest.TestCase):
    def test_optional_primitives_keep_composable_context_interface(self):
        header = (OPTIONAL / "motion_primitives.h").read_text(encoding="utf-8")
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

    def test_optional_primitives_are_nonblocking_and_motor_independent(self):
        source = (OPTIONAL / "motion_primitives.c").read_text(encoding="utf-8")
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

    def test_search_line_yaw_limit_is_owned_by_optional_primitives(self):
        source = (OPTIONAL / "motion_primitives.c").read_text(encoding="utf-8")
        primitive_config = (ROOT / "config/motion_primitives_config.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("MOTION_SEARCH_LINE_MAX_YAW_DEG (45.0f)", primitive_config)
        self.assertIn("MOTION_SEARCH_LINE_MAX_YAW_DEG", source)
        self.assertNotIn("LINE_RECOVERY_MAX_YAW_DEG", source)

    def test_active_tasks_do_not_call_legacy_blocking_tracking_or_primitives(self):
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        line_motion = (ROOT / "modules/line_tracking/line_follower.c").read_text(encoding="utf-8")
        active = tasks + line_motion
        for forbidden in (
            "LineWalking",
            "Question_Task_",
            "MotionPrimitive_",
            "CornerManeuver_",
        ):
            self.assertNotIn(forbidden, active)


if __name__ == "__main__":
    unittest.main()
