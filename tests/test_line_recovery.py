from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class LineRecoveryContract(unittest.TestCase):
    def test_recovery_has_bounded_states_and_motion_request_output(self):
        header = (ROOT / "application/line_recovery.h").read_text(encoding="utf-8")
        source = (ROOT / "application/line_recovery.c").read_text(encoding="utf-8")
        for token in (
            "LINE_RECOVERY_FOLLOW",
            "LINE_RECOVERY_LOSS_CONFIRM",
            "LINE_RECOVERY_PIVOT_LEFT",
            "LINE_RECOVERY_PIVOT_RIGHT",
            "LINE_RECOVERY_ALIGN",
            "LINE_RECOVERY_FAULT",
            "LineRecovery_Step",
            "MotionRequest",
            "yaw_fresh",
            "emergency_stop",
        ):
            self.assertIn(token, header + source)
        self.assertNotIn("Contrl_Speed", source)
        self.assertNotIn("Motion_Car_Control", source)
        self.assertNotIn("Motor_Safety_Arm", source)

    def test_recovery_limits_match_safety_plan(self):
        config = (ROOT / "application/config/line_recovery_config.h").read_text(
            encoding="utf-8"
        )
        expected = (
            ("LINE_LOSS_CONFIRM_COUNT", "3U"),
            ("LINE_REACQUIRE_COUNT", "3U"),
            ("LINE_PIVOT_FORWARD_PERCENT", "12"),
            ("LINE_SEARCH_INNER_PERCENT", "8"),
            ("LINE_SHARP_INNER_REVERSE_PERCENT", "8"),
            ("LINE_SHARP_SEARCH_ERROR", "4.0f"),
            ("LINE_RECOVERY_MAX_YAW_DEG", "45.0f"),
            ("LINE_RECOVERY_TIMEOUT_MS", "3000U"),
            ("LINE_ALIGN_DURATION_MS", "300U"),
            ("LINE_RECOVERY_ESTIMATE_STALE_MS", "20U"),
        )
        for name, value in expected:
            self.assertRegex(config, rf"{name}\s+\({re.escape(value)}\)")

    def test_search_uses_forward_arc_except_after_a_sharp_edge(self):
        source = (ROOT / "application/line_recovery.c").read_text(encoding="utf-8")
        primitives = (ROOT / "application/motion_primitives.c").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("emergency_stop || !yaw_fresh", source)
        self.assertIn("if (emergency_stop)", source)
        self.assertRegex(
            source,
            r"yaw_fresh\s*&&[\s\S]{0,100}LINE_RECOVERY_MAX_YAW_DEG",
        )
        self.assertIn("LINE_RECOVERY_MAX_YAW_DEG", source)
        self.assertIn("LINE_RECOVERY_TIMEOUT_MS", source)
        self.assertIn("request->valid = false", source)
        self.assertNotIn("-LINE_SEARCH_INNER_COMMAND", source + primitives)
        self.assertRegex(source, r"inner_speed\s*=\s*sharp_search\s*\?")
        self.assertIn("LINE_SEARCH_INNER_COMMAND", source)
        self.assertRegex(source, r"right_speed\s*=\s*LINE_PIVOT_FORWARD_COMMAND")
        self.assertRegex(source, r"left_speed\s*=\s*LINE_PIVOT_FORWARD_COMMAND")
        self.assertRegex(source, r"left_speed\s*=\s*inner_speed")
        self.assertRegex(source, r"right_speed\s*=\s*inner_speed")
        self.assertIn(
            "absolute_value(last_seen_error) >= LINE_SHARP_SEARCH_ERROR",
            source,
        )
        self.assertIn("-LINE_SHARP_INNER_REVERSE_COMMAND", source)


if __name__ == "__main__":
    unittest.main()
