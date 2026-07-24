from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class LineRecoveryContract(unittest.TestCase):
    def setUp(self):
        self.header = (ROOT / "application/line_recovery.h").read_text(
            encoding="utf-8"
        )
        self.source = (ROOT / "application/line_recovery.c").read_text(
            encoding="utf-8"
        )
        self.config = (
            ROOT / "application/config/line_recovery_config.h"
        ).read_text(encoding="utf-8")

    def test_recovery_has_no_backtrack_state_or_request(self):
        self.assertNotIn("LINE_RECOVERY_BACKTRACK", self.header)
        self.assertNotIn("set_backtrack_request", self.source)
        self.assertNotIn("LINE_BACKTRACK_MS", self.config)

    def test_recovery_uses_forward_then_pause_then_rotate(self):
        for token in (
            "LINE_RECOVERY_FORWARD_SEARCH",
            "LINE_RECOVERY_ROTATION_PAUSE",
            "LINE_RECOVERY_ROTATE_SEARCH",
            "LINE_RECOVERY_ALIGN",
        ):
            self.assertIn(token, self.header)
        for token in (
            "LINE_FORWARD_SEARCH_MS (500U)",
            "LINE_ROTATION_PAUSE_MS (120U)",
            "LINE_ROTATE_SEARCH_MS (700U)",
            "LINE_ROTATE_INNER_COMMAND (-60)",
            "LINE_ROTATE_OUTER_COMMAND (100)",
        ):
            self.assertIn(token, self.config)

    def test_no_recovery_helper_commands_both_wheels_negative(self):
        pairs = [
            tuple(map(int, pair))
            for pair in re.findall(
                r"publish_request\(\s*(-?\d+|LINE_[A-Z_]+),\s*"
                r"(-?\d+|LINE_[A-Z_]+)",
                self.source,
            )
            if all(value.lstrip("-").isdigit() for value in pair)
        ]
        self.assertFalse(any(left < 0 and right < 0 for left, right in pairs))
        self.assertNotRegex(
            self.source,
            r"publish_request\(\s*-LINE_SEARCH_[A-Z_]+,\s*"
            r"-LINE_SEARCH_[A-Z_]+",
        )

    def test_corner_handling_is_not_owned_by_line_recovery(self):
        self.assertNotIn("LINE_RECOVERY_CORNER", self.header + self.source)
        self.assertNotIn("trend_is_right_angle", self.source)
        self.assertNotIn("set_corner_request", self.source)
        self.assertNotIn("LINE_CORNER_", self.config)

    def test_reacquisition_and_faults_are_bounded(self):
        self.assertIn("line_is_trustworthy", self.source)
        self.assertIn("line->confidence >= LINE_RECOVERY_MIN_CONFIDENCE", self.source)
        self.assertIn("absolute_value(line->error) <= LINE_RECOVERY_CENTER_ERROR", self.source)
        self.assertIn("reacquire_count >= LINE_REACQUIRE_COUNT", self.source)
        self.assertIn("LINE_RECOVERY_TOTAL_TIMEOUT_MS", self.source)
        self.assertIn("LINE_ROTATE_SEARCH_MS", self.source)
        self.assertIn("if (emergency_stop)", self.source)
        self.assertIn("request->valid = false", self.source)
        self.assertNotIn("last_seen_error", self.source)
        self.assertNotIn("LINE_SHARP_SEARCH_ERROR", self.source + self.config)
        self.assertNotIn("Contrl_Speed", self.source)
        self.assertNotIn("Motion_Car_Control", self.source)
        self.assertNotIn("Motor_Safety_Arm", self.source)

    def test_align_timer_is_not_restarted_by_each_trusted_frame(self):
        self.assertRegex(
            self.source,
            r"recovery_state\s*!=\s*LINE_RECOVERY_ALIGN\s*&&\s*"
            r"update_reacquisition",
        )

    def test_recovery_direction_is_locked_during_search(self):
        self.assertRegex(
            self.source,
            r"if\s*\(recovery_state\s*==\s*LINE_RECOVERY_FOLLOW\s*\|\|\s*"
            r"recovery_state\s*==\s*LINE_RECOVERY_LOSS_CONFIRM\)"
            r"\s*\{\s*update_direction",
        )

    def test_forward_search_pauses_before_rotate_search(self):
        self.assertRegex(
            self.source,
            r"LINE_RECOVERY_FORWARD_SEARCH[\s\S]{0,500}"
            r"LINE_RECOVERY_ROTATION_PAUSE",
        )
        self.assertRegex(
            self.source,
            r"LINE_RECOVERY_ROTATION_PAUSE[\s\S]{0,500}"
            r"LINE_ROTATION_PAUSE_MS[\s\S]{0,500}"
            r"LINE_RECOVERY_ROTATE_SEARCH",
        )

    def test_every_recovery_command_stays_inside_competition_limit(self):
        values = {
            name: int(value)
            for name, value in re.findall(
                r"#define\s+(LINE_(?:SEARCH|ROTATE)_[A-Z_]+)\s+\((-?\d+)\)",
                self.config,
            )
        }
        self.assertGreaterEqual(len(values), 5)
        for name, value in values.items():
            with self.subTest(name=name):
                self.assertLessEqual(abs(value), 450)
        pause = re.search(
            r"LINE_ROTATION_PAUSE_MS\s+\((\d+)U\)", self.config
        )
        self.assertIsNotNone(pause)
        self.assertGreaterEqual(int(pause.group(1)), 120)


if __name__ == "__main__":
    unittest.main()
