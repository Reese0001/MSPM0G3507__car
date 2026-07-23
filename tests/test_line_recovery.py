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

    def test_recovery_states_and_trend_input_match_sequence_design(self):
        for token in (
            "LINE_RECOVERY_FOLLOW",
            "LINE_RECOVERY_LOSS_CONFIRM",
            "LINE_RECOVERY_CORNER_PIVOT",
            "LINE_RECOVERY_FORWARD_SEARCH",
            "LINE_RECOVERY_REVERSAL_PAUSE",
            "LINE_RECOVERY_BACKTRACK",
            "LINE_RECOVERY_ALIGN",
            "LINE_RECOVERY_FAULT",
            "const LineTrendResult *trend",
        ):
            self.assertIn(token, self.header + self.source)
        self.assertIn("LINE_TREND_RIGHT_ANGLE_LEFT", self.source)
        self.assertIn("LINE_TREND_RIGHT_ANGLE_RIGHT", self.source)
        self.assertNotIn("LINE_RECOVERY_PIVOT_LEFT", self.header)
        self.assertNotIn("LINE_RECOVERY_PIVOT_RIGHT", self.header)

    def test_timing_and_commands_match_safe_recovery_plan(self):
        expected = (
            ("LINE_LOSS_CONFIRM_COUNT", "3U"),
            ("LINE_REACQUIRE_COUNT", "3U"),
            ("LINE_FORWARD_SEARCH_MS", "500U"),
            ("LINE_REVERSAL_PAUSE_MS", "120U"),
            ("LINE_BACKTRACK_MS", "700U"),
            ("LINE_ALIGN_DURATION_MS", "300U"),
            ("LINE_RECOVERY_TOTAL_TIMEOUT_MS", "3000U"),
            ("LINE_SEARCH_INNER_COMMAND", "80"),
            ("LINE_SEARCH_OUTER_COMMAND", "120"),
            ("LINE_CORNER_INNER_COMMAND", "-80"),
            ("LINE_CORNER_OUTER_COMMAND", "120"),
        )
        for name, value in expected:
            self.assertRegex(self.config, rf"{name}\s+\({re.escape(value)}\)")

    def test_forward_pause_backtrack_and_corner_requests_are_explicit(self):
        for helper in (
            "set_search_request",
            "set_pause_request",
            "set_backtrack_request",
            "set_corner_request",
        ):
            self.assertIn(helper, self.source)
        self.assertRegex(
            self.source,
            r"set_pause_request[\s\S]{0,300}left_speed\s*=\s*0"
            r"[\s\S]{0,120}right_speed\s*=\s*0"
            r"[\s\S]{0,120}valid\s*=\s*true",
        )
        for command in (
            "LINE_SEARCH_INNER_COMMAND",
            "LINE_SEARCH_OUTER_COMMAND",
            "-LINE_SEARCH_INNER_COMMAND",
            "-LINE_SEARCH_OUTER_COMMAND",
            "LINE_CORNER_INNER_COMMAND",
            "LINE_CORNER_OUTER_COMMAND",
        ):
            self.assertIn(command, self.source)

    def test_reacquisition_and_faults_are_bounded(self):
        self.assertIn("line_is_trustworthy", self.source)
        self.assertIn("line->confidence >= LINE_RECOVERY_MIN_CONFIDENCE", self.source)
        self.assertIn("absolute_value(line->error) <= LINE_RECOVERY_CENTER_ERROR", self.source)
        self.assertIn("reacquire_count >= LINE_REACQUIRE_COUNT", self.source)
        self.assertIn("LINE_RECOVERY_TOTAL_TIMEOUT_MS", self.source)
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

    def test_recovery_direction_is_locked_during_search_and_backtrack(self):
        self.assertRegex(
            self.source,
            r"if\s*\(recovery_state\s*==\s*LINE_RECOVERY_FOLLOW\s*\|\|\s*"
            r"recovery_state\s*==\s*LINE_RECOVERY_LOSS_CONFIRM\)"
            r"\s*\{\s*update_direction",
        )

    def test_corner_pivot_observes_reversal_pause(self):
        self.assertRegex(
            self.source,
            r"enter_state\(LINE_RECOVERY_CORNER_PIVOT,\s*now_ms\);"
            r"\s*set_pause_request",
        )
        self.assertRegex(
            self.source,
            r"recovery_state\s*==\s*LINE_RECOVERY_CORNER_PIVOT"
            r"[\s\S]{0,250}LINE_REVERSAL_PAUSE_MS",
        )

    def test_every_recovery_command_stays_inside_competition_limit(self):
        values = {
            name: int(value)
            for name, value in re.findall(
                r"#define\s+(LINE_(?:SEARCH|CORNER)_[A-Z_]+)\s+\((-?\d+)\)",
                self.config,
            )
        }
        self.assertGreaterEqual(len(values), 4)
        for name, value in values.items():
            with self.subTest(name=name):
                self.assertLessEqual(abs(value), 450)
        pause = re.search(
            r"LINE_REVERSAL_PAUSE_MS\s+\((\d+)U\)", self.config
        )
        self.assertIsNotNone(pause)
        self.assertGreaterEqual(int(pause.group(1)), 120)


if __name__ == "__main__":
    unittest.main()
