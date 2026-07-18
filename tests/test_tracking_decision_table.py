from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT
    / "empty_LP_MSPM0G3507_nortos_ticlang"
    / "BSP/Eight_Tracking/app_irtracking.c"
).read_text(encoding="utf-8")


class TrackingDecisionTests(unittest.TestCase):
    def test_pid_updates_previous_error(self):
        body = re.search(
            r"float\s+APP_ELE_PID_Calc\s*\([^)]*\)\s*\{(.*?)\n\}",
            SOURCE,
            re.DOTALL,
        ).group(1)
        self.assertRegex(body, r"error_last\s*=\s*error\s*;")

    def test_all_white_lost_line_requests_zero_speed(self):
        predicate = (
            "else if(x1 == 1 && x2 == 1 &&x3 == 1 &&  x4 == 1  && "
            "x5 == 1 && x6  == 1 && x7 == 1 && x8 == 1 )"
        )
        start = SOURCE.index(predicate)
        end = SOURCE.index("else if", start + len(predicate))
        self.assertIn("Motion_Car_Control(0, 0, 0)", SOURCE[start:end])

    def test_tracking_loop_has_no_blocking_corner_delay(self):
        body = re.search(
            r"void\s+LineWalking\s*\([^)]*\)\s*\{(.*?)\n\}",
            SOURCE,
            re.DOTALL,
        ).group(1)
        self.assertNotIn("delay_ms(100)", body)

    def test_center_pattern_has_zero_error(self):
        self.assertIsNotNone(
            re.search(
                r"x4\s*==\s*0\s*&&\s*x5\s*==\s*0.*?err\s*=\s*0",
                SOURCE,
                re.DOTALL,
            )
        )


if __name__ == "__main__":
    unittest.main()
