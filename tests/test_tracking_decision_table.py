from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT
    / "MSPM0G3507_LineFollowing_Car"
    / "BSP/Eight_Tracking/app_irtracking.c"
).read_text(encoding="utf-8")
HEADER = (
    ROOT
    / "MSPM0G3507_LineFollowing_Car"
    / "BSP/Eight_Tracking/app_irtracking.h"
).read_text(encoding="utf-8")


class TrackingDecisionTests(unittest.TestCase):
    def test_weighted_error_api_and_symmetric_weights_exist(self):
        self.assertIn("Tracking_ComputeWeightedError", HEADER)
        self.assertRegex(
            SOURCE,
            r"TRACKING_WEIGHTS\s*\[\s*8\s*\]\s*=\s*"
            r"\{\s*-7\s*,\s*-5\s*,\s*-3\s*,\s*-1\s*,"
            r"\s*1\s*,\s*3\s*,\s*5\s*,\s*7\s*\}",
        )

    def test_weighted_error_rejects_all_white(self):
        body = re.search(
            r"Tracking_ComputeWeightedError\s*\([^)]*\)\s*\{(.*?)\n\}",
            SOURCE,
            re.DOTALL,
        )
        self.assertIsNotNone(body)
        self.assertRegex(body.group(1), r"active_count\s*==\s*0")
        self.assertRegex(body.group(1), r"return\s+false\s*;")

    def test_pid_updates_previous_error(self):
        body = re.search(
            r"float\s+APP_ELE_PID_Calc\s*\([^)]*\)\s*\{(.*?)\n\}",
            SOURCE,
            re.DOTALL,
        ).group(1)
        self.assertRegex(body, r"error_last\s*=\s*error\s*;")

    def test_all_white_lost_line_requests_zero_speed(self):
        self.assertRegex(SOURCE, r"lost_count\s*>=\s*TRACKING_LOST_STOP_CYCLES")
        self.assertIn("Motion_Car_Control(0, 0, 0)", SOURCE)

    def test_tracking_loop_has_no_blocking_corner_delay(self):
        body = re.search(
            r"void\s+LineWalking\s*\([^)]*\)\s*\{(.*?)\n\}",
            SOURCE,
            re.DOTALL,
        ).group(1)
        self.assertNotIn("delay_ms(100)", body)

    def test_center_pattern_has_zero_error(self):
        self.assertRegex(SOURCE, r"weighted_sum\s*/\s*\(float\)\s*active_count")


if __name__ == "__main__":
    unittest.main()
