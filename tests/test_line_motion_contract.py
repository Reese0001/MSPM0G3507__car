from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class LineMotionContract(unittest.TestCase):
    def test_line_motion_owns_line_to_request_logic(self):
        header = (ROOT / "app/line/line_motion.h").read_text(encoding="utf-8")
        source = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        control = (ROOT / "app/control/control_runtime.c").read_text(
            encoding="utf-8"
        )
        for token in (
            "AppLineMotion_Init",
            "AppLineMotion_ServiceImu",
            "AppLineMotion_BuildRequest",
            "LineLookupControl_Step",
            "LineRecovery_Step",
        ):
            self.assertIn(token, header + source)
        self.assertIn("AppLineMotion_BuildRequest", control)

    def test_noise_and_bad_inputs_do_not_publish_valid_line_request(self):
        source = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        self.assertIn("sample == 0", source)
        self.assertIn("request == 0", source)
        self.assertIn("LINE_PATTERN_NOISE", source)
        self.assertIn("request->valid = false", source)


if __name__ == "__main__":
    unittest.main()
