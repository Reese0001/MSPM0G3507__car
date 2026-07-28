from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class RunControllerContract(unittest.TestCase):
    def test_public_api_is_start_gate_only(self):
        header = (ROOT / "app/run/run_controller.h").read_text(encoding="utf-8")
        source = (ROOT / "app/run/run_controller.c").read_text(encoding="utf-8")
        for token in (
            "RunController_Init",
            "RunController_OnKeyEvent",
            "RunController_IsRunRequested",
            "KeyEvent",
        ):
            self.assertIn(token, header + source)
        self.assertNotIn("RunController_BuildRequest", header + source)
        self.assertNotIn("RUN_CONTROLLER_BRINGUP_SPEED", source)
        self.assertNotIn("MotionRequest", header + source)
        self.assertNotIn("Motor_Safety_RequestSpeed", source)
        self.assertNotIn("Motor_SendSpeedFrame", source)


if __name__ == "__main__":
    unittest.main()
