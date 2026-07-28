import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TASKS = ROOT / "MSPM0G3507_LineFollowing_Car" / "app" / "tasks" / "app_tasks.c"


def read_tasks():
    return TASKS.read_text(encoding="utf-8")


class RunFirstBringupContract(unittest.TestCase):
    def setUp(self):
        self.source = read_tasks()

    def test_bringup_constants_are_low_and_bounded(self):
        self.assertIn("APP_BRINGUP_RUN_SPEED", self.source)
        match = re.search(
            r"#define\s+APP_BRINGUP_RUN_SPEED\s+\((\d+)\)", self.source
        )
        self.assertIsNotNone(match)
        self.assertLessEqual(int(match.group(1)), 180)

    def test_safety_task_builds_default_run_without_line_frame(self):
        safety = self.source[self.source.index("static void SafetyTask") :]
        safety = safety[: self.source.index("static void DisplayTask")]
        self.assertIn("BuildBringupRunRequest", safety)
        self.assertLess(
            safety.index("BuildBringupRunRequest"),
            safety.index("AppMailbox_ReadMotionRequest"),
        )
        self.assertNotIn("APP_FAULT_CONTROL_HEARTBEAT", safety)

    def test_k1_can_request_run_after_boot(self):
        self.assertIn('#include "../../modules/key/key.h"', self.source)
        safety = self.source[self.source.index("static void SafetyTask") :]
        safety = safety[: self.source.index("static void DisplayTask")]
        self.assertIn("Key_PollEvent()", safety)
        self.assertIn("KEY_EVENT_SHORT", safety)
        self.assertIn("bringup_run_requested = true", safety)

    def test_motor_output_still_goes_through_safety_layer(self):
        self.assertIn("MotorAdapter_Apply(&decision);", self.source)
        self.assertNotIn("Motor_SendSpeedFrame(", self.source)
        self.assertNotIn("Send_Motor_ArrayU8(", self.source)

    def test_oled_logs_test_run(self):
        self.assertIn('"TEST RUN"', self.source)


if __name__ == "__main__":
    unittest.main()
