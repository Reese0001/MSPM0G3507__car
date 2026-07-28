import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"
TASKS = PROJECT / "app" / "tasks" / "app_tasks.c"
RUN_CONTROLLER = PROJECT / "app" / "run" / "run_controller.c"
OBSERVER = PROJECT / "app" / "log" / "runtime_observer.c"
SAFETY_RUNTIME = PROJECT / "app" / "safety" / "safety_runtime.c"


def read_tasks():
    return TASKS.read_text(encoding="utf-8")


class RunFirstBringupContract(unittest.TestCase):
    def setUp(self):
        self.source = read_tasks()
        self.run_controller = RUN_CONTROLLER.read_text(encoding="utf-8")
        self.observer = OBSERVER.read_text(encoding="utf-8")
        self.safety_runtime = SAFETY_RUNTIME.read_text(encoding="utf-8")

    def test_bringup_constants_are_low_and_bounded(self):
        self.assertIn("RUN_CONTROLLER_BRINGUP_SPEED", self.run_controller)
        match = re.search(
            r"#define\s+RUN_CONTROLLER_BRINGUP_SPEED\s+\((\d+)\)",
            self.run_controller,
        )
        self.assertIsNotNone(match)
        self.assertLessEqual(int(match.group(1)), 180)

    def test_safety_task_builds_default_run_without_line_frame(self):
        safety = self.safety_runtime
        self.assertIn("RunController_BuildRequest", safety)
        self.assertLess(
            safety.index("RunController_BuildRequest"),
            safety.index("AppMailbox_ReadMotionRequest"),
        )
        self.assertNotIn("APP_FAULT_CONTROL_HEARTBEAT", safety)

    def test_k1_can_request_run_after_boot(self):
        self.assertIn('#include "../../modules/key/key.h"', self.safety_runtime)
        safety = self.safety_runtime
        self.assertIn("Key_PollEvent()", safety)
        self.assertIn("RunController_OnKeyEvent", safety)
        self.assertIn("KEY_EVENT_SHORT", self.run_controller)
        self.assertIn("run_requested = true", self.run_controller)

    def test_motor_output_still_goes_through_safety_layer(self):
        self.assertIn("MotorAdapter_Apply(&decision);", self.safety_runtime)
        self.assertNotIn("Motor_SendSpeedFrame(", self.source + self.safety_runtime)
        self.assertNotIn("Send_Motor_ArrayU8(", self.source + self.safety_runtime)

    def test_oled_logs_test_run(self):
        self.assertIn('"TEST RUN"', self.observer)


if __name__ == "__main__":
    unittest.main()
