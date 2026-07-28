import re
import os
import subprocess
import tempfile
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
        self.assertNotIn("latched_fault = APP_FAULT_SENSOR_HEARTBEAT", safety)
        self.assertIn("sensor_heartbeat_missing", safety)

    def test_zero_line_request_cannot_hide_initial_test_run(self):
        safety = self.safety_runtime
        self.assertIn("LineRequestCanOverride", safety)
        self.assertIn("motion_output_seen", safety)
        override = safety[
            safety.index("static bool LineRequestCanOverride"):
            safety.index("static SafetyInputs BuildInputs")
        ]
        self.assertIn("MotionRequestHasOutput", override)
        self.assertIn("motion_output_seen", override)

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

    def test_reset_path_reaches_nonzero_motor_frame_without_line_frame(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        self.assertTrue(vsdevcmd.exists(), "Visual Studio host toolchain missing")
        harness = ROOT / "tests" / "run_first_bringup_harness.c"
        sources = [
            PROJECT / "app/safety/safety_runtime.c",
            PROJECT / "app/safety/safety_supervisor.c",
            PROJECT / "app/mailbox/app_mailbox.c",
            PROJECT / "app/run/run_controller.c",
            PROJECT / "modules/motor/adapter/motor_adapter.c",
            PROJECT / "modules/motor/safety/motor_safety.c",
        ]
        include_flags = [
            f'/I"{ROOT / "tests/host_stubs"}"',
            f'/I"{PROJECT}"',
            f'/I"{PROJECT / "app"}"',
            f'/I"{PROJECT / "config"}"',
            f'/I"{PROJECT / "shared"}"',
            f'/I"{PROJECT / "modules"}"',
            f'/I"{PROJECT / "modules/motor/safety"}"',
            f'/I"{PROJECT / "modules/motor/adapter"}"',
            f'/I"{PROJECT / "modules/key"}"',
        ]
        source_args = " ".join(f'"{path}"' for path in [harness, *sources])

        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "run_first_bringup_harness.exe"
            compile_command = (
                f'call "{vsdevcmd}" -arch=x64 >nul && '
                f'cl /nologo /std:c11 /utf-8 /W4 /WX '
                f'/DMOTOR_SAFETY_HOST_TEST /TC '
                f'{" ".join(include_flags)} '
                f'{source_args} /Fe"{executable}"'
            )
            build = subprocess.run(
                compile_command,
                cwd=temp_dir,
                capture_output=True,
                text=True,
                errors="replace",
                check=False,
                shell=True,
                executable=os.environ["ComSpec"],
            )
            self.assertEqual(
                0, build.returncode, (build.stdout or "") + build.stderr
            )
            run = subprocess.run(
                [str(executable)], capture_output=True, text=True, check=False
            )
            self.assertEqual(0, run.returncode, run.stdout + run.stderr)


if __name__ == "__main__":
    unittest.main()
