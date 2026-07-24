import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"
WORKTREE = ROOT.parent


class MotorDirectionInterlockContract(unittest.TestCase):
    def test_arm_serializes_fault_latch_check_and_assignment(self):
        source = (ROOT / "modules/motor/motor_safety.c").read_text(
            encoding="utf-8"
        )
        arm = source[source.index("void Motor_Safety_Arm"):
                     source.index("void Motor_Safety_Disarm")]
        self.assertIn("motor_safety_enter_critical", arm)
        self.assertIn("MOTOR_SAFETY_FAULT_LATCHED", arm)
        self.assertIn("motor_safety_exit_critical", arm)
        self.assertLess(arm.index("MOTOR_SAFETY_FAULT_LATCHED"),
                        arm.index("safety_state = MOTOR_SAFETY_ARMED"))

    def test_safety_header_exposes_the_120ms_direction_pause(self):
        header = (ROOT / "modules/motor/motor_safety.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("MOTOR_SAFETY_DIRECTION_CHANGE_PAUSE_MS (120U)", header)

    def test_only_motor_adapter_submits_speed_requests(self):
        callers = []
        for path in ROOT.rglob("*.c"):
            if path.name == "motor_safety.c":
                continue
            if "Motor_Safety_RequestSpeed(" in path.read_text(
                encoding="utf-8", errors="ignore"
            ):
                callers.append(path.as_posix())
        self.assertEqual(
            [str(ROOT / "modules/motor/motor_adapter.c").replace("\\", "/")],
            callers,
        )


class MotorDirectionInterlockRuntime(unittest.TestCase):
    def test_host_harness_exercises_production_safety_layer(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        self.assertTrue(vsdevcmd.exists(), "Visual Studio host toolchain missing")
        harness = WORKTREE / "tests/motor_direction_interlock_harness.c"
        source = ROOT / "modules/motor/motor_safety.c"
        build_dir = ROOT / "Build_LineFollowing"
        sdk = Path(r"C:\ti\mspm0_sdk_2_10_00_04")

        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "motor_direction_interlock_harness.exe"
            compile_command = (
                f'call "{vsdevcmd}" -arch=x64 >nul && '
                f'cl /nologo /std:c11 /utf-8 /W4 /WX /DMOTOR_SAFETY_HOST_TEST /TC '
                f'/I"{ROOT / "modules/motor"}" '
                f'/I"{ROOT / "modules/led"}" '
                f'/I"{build_dir}" '
                f'/I"{sdk / "source"}" '
                f'/I"{sdk / "source/third_party/CMSIS/Core/Include"}" '
                f'"{harness}" "{source}" /Fe"{executable}"'
            )
            build = subprocess.run(
                compile_command,
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
