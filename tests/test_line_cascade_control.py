from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class LineCascadeControlRuntime(unittest.TestCase):
    def test_lookup_feedforward_is_fused_into_two_loop_controller(self):
        config = (PROJECT / "config/line_cascade_config.h").read_text(
            encoding="utf-8"
        )
        source = (
            PROJECT / "modules/line_tracking/controller/line_cascade_control.c"
        ).read_text(encoding="utf-8")

        self.assertIn("LINE_CASCADE_MAX_COMMAND (140)", config)
        self.assertIn("LINE_CASCADE_POSITION_KP (14.0f)", config)
        self.assertIn("LINE_CASCADE_POSITION_KD (0.010f)", config)
        self.assertIn("LINE_CASCADE_ANGLE_KP (1.5f)", config)
        self.assertIn("LINE_CASCADE_ANGLE_KD (0.55f)", config)
        self.assertNotIn("forward_by_position", source)
        self.assertNotIn("YawSpeedBand", source)
        self.assertNotIn("yaw_speed_bands", source)
        self.assertNotIn("target_forward", source)

    def test_fused_controller_runtime_contract(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        harness = ROOT / "tests/line_cascade_control_harness.c"
        source = PROJECT / "modules/line_tracking/controller/line_cascade_control.c"

        self.assertTrue(vsdevcmd.exists(), "Visual Studio host toolchain missing")
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "line_cascade_control_harness.exe"
            command = (
                f'call "{vsdevcmd}" -arch=x64 >nul && '
                f'cl /nologo /std:c11 /utf-8 /W4 /WX /TC /I"{PROJECT}" '
                f'"{harness}" "{source}" /Fe"{executable}"'
            )
            result = subprocess.run(
                command,
                cwd=temp_dir,
                capture_output=True,
                text=True,
                errors="replace",
                check=False,
                shell=True,
                executable=os.environ["ComSpec"],
            )
            self.assertEqual(
                result.returncode, 0, (result.stdout or "") + result.stderr
            )
            run = subprocess.run(
                [str(executable)], capture_output=True, text=True, check=False
            )
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)


if __name__ == "__main__":
    unittest.main()
