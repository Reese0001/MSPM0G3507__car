from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


def compile_and_run(harness: str, sources: tuple[str, ...]):
    vsdevcmd = (
        Path(os.environ["ProgramFiles"])
        / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
    )
    with tempfile.TemporaryDirectory() as temp_dir:
        executable = Path(temp_dir) / "harness.exe"
        source_args = " ".join(f'"{PROJECT / source}"' for source in sources)
        command = (
            f'call "{vsdevcmd}" -arch=x64 >nul && '
            f'cl /nologo /std:c11 /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /TC '
            f'/I"{PROJECT}" "{ROOT / "tests" / harness}" {source_args} '
            f'/Fe"{executable}" && "{executable}"'
        )
        return subprocess.run(
            command,
            cwd=temp_dir,
            capture_output=True,
            text=True,
            errors="replace",
            check=False,
            shell=True,
            executable=os.environ["ComSpec"],
        )


class H2Control(unittest.TestCase):
    def test_official_lap_speed_budget(self):
        config = (PROJECT / "modules/line_tracking/line_tracking_config.h").read_text(
            encoding="utf-8"
        )

        def value(name):
            match = re.search(rf"#define\s+{name}\s+\(([-0-9.]+)f?\)", config)
            self.assertIsNotNone(match, name)
            return float(match.group(1))

        straight = value("LINE_STRAIGHT_SPEED_MM_S")
        curve = value("LINE_SHARP_CURVE_SPEED_MM_S")
        lap_seconds = 3000.0 / straight + (3.1415926 * 1000.0) / curve
        self.assertLess(lap_seconds, 20.0)

    def test_stop_line_detector(self):
        result = compile_and_run(
            "stop_line_detector_harness.c",
            ("modules/line_tracking/stop_line_detector.c",),
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_motor_feedback(self):
        result = compile_and_run(
            "motor_feedback_harness.c",
            ("modules/motor/feedback/motor_feedback.c",),
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_yaw_estimator(self):
        result = compile_and_run(
            "yaw_estimator_harness.c",
            ("modules/imu/yaw_estimator.c",),
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_differential_controller(self):
        result = compile_and_run(
            "differential_controller_harness.c",
            ("modules/motor/feedback/differential_controller.c",),
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_stop_controller(self):
        result = compile_and_run(
            "stop_controller_harness.c",
            ("modules/motor/feedback/stop_controller.c",),
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
