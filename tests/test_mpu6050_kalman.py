from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class Mpu6050KalmanRuntime(unittest.TestCase):
    def test_yaw_angle_kalman_predicts_and_learns_stationary_bias(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        harness = ROOT / "tests/mpu6050_kalman_harness.c"
        source = PROJECT / "modules/mpu6050/mpu6050_kalman.c"

        self.assertTrue(vsdevcmd.exists(), "Visual Studio host toolchain missing")
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "mpu6050_kalman_harness.exe"
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
