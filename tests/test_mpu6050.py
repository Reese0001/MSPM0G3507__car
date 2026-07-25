from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class Mpu6050Runtime(unittest.TestCase):
    def test_calibration_filter_sign_and_error_degrade(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        harness = ROOT.parent / "tests/mpu6050_harness.c"
        source = ROOT / "modules/mpu6050/mpu6050.c"

        self.assertTrue(vsdevcmd.exists(), "Visual Studio host toolchain missing")
        self.assertTrue(source.exists(), source)
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "mpu6050_harness.exe"
            command = (
                f'call "{vsdevcmd}" -arch=x64 >nul && '
                f'cl /nologo /W4 /WX /TC /I"{ROOT}" '
                f'"{harness}" "{source}" /Fe"{executable}"'
            )
            build = subprocess.run(
                command,
                capture_output=True,
                text=True,
                errors="replace",
                check=False,
                shell=True,
                executable=os.environ["ComSpec"],
            )
            self.assertEqual(
                build.returncode, 0, (build.stdout or "") + build.stderr
            )
            run = subprocess.run(
                [str(executable)], capture_output=True, text=True, check=False
            )
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)


if __name__ == "__main__":
    unittest.main()
