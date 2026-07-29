from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class OptionalFocMathRuntime(unittest.TestCase):
    def test_foc_transforms_and_voltage_limit(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        harness = ROOT / "tests/foc_math_harness.c"
        source = PROJECT / "modules/optional/foc/foc_math.c"

        self.assertTrue(vsdevcmd.exists(), "Visual Studio host toolchain missing")
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "foc_math_harness.exe"
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

    def test_foc_is_not_connected_to_dc_motor_build(self):
        makefile = (PROJECT / "Makefile").read_text(encoding="utf-8")
        motor_sources = "\n".join(
            path.read_text(encoding="utf-8", errors="ignore")
            for path in (PROJECT / "modules/motor").rglob("*.c")
        )

        self.assertNotIn("modules/optional/foc/foc_math.c", makefile)
        self.assertNotIn("FocMath_", motor_sources)


if __name__ == "__main__":
    unittest.main()
