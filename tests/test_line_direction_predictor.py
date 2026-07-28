import os
import pathlib
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
HARNESS = ROOT / "tests" / "line_direction_predictor_harness.c"
SOURCE = ROOT / "MSPM0G3507_LineFollowing_Car" / "modules" / "line_tracking" / "prediction" / "line_direction_predictor.c"


class LineDirectionPredictorTests(unittest.TestCase):
    def test_recent_frames_predict_direction_without_warnings(self):
        compilers = sorted(
            pathlib.Path("C:/Program Files/Microsoft Visual Studio").glob(
                "**/bin/Hostx64/x64/cl.exe"
            )
        )
        if not compilers:
            self.fail("MSVC cl.exe is required to verify /W4 /WX")

        compiler = compilers[-1]
        msvc_root = compiler.parents[3]
        environment = {
            key: value
            for key, value in os.environ.items()
            if key.upper() not in {"INCLUDE", "LIB", "PATH"}
        }
        environment["INCLUDE"] = str(msvc_root / "include")
        environment["LIB"] = str(msvc_root / "lib" / "x64")
        environment["PATH"] = str(compiler.parent) + os.pathsep + os.environ["PATH"]

        sdk_versions = sorted(
            pathlib.Path("C:/Program Files (x86)/Windows Kits/10/Lib").glob("*/um/x64")
        )
        if sdk_versions:
            sdk_root = sdk_versions[-1].parents[1]
            environment["LIB"] += os.pathsep + str(sdk_root / "um" / "x64")
            environment["LIB"] += os.pathsep + str(sdk_root / "ucrt" / "x64")

        with tempfile.TemporaryDirectory() as temp_dir:
            executable = pathlib.Path(temp_dir) / "line_direction_predictor_harness.exe"
            compile_result = subprocess.run(
                [str(compiler), "/nologo", "/W4", "/WX", str(HARNESS), str(SOURCE),
                 f"/Fe:{executable}"],
                cwd=ROOT,
                text=True,
                capture_output=True,
                env=environment,
                encoding="utf-8",
                errors="replace",
            )
            self.assertEqual(
                compile_result.returncode,
                0,
                compile_result.stdout + compile_result.stderr,
            )

            run_result = subprocess.run(
                [str(executable)], cwd=ROOT, text=True, capture_output=True
            )
            self.assertEqual(run_result.returncode, 0, run_result.stderr)


if __name__ == "__main__":
    unittest.main()
