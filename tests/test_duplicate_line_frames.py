from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class DuplicateLineFrameRuntime(unittest.TestCase):
    def test_duplicate_frames_do_not_advance_temporal_evidence(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        harness = ROOT / "tests/duplicate_line_frame_harness.c"
        sources = (
            PROJECT / "modules/optional/competition/line_tracking/line_trend_detector.c",
            PROJECT / "modules/optional/competition/line_tracking/line_controller.c",
        )

        self.assertTrue(vsdevcmd.exists(), "Visual Studio host toolchain missing")
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "duplicate_line_frame_harness.exe"
            command = (
                f'call "{vsdevcmd}" -arch=x64 >nul && '
                f'cl /nologo /std:c11 /utf-8 /W4 /WX /TC /I"{PROJECT}" '
                f'"{harness}" "{sources[0]}" "{sources[1]}" '
                f'/Fe"{executable}" && "{executable}"'
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


if __name__ == "__main__":
    unittest.main()

