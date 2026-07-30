from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class LineScannerTimebaseRuntime(unittest.TestCase):
    def test_scanner_has_no_blocking_full_frame_api(self):
        source = (PROJECT / "modules/line_tracking/scanner/four_line_scanner.c").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("LineScanner_ReadFrame", source)

    def test_published_timestamp_uses_system_millisecond_clock(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        harness = ROOT / "tests/line_scanner_timebase_harness.c"
        source = PROJECT / "modules/line_tracking/scanner/four_line_scanner.c"

        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "line_scanner_timebase_harness.exe"
            command = (
                f'call "{vsdevcmd}" -arch=x64 >nul && '
                f'cl /nologo /std:c11 /utf-8 /W4 /WX /TC '
                f'/DFOUR_LINE_SCANNER_HOST_TEST /I"{PROJECT}" '
                f'"{harness}" "{source}" '
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
