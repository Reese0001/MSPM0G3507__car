from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class LineOfficialControlRuntime(unittest.TestCase):
    def test_baseline_lookup_with_optional_z_rate_damping(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        harness = ROOT / "tests/line_official_control_harness.c"
        sources = (
            PROJECT / "modules/line_tracking/controller/line_official_control.c",
            PROJECT / "modules/line_tracking/controller/line_lookup_control.c",
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "line_official_control_harness.exe"
            source_args = " ".join(f'"{source}"' for source in sources)
            command = (
                f'call "{vsdevcmd}" -arch=x64 >nul && '
                f'cl /nologo /std:c11 /utf-8 /W4 /WX /TC /I"{PROJECT}" '
                f'"{harness}" {source_args} /Fe"{executable}" && "{executable}"'
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
