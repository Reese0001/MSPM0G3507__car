from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class WifiAtProbeTests(unittest.TestCase):
    def test_at_probe_state_machine(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "wifi_at_probe_harness.exe"
            command = (
                f'call "{vsdevcmd}" -arch=x64 >nul && '
                f'cl /nologo /std:c11 /utf-8 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /TC '
                f'/I"{PROJECT}" "{ROOT / "tests" / "wifi_at_probe_harness.c"}" '
                f'"{PROJECT / "modules/wifi/wifi_at_probe.c"}" '
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
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
