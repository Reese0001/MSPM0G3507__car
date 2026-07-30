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

    def test_probe_state_is_plumbed_to_oled_dashboard(self):
        app_tasks = (PROJECT / "app/tasks/app_tasks.c").read_text(
            encoding="utf-8"
        )
        dashboard_header = (PROJECT / "modules/display/dashboard.h").read_text(
            encoding="utf-8"
        )
        dashboard = (PROJECT / "modules/display/dashboard.c").read_text(
            encoding="utf-8"
        )
        wifi_uart = (PROJECT / "modules/wifi/wifi_uart.c").read_text(
            encoding="utf-8"
        )

        self.assertIn("WifiUart_Init(now_ms);", app_tasks)
        self.assertIn("WifiUart_Service(now_ms);", app_tasks)
        self.assertIn("WifiUart_GetProbeState()", app_tasks)
        self.assertIn("WifiAtProbeState wifi_state;", dashboard_header)
        self.assertIn(
            "static const uint8_t at_command[] = {'A', 'T', '\\r', '\\n'};",
            wifi_uart,
        )
        self.assertIn('"DST%04d ANG%+03d"', dashboard)
        self.assertIn("ESP OK", dashboard)
        self.assertIn("ESP TIMEOUT", dashboard)
        self.assertIn("ESP WAIT", dashboard)
        self.assertIn("Ssd1306_DrawText(7U, 0U, text);", dashboard)
        self.assertNotIn('"YAW%+03d TURN%+03d"', dashboard)
        self.assertEqual(dashboard.count("Ssd1306_DrawText(6U, 0U, text);"), 1)


if __name__ == "__main__":
    unittest.main()
