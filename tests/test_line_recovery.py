from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class LineRecoveryContract(unittest.TestCase):
    def setUp(self):
        self.header = (ROOT / "modules/line_tracking/recovery/line_recovery.h").read_text(
            encoding="utf-8"
        )
        self.source = (ROOT / "modules/line_tracking/recovery/line_recovery.c").read_text(
            encoding="utf-8"
        )
        self.config = (ROOT / "config/line_recovery_config.h").read_text(
            encoding="utf-8"
        )

    def test_public_contract_uses_only_five_states_and_predicted_direction(self):
        for token in (
            "LINE_RECOVERY_FOLLOW",
            "LINE_RECOVERY_SEEK_LEFT",
            "LINE_RECOVERY_SEEK_RIGHT",
            "LINE_RECOVERY_ALIGN",
            "LINE_RECOVERY_STOPPED",
            "LineRecoveryDiagnostics",
            "LineRecovery_GetDiagnostics",
            "int8_t predicted_direction",
            "float yaw_rate_dps",
        ):
            self.assertIn(token, self.header)

        forbidden = (
            "LINE_RECOVERY_LOSS_CONFIRM",
            "LINE_RECOVERY_FORWARD_SEARCH",
            "LINE_RECOVERY_ROTATION_PAUSE",
            "LINE_RECOVERY_ROTATE_SEARCH",
            "LineTrendResult",
        )
        for token in forbidden:
            with self.subTest(token=token):
                self.assertNotIn(token, self.header + self.source)

    def test_configuration_contains_only_the_new_recovery_constants(self):
        definitions = dict(
            re.findall(r"#define\s+(LINE_[A-Z0-9_]+)\s+\(([^)]+)\)", self.config)
        )
        self.assertEqual(
            definitions,
            {
                "LINE_REACQUIRE_COUNT": "3U",
                "LINE_ALIGN_DURATION_MS": "300U",
                "LINE_RECOVERY_ESTIMATE_STALE_MS": "20U",
                "LINE_RECOVERY_MIN_CONFIDENCE": "40U",
                "LINE_SEEK_COMMAND": "100",
                "LINE_SEEK_LIMITED_COMMAND": "80",
                "LINE_SEEK_HIGH_YAW_DPS": "120.0f",
                "LINE_ALIGN_COMMAND_LIMIT": "80",
            },
        )

    def test_legacy_search_contract_is_deleted(self):
        combined = self.header + self.source + self.config
        for token in (
            "LINE_FORWARD_SEARCH_MS",
            "LINE_ROTATE_SEARCH_MS",
            "LINE_SEARCH_",
            "LINE_ROTATE_",
            "LINE_RECOVERY_CENTER_ERROR",
            "recovery_direction = (int8_t)-recovery_direction",
            "set_forward_search",
            "set_rotate_search",
        ):
            with self.subTest(token=token):
                self.assertNotIn(token, combined)

        self.assertNotRegex(self.source, r"publish_request\(\s*-\d+")
        self.assertEqual(self.source.count("static void set_seek_request("), 1)

    def test_reacquisition_and_safety_contract_is_explicit(self):
        self.assertIn("line->event == LINE_EVENT_NONE", self.source)
        self.assertIn("line->confidence >= LINE_RECOVERY_MIN_CONFIDENCE", self.source)
        self.assertIn("reacquire_count >= LINE_REACQUIRE_COUNT", self.source)
        self.assertIn("if (emergency_stop)", self.source)
        self.assertIn("request->valid = false", self.source)
        self.assertNotIn("LINE_RECOVERY_TOTAL_TIMEOUT_MS", self.source + self.config)
        for token in (
            "Contrl_Speed",
            "Motion_Car_Control",
            "Motor_Safety_Arm",
            "LineTrendResult",
        ):
            self.assertNotIn(token, self.source)


class LineRecoveryRuntime(unittest.TestCase):
    def test_host_harness_exercises_one_way_recovery_state_machine(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        harness = ROOT.parent / "tests/line_recovery_harness.c"
        source = ROOT / "modules/line_tracking/recovery/line_recovery.c"

        self.assertTrue(vsdevcmd.exists(), "Visual Studio host toolchain missing")
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "line_recovery_harness.exe"
            compile_command = (
                f'call "{vsdevcmd}" -arch=x64 >nul && '
                f'cl /nologo /std:c11 /utf-8 /W4 /WX /TC /I"{ROOT}" '
                f'"{harness}" "{source}" /Fe"{executable}"'
            )
            build = subprocess.run(
                compile_command,
                cwd=temp_dir,
                capture_output=True,
                text=True,
                errors="replace",
                check=False,
                shell=True,
                executable=os.environ["ComSpec"],
            )
            self.assertEqual(build.returncode, 0, (build.stdout or "") + build.stderr)
            run = subprocess.run(
                [str(executable)], capture_output=True, text=True, check=False
            )
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)


if __name__ == "__main__":
    unittest.main()
