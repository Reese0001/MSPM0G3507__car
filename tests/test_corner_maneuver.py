import re
import os
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class CornerManeuverContract(unittest.TestCase):
    def setUp(self):
        self.header = (ROOT / "application/corner_maneuver.h").read_text(
            encoding="utf-8"
        )
        self.source = (ROOT / "application/corner_maneuver.c").read_text(
            encoding="utf-8"
        )
        self.config = (
            ROOT / "application/config/corner_maneuver_config.h"
        ).read_text(encoding="utf-8")

    def test_states_cover_probe_brake_commit_seek_settle(self):
        for token in (
            "CORNER_MANEUVER_FORWARD_PROBE",
            "CORNER_MANEUVER_BRAKE",
            "CORNER_MANEUVER_COMMIT",
            "CORNER_MANEUVER_SEEK",
            "CORNER_MANEUVER_SETTLE",
            "CORNER_MANEUVER_FAULT",
        ):
            self.assertIn(token, self.header)

    def test_probe_is_forward_only_and_bounded(self):
        self.assertIn("CORNER_PROBE_COMMAND (100)", self.config)
        self.assertIn("CORNER_PROBE_MAX_MS (80U)", self.config)
        self.assertRegex(
            self.source,
            r"publish_request\(CORNER_PROBE_COMMAND,\s*"
            r"CORNER_PROBE_COMMAND",
        )

    def test_left_and_right_pivots_are_mirrors(self):
        self.assertIn("CORNER_INNER_COMMAND (-80)", self.config)
        self.assertIn("CORNER_OUTER_COMMAND (120)", self.config)
        self.assertIn("corner_direction < 0", self.source)

    def test_completion_requires_three_new_centered_frames(self):
        self.assertIn("CORNER_REACQUIRE_FRAMES (3U)", self.config)
        self.assertIn("last_feature_sequence", self.source)
        self.assertIn("reacquire_frames", self.source)

    def test_every_state_has_a_total_timeout(self):
        self.assertIn("CORNER_TOTAL_TIMEOUT_MS (2000U)", self.config)
        self.assertIn("enter_fault", self.source)

    def test_follow_request_rejects_whole_vehicle_reverse(self):
        self.assertRegex(
            self.source,
            r"if\s*\(left\s*<\s*0\s*&&\s*right\s*<\s*0\)"
            r"[\s\S]{0,160}invalidate_request",
        )


class CornerManeuverRuntime(unittest.TestCase):
    def test_host_harness_exercises_corner_state_machine(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        harness = ROOT.parent / "tests/corner_maneuver_harness.c"
        source = ROOT / "application/corner_maneuver.c"

        self.assertTrue(vsdevcmd.exists(), "Visual Studio host toolchain missing")
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "corner_maneuver_harness.exe"
            compile_command = (
                f'call "{vsdevcmd}" -arch=x64 >nul && '
                f'cl /nologo /std:c11 /utf-8 /W4 /WX /TC /I"{ROOT}" '
                f'"{harness}" "{source}" '
                f'/Fe"{executable}"'
            )
            build = subprocess.run(
                compile_command,
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
