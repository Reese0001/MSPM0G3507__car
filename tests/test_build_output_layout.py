from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class BuildOutputLayoutContract(unittest.TestCase):
    def test_cli_and_uniflash_outputs_use_normalized_paths(self):
        makefile = (PROJECT / "Makefile").read_text(encoding="utf-8")
        gitignore = (ROOT / ".gitignore").read_text(encoding="utf-8")
        self.assertIn(
            "BUILD_DIR  := ../build/cli/MSPM0G3507_LineFollowing_Car",
            makefile,
        )
        self.assertIn("FIRMWARE_DIR := ../dist/firmware", makefile)
        self.assertIn("dist/", gitignore)

    def test_firmware_build_uses_only_the_native_task_entry(self):
        makefile = (PROJECT / "Makefile").read_text(encoding="utf-8")
        cproject = (PROJECT / ".cproject").read_text(encoding="utf-8")
        for old_source in (
            "application/app_scheduler.c",
            "application/corner_maneuver.c",
            "application/motion_primitives.c",
            "modules/optional/competition/corner_maneuver.c",
            "modules/optional/competition/motion_primitives.c",
            "modules/optional/legacy/tracking/app_irtracking.c",
            "modules/line_tracking/line_controller.c",
            "modules/line_tracking/line_estimator.c",
            "modules/line_tracking/line_event_classifier.c",
            "modules/line_tracking/line_features.c",
            "modules/line_tracking/line_trend_detector.c",
            "modules/line_tracking/decoder/line_start_gate.c",
        ):
            self.assertNotIn(old_source, makefile)
        for excluded in ("application", "modules/optional/competition", "modules/optional/legacy"):
            self.assertIn(excluded, cproject)
        self.assertIn("app/tasks/app_tasks.c", makefile)
        self.assertNotIn("freertos_kernel_ticlang.lib", makefile)


if __name__ == "__main__":
    unittest.main()
