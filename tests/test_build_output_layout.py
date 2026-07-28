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

    def test_firmware_build_uses_only_the_freertos_task_entry(self):
        makefile = (PROJECT / "Makefile").read_text(encoding="utf-8")
        cproject = (PROJECT / ".cproject").read_text(encoding="utf-8")
        for old_source in (
            "application/app_scheduler.c",
            "application/corner_maneuver.c",
            "application/motion_primitives.c",
        ):
            self.assertNotIn(old_source, makefile)
            self.assertIn(old_source, cproject)
        self.assertIn("app/tasks/app_tasks.c", makefile)


if __name__ == "__main__":
    unittest.main()
