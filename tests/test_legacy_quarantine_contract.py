from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class LegacyQuarantineContract(unittest.TestCase):
    def test_old_polling_scheduler_is_removed_from_project_sources(self):
        for rel in (
            "application/app_scheduler.c",
            "application/app_scheduler.h",
            "application/legacy_task/task.c",
            "application/legacy_task/task.h",
        ):
            self.assertFalse((ROOT / rel).exists(), rel)

    def test_old_direct_tracking_and_competition_modules_are_optional_only(self):
        for rel in (
            "modules/line_tracking/app_irtracking.c",
            "modules/line_tracking/app_irtracking.h",
            "application/corner_maneuver.c",
            "application/corner_maneuver.h",
            "application/motion_primitives.c",
            "application/motion_primitives.h",
            "application/legacy_questions/questions.c",
            "application/legacy_questions/questions.h",
        ):
            self.assertFalse((ROOT / rel).exists(), rel)

        for rel in (
            "modules/optional/legacy/tracking/app_irtracking.c",
            "modules/optional/competition/corner_maneuver.c",
            "modules/optional/competition/motion_primitives.c",
            "modules/optional/competition/questions/questions.c",
        ):
            self.assertTrue((ROOT / rel).exists(), rel)

    def test_active_modules_do_not_include_application_or_direct_motion_helpers(self):
        for base in (ROOT / "app", ROOT / "modules"):
            for path in base.rglob("*.[ch]"):
                if "optional" in path.parts:
                    continue
                text = path.read_text(encoding="utf-8", errors="ignore")
                self.assertNotIn("application/", text, str(path))
                self.assertNotIn("Motion_Car_Control", text, str(path))


if __name__ == "__main__":
    unittest.main()
