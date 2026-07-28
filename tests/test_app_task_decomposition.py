from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"
TASKS = ROOT / "app/tasks/app_tasks.c"


class AppTaskDecompositionContract(unittest.TestCase):
    def setUp(self):
        self.tasks = TASKS.read_text(encoding="utf-8")

    def test_app_tasks_is_only_a_task_shell(self):
        line_count = len(self.tasks.splitlines())
        self.assertLessEqual(line_count, 260)
        self.assertNotIn("BuildBringupRunRequest", self.tasks)
        self.assertNotIn("BuildMotionRequest", self.tasks)
        self.assertNotIn("RuntimeLog_Push(now_ms", self.tasks)

    def test_new_app_modules_exist(self):
        for path in (
            "app/run/run_controller.c",
            "app/run/run_controller.h",
            "app/line/line_motion.c",
            "app/line/line_motion.h",
            "app/log/runtime_observer.c",
            "app/log/runtime_observer.h",
        ):
            self.assertTrue((ROOT / path).is_file(), path)

    def test_makefile_builds_new_app_modules(self):
        makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
        for source in (
            "app/run/run_controller.c",
            "app/line/line_motion.c",
            "app/log/runtime_observer.c",
        ):
            self.assertIn(source, makefile)


if __name__ == "__main__":
    unittest.main()
