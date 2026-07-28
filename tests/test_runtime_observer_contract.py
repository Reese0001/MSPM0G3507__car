from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class RuntimeObserverContract(unittest.TestCase):
    def test_observer_owns_runtime_log_events(self):
        header = (ROOT / "app/log/runtime_observer.h").read_text(encoding="utf-8")
        source = (ROOT / "app/log/runtime_observer.c").read_text(encoding="utf-8")
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        for token in (
            "RuntimeObserverInputs",
            "RuntimeObserver_Init",
            "RuntimeObserver_Update",
            '"TEST RUN"',
            '"UART TIMEOUT"',
            '"WATCHDOG"',
        ):
            self.assertIn(token, header + source)
        self.assertNotIn("RuntimeLog_Push(now_ms", tasks)
        self.assertIn("RuntimeObserver_Update", tasks)
        self.assertNotIn('"TEST RUN"', tasks)


if __name__ == "__main__":
    unittest.main()
