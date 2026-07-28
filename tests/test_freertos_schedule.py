from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class NativeScheduleContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tasks = (ROOT / "app/tasks/app_tasks.c").read_text(
            encoding="utf-8"
        )
        cls.mailbox = (ROOT / "app/mailbox/app_mailbox.c").read_text(
            encoding="utf-8"
        )
        cls.timer = (ROOT / "modules/time/timer.c").read_text(encoding="utf-8")
        cls.safety_runtime = (ROOT / "app/safety/safety_runtime.c").read_text(
            encoding="utf-8"
        )

    def test_native_periods_are_explicit_and_small(self):
        for token in (
            "{APP_TASK_SAFETY, safety_task, APP_TASK_BASE_TICK_MS",
            "{APP_TASK_SENSOR, sensor_task, 2U * APP_TASK_BASE_TICK_MS",
            "{APP_TASK_DISPLAY, display_task, 10U * APP_TASK_BASE_TICK_MS",
            "MOTOR_UART_MIN_PERIOD_MS",
        ):
            self.assertIn(token, self.tasks + self.safety_runtime)

    def test_poll_loop_runs_safety_even_without_sensor_frame(self):
        safety = self.tasks.index("SafetyRuntime_Step")
        sensor = self.tasks.index("SensorRuntime_Step")
        self.assertLess(safety, sensor)
        self.assertIn("sensor_frame_pending", self.tasks)
        self.assertIn("ControlRuntime_RunOnce", self.tasks)

    def test_mailboxes_are_single_thread_latest_value_slots(self):
        self.assertNotIn("FreeRTOS", self.mailbox)
        self.assertNotIn("taskENTER_CRITICAL()", self.mailbox)
        self.assertNotIn("taskEXIT_CRITICAL()", self.mailbox)
        self.assertNotIn("pvPortMalloc", self.mailbox + self.tasks)

    def test_hardware_tick_isr_stays_scheduler_free(self):
        self.assertNotIn("FreeRTOS", self.timer)
        self.assertNotIn("FromISR", self.timer)
        self.assertIn("tick_callback()", self.timer)


if __name__ == "__main__":
    unittest.main()
