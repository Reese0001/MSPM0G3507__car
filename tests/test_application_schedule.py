from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class ApplicationScheduleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tasks = (PROJECT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        cls.main = (PROJECT / "app/boot/app_boot.c").read_text(encoding="utf-8")
        cls.entry = (PROJECT / "empty.c").read_text(encoding="utf-8")
        cls.syscfg = (PROJECT / "empty.syscfg").read_text(encoding="utf-8")

    def test_four_freertos_tasks_are_the_only_active_scheduler(self):
        for name in ("SensorTask", "ControlTask", "SafetyTask", "DisplayTask"):
            self.assertIn(name, self.tasks)
        self.assertIn("vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2U))", self.tasks)
        self.assertIn("vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1U))", self.tasks)
        self.assertIn("vTaskDelay(pdMS_TO_TICKS(100U))", self.tasks)
        self.assertNotIn("static AppTask app_tasks", self.tasks)
        self.assertNotIn("AppScheduler_", self.tasks + self.entry + self.main)

    def test_microsecond_services_use_32_bit_timebase(self):
        line_motion = (PROJECT / "app/line/line_motion.c").read_text(encoding="utf-8")
        self.assertIn('TIMER2.$name              = "MICROSECOND_TIMEBASE"', self.syscfg)
        self.assertIn('TIMER2.peripheral.$assign = "TIMG12"', self.syscfg)
        self.assertIn("BSP_Time_GetUs", line_motion)

    def test_boot_stays_disarmed_until_safety_task_arms(self):
        safety_runtime = (PROJECT / "app/safety/safety_runtime.c").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("Motor_Safety_Arm", self.main)
        self.assertIn("AppTasks_Create()", self.entry)
        self.assertIn("vTaskStartScheduler();", self.entry)
        self.assertIn("Motor_Safety_Disarm();", self.entry)
        self.assertIn("Motor_Safety_Arm", safety_runtime)

    def test_runtime_diagnostics_are_boot_trace_and_oled_log(self):
        boot_trace = (PROJECT / "modules/diagnostics/boot_trace.h").read_text(
            encoding="utf-8"
        )
        observer = (PROJECT / "app/log/runtime_observer.c").read_text(encoding="utf-8")
        for symbol in ("BOOT_TASK_SENSOR", "BOOT_TASK_CONTROL", "BOOT_TASK_SAFETY", "BOOT_TASK_DISPLAY"):
            self.assertIn(symbol, boot_trace)
        for symbol in ("SAFETY RUN", "MOTOR ARMED", "TEST RUN", "RuntimeLog_PushMotor"):
            self.assertIn(symbol, observer)


if __name__ == "__main__":
    unittest.main()
