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

    def test_native_poll_scheduler_is_the_only_active_scheduler(self):
        self.assertIn("AppTasks_Init", self.tasks)
        self.assertIn("AppTasks_Poll", self.tasks)
        self.assertIn("{APP_TASK_SAFETY, safety_task, APP_TASK_BASE_TICK_MS", self.tasks)
        self.assertIn("{APP_TASK_SENSOR, sensor_task, 2U * APP_TASK_BASE_TICK_MS", self.tasks)
        self.assertIn("{APP_TASK_DISPLAY, display_task, 100U * APP_TASK_BASE_TICK_MS", self.tasks)
        for name in ("SensorTask", "ControlTask", "SafetyTask", "DisplayTask"):
            self.assertNotIn(name, self.tasks)
        self.assertNotIn("vTaskDelay", self.tasks)
        self.assertNotIn("AppScheduler_", self.tasks + self.entry + self.main)

    def test_microsecond_services_use_32_bit_timebase(self):
        line_motion = (PROJECT / "app/line/line_motion.c").read_text(encoding="utf-8")
        self.assertIn('TIMER2.$name              = "MICROSECOND_TIMEBASE"', self.syscfg)
        self.assertIn('TIMER2.peripheral.$assign = "TIMG12"', self.syscfg)
        self.assertIn("BSP_I2C_Service(BSP_Time_GetUs())", self.tasks)
        self.assertNotRegex(line_motion, r"while\s*\(")

    def test_boot_stays_disarmed_until_safety_task_arms(self):
        safety_runtime = (PROJECT / "app/safety/safety_runtime.c").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("Motor_Safety_Arm", self.main)
        self.assertIn("AppTasks_Init();", self.entry)
        self.assertIn("AppTasks_Poll(Get_Time());", self.entry)
        self.assertNotIn("vTaskStartScheduler();", self.entry)
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
