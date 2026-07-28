from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class FaultDiagnosticsContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tasks = (ROOT / "app/tasks/app_tasks.c").read_text(
            encoding="utf-8"
        )
        cls.dashboard_h = (
            ROOT / "modules/display/dashboard.h"
        ).read_text(encoding="utf-8")
        cls.dashboard = (
            ROOT / "modules/display/dashboard.c"
        ).read_text(encoding="utf-8")
        cls.config = (
            ROOT / "config/line_lookup_config.h"
        ).read_text(encoding="utf-8")
        cls.lookup = (
            ROOT / "modules/line_tracking/controller/line_lookup_control.c"
        ).read_text(encoding="utf-8")

    def test_supervision_constants_exist(self):
        self.assertIn("LINE_LOOKUP_HIGH_YAW_DPS (95.0f)", self.config)
        self.assertIn("LINE_LOOKUP_IMU_DEGRADED_LIMIT (280)", self.config)
        self.assertIn("APP_CONTROL_HEARTBEAT_TIMEOUT_MS (30U)", self.config)
        self.assertIn("APP_SENSOR_HEARTBEAT_TIMEOUT_MS (20U)", self.config)

    def test_stale_imu_caps_lookup_output(self):
        self.assertIn("LINE_LOOKUP_IMU_DEGRADED_LIMIT", self.lookup)
        self.assertNotIn("yaw_pid", self.lookup)
        self.assertNotIn("integral", self.lookup)

    def test_fault_codes_cover_latching_and_recoverable_classes(self):
        for code in (
            "APP_FAULT_NONE",
            "APP_FAULT_CORNER_SEARCH",
            "APP_FAULT_LINE_LOST",
            "APP_FAULT_OLED_I2C",
            "APP_FAULT_MOTOR_UART",
            "APP_FAULT_CONTROL_HEARTBEAT",
            "APP_FAULT_SENSOR_HEARTBEAT",
        ):
            self.assertIn(code, self.dashboard_h)
        for label in (
            '"C-SEARCH"',
            '"L-LOST"',
            '"OLED-I2C"',
            '"M-UART"',
            '"CTRL-HB"',
            '"SENS-HB"',
        ):
            self.assertIn(label, self.dashboard)

    def test_heartbeat_expiry_latches_and_stops(self):
        safety_body = self.tasks[self.tasks.index("static void SafetyTask"):]
        safety_body = safety_body[: self.tasks.index("static void DisplayTask")]

        self.assertIn("APP_SENSOR_HEARTBEAT_TIMEOUT_MS", self.tasks)
        self.assertIn("APP_FAULT_SENSOR_HEARTBEAT", self.tasks)
        self.assertIn("APP_FAULT_MOTOR_UART", self.tasks)
        self.assertNotIn("APP_CONTROL_HEARTBEAT_TIMEOUT_MS", safety_body)
        self.assertNotIn("APP_FAULT_CONTROL_HEARTBEAT", safety_body)
        self.assertRegex(
            self.tasks,
            r"latched_fault\s*!=\s*APP_FAULT_NONE[\s\S]{0,400}LED_ON\(\)",
        )

    def test_display_failure_stays_recoverable(self):
        display_body = re.search(
            r"static void DisplayTask\(void \*argument\)\s*\{([\s\S]*?)\n\}",
            self.tasks,
        )
        self.assertIsNotNone(display_body)
        self.assertIn("Ssd1306_Init()", display_body.group(1))
        self.assertIn('RuntimeLog_Push(now_ms, "OLED FAIL")', display_body.group(1))
        self.assertNotIn("Motor_Safety_Disarm", display_body.group(1))


if __name__ == "__main__":
    unittest.main()
