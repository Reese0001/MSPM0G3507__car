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
        cls.observer = (ROOT / "app/log/runtime_observer.c").read_text(
            encoding="utf-8"
        )
        cls.safety_runtime = (
            ROOT / "app/safety/safety_runtime.c"
        ).read_text(encoding="utf-8")
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

    def test_motor_uart_fault_latches_and_stops(self):
        self.assertIn("APP_SENSOR_HEARTBEAT_TIMEOUT_MS", self.safety_runtime)
        self.assertIn("APP_FAULT_MOTOR_UART", self.safety_runtime)
        self.assertNotIn("APP_CONTROL_HEARTBEAT_TIMEOUT_MS", self.safety_runtime)
        self.assertNotIn("APP_FAULT_CONTROL_HEARTBEAT", self.safety_runtime)
        self.assertIn("latched_fault = APP_FAULT_MOTOR_UART", self.safety_runtime)
        self.assertRegex(
            self.safety_runtime,
            r"latched_fault\s*!=\s*APP_FAULT_NONE[\s\S]{0,400}LED_ON\(\)",
        )

    def test_sensor_heartbeat_is_diagnostic_not_motor_stop(self):
        self.assertIn("sensor_heartbeat_missing", self.safety_runtime)
        self.assertIn("SafetyRuntime_IsSensorHeartbeatMissing", self.safety_runtime)
        self.assertNotIn("latched_fault = APP_FAULT_SENSOR_HEARTBEAT", self.safety_runtime)
        self.assertIn('RuntimeLog_Push(now_ms, "SENSOR WAIT")', self.observer)

    def test_display_failure_stays_recoverable(self):
        self.assertIn("Ssd1306_Init()", self.observer)
        self.assertIn('RuntimeLog_Push(now_ms, "OLED FAIL")', self.observer)
        self.assertNotIn("Motor_Safety_Disarm", self.observer)
        self.assertNotIn("Motor_Safety_Disarm", self.tasks)


if __name__ == "__main__":
    unittest.main()
