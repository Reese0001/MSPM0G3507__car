from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class OledContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.syscfg = (ROOT / "empty.syscfg").read_text(encoding="utf-8")
        cls.bsp = (ROOT / "modules/display/i2c/oled_i2c.c").read_text(encoding="utf-8")
        cls.bsp_h = (ROOT / "modules/display/i2c/oled_i2c.h").read_text(encoding="utf-8")
        cls.ssd = (ROOT / "modules/display/ssd1306/ssd1306.c").read_text(
            encoding="utf-8"
        )
        cls.ssd_h = (ROOT / "modules/display/ssd1306/ssd1306.h").read_text(
            encoding="utf-8"
        )
        cls.runtime_log = (
            ROOT / "modules/display/runtime_log.c"
        ).read_text(encoding="utf-8")
        cls.tasks = (ROOT / "app/tasks/app_tasks.c").read_text(
            encoding="utf-8"
        )
        cls.observer = (ROOT / "app/log/runtime_observer.c").read_text(
            encoding="utf-8"
        )
        cls.cproject = (ROOT / ".cproject").read_text(encoding="utf-8")

    def test_pa10_pa11_become_oled_gpio_and_uart0_is_gone(self):
        self.assertNotIn('"UART_0"', self.syscfg)
        self.assertNotIn("UART_0_INST", self.syscfg)
        # Pin names are globally unique in SysConfig, so the OLED lines are
        # CLK/DAT; MPU6050_I2C keeps SCL/SDA.
        self.assertRegex(
            self.syscfg,
            r'OLED_I2C[\s\S]{0,400}"CLK";[\s\S]{0,80}"PA10"',
        )
        self.assertRegex(
            self.syscfg,
            r'OLED_I2C[\s\S]{0,700}"DAT";[\s\S]{0,80}"PA11"',
        )
        self.assertRegex(
            self.syscfg,
            r'MPU6050_I2C[\s\S]{0,400}"SCL";[\s\S]{0,80}"PA12"',
        )
        self.assertIn("debug_uart.c", self.cproject)

    def test_oled_bus_is_open_drain_software_i2c(self):
        self.assertIn("OLED_I2C_PORT", self.bsp)
        self.assertIn("DL_GPIO_disableOutput", self.bsp)
        self.assertIn("DL_GPIO_enableOutput", self.bsp)
        self.assertNotIn("delay_ms", self.bsp)

    def test_ssd1306_uses_fixed_address_and_page_buffer(self):
        combined = self.ssd + self.ssd_h
        self.assertIn("#define SSD1306_ADDRESS_7BIT 0x3CU", combined)
        self.assertIn("#define SSD1306_WIDTH 128U", combined)
        self.assertIn("#define SSD1306_PAGES 8U", combined)
        self.assertIn("dirty", self.ssd)
        self.assertIn("bool Ssd1306_Init(void)", combined)
        self.assertIn(
            "bool Ssd1306_WritePage(uint8_t page, const uint8_t data[128])",
            combined,
        )

    def test_display_task_observes_and_renders_runtime_log_at_100ms(self):
        self.assertIn(
            "{APP_TASK_DISPLAY, display_task, 100U * APP_TASK_BASE_TICK_MS",
            self.tasks,
        )
        self.assertIn("RuntimeObserver_Update", self.tasks)
        self.assertIn("RuntimeLog_Draw", self.observer)
        self.assertNotIn("Dashboard_Render", self.tasks)
        self.assertIn("UART TIMEOUT", self.observer)
        self.assertIn("WATCHDOG", self.observer)
        self.assertIn("DIR WAIT", self.observer)
        for state in (
            "LINE SEEK L",
            "LINE SEEK R",
            "LINE ALIGN",
            "LINE SAFE STOP",
            "LINE FOLLOW",
        ):
            self.assertIn(state, self.observer)
        self.assertNotIn("LINE SEEK F", self.observer)
        self.assertNotIn("LINE SEEK ROT", self.observer)
        self.assertIn("LineRecovery_GetDiagnostics", self.observer)
        self.assertIn("yaw_delta_deg", self.observer)
        self.assertIn("LineCascadeControl_IsImuUsed() ? 'U' : 'B'", self.observer)
        self.assertIn("SAFETY RUN", self.observer)
        self.assertIn("MOTOR ARMED", self.observer)
        self.assertNotIn('"MOTOR ARM"', self.observer)
        self.assertIn("RuntimeLog_PushMotor", self.observer)

    def test_oled_failure_never_stops_the_motors(self):
        display_body = re.search(
            r"static void display_task\(uint32_t now_ms\)\n\{([\s\S]*?)\n\}",
            self.tasks,
        )
        self.assertIsNotNone(display_body)
        self.assertNotIn("Motor_Safety_Disarm", display_body.group(1))
        self.assertNotIn("Motor_Safety_RequestSpeed", display_body.group(1))
        self.assertNotIn("Motor_Safety_Arm", display_body.group(1))
        for text in (self.bsp, self.ssd, self.runtime_log):
            self.assertNotIn("Motor_Safety_Disarm", text)
            self.assertNotIn("motor_safety.h", text)


if __name__ == "__main__":
    unittest.main()
