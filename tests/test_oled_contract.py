from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class OledContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.syscfg = (ROOT / "empty.syscfg").read_text(encoding="utf-8")
        cls.bsp = (ROOT / "bsp/bsp_oled_i2c.c").read_text(encoding="utf-8")
        cls.bsp_h = (ROOT / "bsp/bsp_oled_i2c.h").read_text(encoding="utf-8")
        cls.ssd = (ROOT / "modules/display/ssd1306.c").read_text(
            encoding="utf-8"
        )
        cls.ssd_h = (ROOT / "modules/display/ssd1306.h").read_text(
            encoding="utf-8"
        )
        cls.dashboard = (
            ROOT / "application/diagnostics/dashboard.c"
        ).read_text(encoding="utf-8")
        cls.tasks = (ROOT / "application/freertos/app_tasks.c").read_text(
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

    def test_display_task_runs_at_200ms_and_renders_dashboard(self):
        self.assertIn("pdMS_TO_TICKS(200U)", self.tasks)
        self.assertIn("Dashboard_Render", self.tasks)
        self.assertIn("void Dashboard_Render(const AppDiagnostics", self.dashboard)

    def test_oled_failure_never_stops_the_motors(self):
        display_body = re.search(
            r"static void DisplayTask\(void \*argument\)\s*\{([\s\S]*?)\n\}",
            self.tasks,
        )
        self.assertIsNotNone(display_body)
        self.assertNotIn("Motor_Safety_Disarm", display_body.group(1))
        for text in (self.bsp, self.ssd, self.dashboard):
            self.assertNotIn("Motor_Safety_Disarm", text)
            self.assertNotIn("motor_safety.h", text)


if __name__ == "__main__":
    unittest.main()
