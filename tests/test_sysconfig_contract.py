from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class SysConfigContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.syscfg = (PROJECT / "empty.syscfg").read_text(encoding="utf-8")
        cls.tracking_h = (PROJECT / "modules/line_tracking/scanner/four_line_scanner.c").read_text(
            encoding="utf-8"
        )

    def test_gpio_four_channel_tracking_pins(self):
        for assignment in ('"PA24"', '"PA25"', '"PA26"', '"PA27"'):
            self.assertIn(assignment, self.syscfg)
        for symbol in ("FOUR_LINE_X1_PIN", "FOUR_LINE_X2_PIN", "FOUR_LINE_X3_PIN", "FOUR_LINE_X4_PIN"):
            self.assertIn(symbol, self.tracking_h)

    def test_motor_uart_pins_and_baud(self):
        self.assertIn('UART2.peripheral.txPin.$assign         = "PB6"', self.syscfg)
        self.assertIn('UART2.peripheral.rxPin.$assign         = "PB7"', self.syscfg)
        self.assertIn("UART2.targetBaudRate                   = 115200", self.syscfg)

    def test_pa10_pa11_serve_the_oled_not_debug_uart(self):
        self.assertNotIn('"UART_0"', self.syscfg)
        self.assertIn('"PA10"', self.syscfg)
        self.assertIn('"PA11"', self.syscfg)
        self.assertIn("OLED_I2C", self.syscfg)

    def test_pa12_pa13_serve_mpu6050_software_i2c(self):
        self.assertIn('GPIO5.$name                         = "MPU6050_I2C"', self.syscfg)
        self.assertIn('GPIO5.associatedPins[0].pin.$assign = "PA12"', self.syscfg)
        self.assertIn('GPIO5.associatedPins[1].pin.$assign = "PA13"', self.syscfg)

    def test_watchdog_timer_period_is_one_millisecond(self):
        self.assertIn('TIMER1.timerPeriod        = "1 ms"', self.syscfg)
        self.assertIn('TIMER1.timerMode          = "PERIODIC"', self.syscfg)

    def test_other_unfitted_modules_do_not_initialize_peripherals(self):
        for token in (
            "ULTRASONIC_TRIG",
            "ULTRASONIC_ECHO",
            'UART3.$name                    = "K230"',
            '"PA21"',
            '"PA22"',
        ):
            self.assertNotIn(token, self.syscfg)


if __name__ == "__main__":
    unittest.main()
