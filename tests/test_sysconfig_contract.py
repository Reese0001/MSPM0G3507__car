from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class SysConfigContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.syscfg = (PROJECT / "empty.syscfg").read_text(encoding="utf-8")
        cls.tracking_h = (PROJECT / "modules/line_tracking/app_irtracking.h").read_text(
            encoding="utf-8"
        )

    def test_gpio_multiplexer_tracking_pins(self):
        for assignment in ('"PA15"', '"PA16"', '"PA17"', '"PA18"'):
            self.assertIn(assignment, self.syscfg)
        for symbol in ("GRAY_AD0", "GRAY_AD1", "GRAY_AD2", "GRAY_OUT"):
            self.assertIn(symbol, self.tracking_h)

    def test_motor_uart_pins_and_baud(self):
        self.assertIn('UART2.peripheral.txPin.$assign         = "PB6"', self.syscfg)
        self.assertIn('UART2.peripheral.rxPin.$assign         = "PB7"', self.syscfg)
        self.assertIn("UART2.targetBaudRate                   = 115200", self.syscfg)

    def test_debug_uart_pins_and_baud(self):
        self.assertIn('UART1.peripheral.txPin.$assign = "PA10"', self.syscfg)
        self.assertIn('UART1.peripheral.rxPin.$assign = "PA11"', self.syscfg)
        self.assertIn("UART1.targetBaudRate           = 115200", self.syscfg)

    def test_watchdog_timer_period_is_one_millisecond(self):
        self.assertIn('TIMER1.timerPeriod        = "1 ms"', self.syscfg)
        self.assertIn('TIMER1.timerMode          = "PERIODIC"', self.syscfg)


if __name__ == "__main__":
    unittest.main()
