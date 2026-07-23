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

    def test_ybimu_software_i2c_pins(self):
        self.assertIn('GPIO5.$name                          = "YBIMU_I2C"', self.syscfg)
        self.assertIn('GPIO5.associatedPins[0].pin.$assign  = "PA12"', self.syscfg)
        self.assertIn('GPIO5.associatedPins[1].pin.$assign  = "PA13"', self.syscfg)

    def test_k230_uart2_pins_baud_and_rx_interrupt(self):
        self.assertIn('UART3.peripheral.txPin.$assign = "PA21"', self.syscfg)
        self.assertIn('UART3.peripheral.rxPin.$assign = "PA22"', self.syscfg)
        self.assertIn("UART3.targetBaudRate", self.syscfg)
        self.assertIn('UART3.enabledInterrupts        = ["RX"]', self.syscfg)
        self.assertIn('UART3.peripheral.$suggestSolution          = "UART2"', self.syscfg)

    def test_ultrasonic_trigger_and_echo_capture(self):
        self.assertIn('GPIO6.associatedPins[0].pin.$assign = "PA26"', self.syscfg)
        self.assertIn('CAPTURE1.peripheral.$assign', self.syscfg)
        self.assertIn('CAPTURE1.peripheral.ccp1Pin.$assign = "PA27"', self.syscfg)
        self.assertIn('CAPTURE1.captMode', self.syscfg)
        self.assertIn('"CC1_DN"', self.syscfg)


if __name__ == "__main__":
    unittest.main()
