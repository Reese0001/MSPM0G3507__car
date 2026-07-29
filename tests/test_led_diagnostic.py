from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class LedDiagnosticContract(unittest.TestCase):
    def test_standalone_led_test_avoids_timer_and_motor_dependencies(self):
        source = (ROOT / "diagnostics/led_alive_test.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("SYSCFG_DL_init();", source)
        self.assertIn("LED_D1_PIN", source)
        self.assertIn("LED_D2_PIN", source)
        self.assertIn("DL_Common_delayCycles", source)
        self.assertNotIn("Set_Motor", source)
        self.assertNotIn("Motor_", source)
        self.assertNotIn("Get_Time", source)
        self.assertNotIn("Timer_Init", source)


if __name__ == "__main__":
    unittest.main()
