from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


def read(relative_path: str) -> str:
    return (PROJECT / relative_path).read_text(encoding="utf-8")


class MotorSafetyContractTests(unittest.TestCase):
    def test_safety_module_and_limits_exist(self):
        header = read("modules/motor/motor_safety.h")
        source = read("modules/motor/motor_safety.c")
        combined = header + source
        for value in ("1000", "10", "30", "200", "100"):
            self.assertIn(value, combined)
        for symbol in (
            "Motor_Safety_Init",
            "Motor_Safety_Arm",
            "Motor_Safety_RequestSpeed",
            "Motor_Safety_Service",
            "Motor_Safety_Tick1ms",
        ):
            self.assertIn(symbol, combined)

    def test_main_initializes_timer_and_uses_l520(self):
        main = read("empty.c")
        self.assertIn("Set_Motor(5)", main)
        self.assertNotIn("Set_Motor(1)", main)
        self.assertNotIn("motor_init_count", main)
        calls = [
            main.index("Timer_Init()"),
            main.index("Motor_Safety_Init()"),
            main.index("Motor_Safety_Arm()"),
        ]
        self.assertEqual(calls, sorted(calls))
        self.assertIn("Motor_Safety_Service()", main)

    def test_timer_starts_irq_and_ticks_watchdog(self):
        timer_c = read("bsp/time/timer.c")
        timer_h = read("bsp/time/timer.h")
        self.assertIn("Timer_Init", timer_h)
        self.assertIn("Timer_Init", timer_c)
        self.assertIn("NVIC_EnableIRQ", timer_c)
        self.assertIn("DL_TimerG_startCounter", timer_c)
        self.assertIn("Motor_Safety_Tick1ms", timer_c)

    def test_motion_commands_are_routed_through_safety(self):
        source = read("modules/motor/app_motor.c")
        for function in ("Motion_Car_Control", "Motion_Yaw_Calc"):
            match = re.search(
                rf"void\s+{function}\s*\([^)]*\)\s*\{{(.*?)\n\}}",
                source,
                re.DOTALL,
            )
            self.assertIsNotNone(match, function)
            self.assertIn("Motor_Safety_RequestSpeed", match.group(1))

    def test_isr_stop_is_fixed_and_bounded(self):
        source = read("modules/motor/bsp_motor_usart.c")
        match = re.search(
            r"Motor_EmergencyStop_FromISR\s*\([^)]*\)\s*\{(.*?)\n\}",
            source,
            re.DOTALL,
        )
        self.assertIsNotNone(match)
        body = match.group(1)
        self.assertIn("$spd:0,0,0,0#", body)
        self.assertNotIn("sprintf", body)
        self.assertNotIn("strlen", body)


if __name__ == "__main__":
    unittest.main()
