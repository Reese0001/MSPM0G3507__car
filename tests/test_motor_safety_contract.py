from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


def read(relative_path: str) -> str:
    return (PROJECT / relative_path).read_text(encoding="utf-8")


class MotorSafetyContractTests(unittest.TestCase):
    def test_watchdog_latch_marks_system_stall_on_d1(self):
        source = read("modules/motor/safety/motor_safety.c")
        init = source[source.index("void Motor_Safety_Init"):]
        init = init[:init.index("void Motor_Safety_Arm")]
        tick = source[source.index("void Motor_Safety_Tick1ms"):]
        tick = tick[:tick.index("uint8_t Motor_Safety_IsFaultLatched")]
        self.assertIn('#include "../../led/led.h"', source)
        self.assertIn("LED_OFF();", init)
        self.assertIn("LED_ON();", tick)
        self.assertLess(tick.index("MOTOR_SAFETY_FAULT_LATCHED"),
                        tick.index("LED_ON();"))

    def test_safety_module_and_limits_exist(self):
        header = read("modules/motor/safety/motor_safety.h")
        source = read("modules/motor/safety/motor_safety.c")
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
        app_main = read("app/boot/app_boot.c")
        self.assertIn("AppBoot_Init();", main)
        self.assertIn("AppTasks_Create()", main)
        self.assertIn("vTaskStartScheduler();", main)
        self.assertIn("Motor_Safety_Disarm();", main)
        self.assertIn("Set_Motor(5)", app_main)
        self.assertNotIn("Set_Motor(1)", app_main)
        self.assertNotIn("motor_init_count", app_main)
        self.assertNotIn("Motor_Safety_Arm()", app_main)
        self.assertNotIn("Motor_Safety_Service()", app_main)

    def test_timer_starts_irq_and_ticks_watchdog(self):
        timer_c = read("modules/time/timer.c")
        timer_h = read("modules/time/timer.h")
        app_main = read("app/boot/app_boot.c")
        self.assertIn("Timer_Init", timer_h)
        self.assertIn("Timer_Init", timer_c)
        self.assertIn("NVIC_EnableIRQ", timer_c)
        self.assertIn("DL_TimerG_startCounter", timer_c)
        self.assertIn("BSP_Time_RegisterTick1ms", timer_h)
        self.assertIn("Motor_Safety_Tick1ms", app_main)

    def test_motion_commands_are_routed_through_safety(self):
        source = read("modules/motor/configuration/motor_configuration.c")
        header = read("modules/motor/configuration/motor_configuration.h")
        adapter = read("modules/motor/adapter/motor_adapter.c")
        self.assertNotIn("Motion_Car_Control", source + header)
        self.assertNotIn("Motion_Yaw_Calc", source + header)
        self.assertNotIn("Get_Odometry", source + header)
        self.assertNotIn("Motor_Safety_RequestSpeed", source)
        self.assertIn("Motor_Safety_RequestSpeed", adapter)

    def test_isr_stop_is_fixed_and_bounded(self):
        source = read("modules/motor/uart/motor_uart.c")
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
