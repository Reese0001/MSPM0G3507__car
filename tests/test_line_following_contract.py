from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


def read(relative: str) -> str:
    return (PROJECT / relative).read_text(encoding="utf-8")


class LineFollowingContractTests(unittest.TestCase):
    def test_main_starts_static_scheduler_with_motors_disarmed(self):
        main = read("empty.c")
        app_main = read("application/app_main.c")
        scheduler = read("application/app_scheduler.c")
        self.assertIn("AppTasks_Create()", main)
        self.assertIn("vTaskStartScheduler();", main)
        self.assertIn("Motor_Safety_Disarm();", main)
        self.assertNotIn("App_Main_RunOnce", main)
        self.assertNotIn("AppScheduler_Init", app_main)
        self.assertNotIn("Motor_Safety_Arm()", app_main)
        self.assertNotIn("Key_PollEvent", scheduler)
        self.assertIn("AppScheduler_Start();", scheduler)
        self.assertIn("LineRecovery_Step", scheduler)
        self.assertIn("SafetySupervisor_Step", scheduler)

    def test_tracking_routes_motion_through_safety_layer(self):
        tracking = read("modules/line_tracking/app_irtracking.c")
        motor = read("modules/motor/app_motor.c")
        adapter = read("modules/motor/motor_adapter.c")
        self.assertIn("Motion_Car_Control", tracking)
        self.assertNotIn("Contrl_Speed(", tracking)
        self.assertNotIn("Motor_Safety_RequestSpeed", motor)
        self.assertIn("Motor_Safety_RequestSpeed", adapter)

    def test_two_wheel_mapping_keeps_m1_and_m3_stopped(self):
        motor = read("modules/motor/app_motor.c")
        body = re.search(
            r"void\s+Motion_Car_Control\s*\([^)]*\)\s*\{(.*?)\n\}",
            motor,
            re.DOTALL,
        ).group(1)
        self.assertRegex(body, r"speed_L1_setup\s*=\s*0\s*;")
        self.assertRegex(body, r"speed_R1_setup\s*=\s*0\s*;")
        self.assertRegex(body, r"speed_L2_setup\s*=\s*speed_fb\s*\+\s*speed_spin")
        self.assertRegex(body, r"speed_R2_setup\s*=\s*speed_fb\s*-\s*speed_spin")

    def test_timer_refreshes_motor_watchdog(self):
        timer = read("bsp/time/timer.c")
        app_main = read("application/app_main.c")
        self.assertIn("tick_callback();", timer)
        self.assertIn("Motor_Safety_Tick1ms();", app_main)
        self.assertIn("BSP_Time_RegisterTick1ms", app_main)

    def test_l520_configuration_is_selected(self):
        self.assertIn("Set_Motor(5);", read("application/app_main.c"))


if __name__ == "__main__":
    unittest.main()
