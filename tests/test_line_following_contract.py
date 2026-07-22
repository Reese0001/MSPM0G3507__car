from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


def read(relative: str) -> str:
    return (PROJECT / relative).read_text(encoding="utf-8")


class LineFollowingContractTests(unittest.TestCase):
    def test_main_runs_tracking_and_safety_service(self):
        main = read("empty.c")
        app_main = read("application/app_main.c")
        scheduler = read("application/app_scheduler.c")
        questions = read("application/legacy_questions/questions.c")
        self.assertIn("App_Main_RunOnce();", main)
        self.assertIn("AppScheduler_Run(Get_Time());", app_main)
        self.assertIn("Motor_Safety_Service();", app_main)
        self.assertLess(
            app_main.index("AppScheduler_Run(Get_Time());"),
            app_main.index("Motor_Safety_Service();"),
        )
        self.assertIn("Legacy_Questions_HandleKey", scheduler)
        self.assertIn("LineWalking();", questions)

    def test_tracking_routes_motion_through_safety_layer(self):
        tracking = read("modules/line_tracking/app_irtracking.c")
        motor = read("modules/motor/app_motor.c")
        self.assertIn("Motion_Car_Control", tracking)
        self.assertNotIn("Contrl_Speed(", tracking)
        self.assertIn("Motor_Safety_RequestSpeed", motor)

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
