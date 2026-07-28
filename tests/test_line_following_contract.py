from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


def read(relative: str) -> str:
    return (PROJECT / relative).read_text(encoding="utf-8")


class LineFollowingContractTests(unittest.TestCase):
    def test_main_starts_native_poll_loop_with_motors_disarmed(self):
        main = read("empty.c")
        boot = read("app/boot/app_boot.c")
        tasks = read("app/tasks/app_tasks.c")
        self.assertIn("AppTasks_Init();", main)
        self.assertIn("AppTasks_Poll(Get_Time());", main)
        self.assertNotIn("vTaskStartScheduler();", main)
        self.assertNotIn("AppScheduler_", main + boot + tasks)
        self.assertNotIn("Motor_Safety_Arm()", boot)
        self.assertIn("SafetyRuntime_Step", tasks)

    def test_line_control_routes_motion_through_safety_runtime(self):
        control = read("app/control/control_runtime.c")
        safety = read("app/safety/safety_runtime.c")
        adapter = read("modules/motor/adapter/motor_adapter.c")
        motor_config = read("modules/motor/configuration/motor_configuration.c")
        self.assertIn("AppLineMotion_BuildRequest", control)
        self.assertIn("AppMailbox_PublishMotionRequest", control)
        self.assertIn("SafetySupervisor_Step", safety)
        self.assertIn("MotorAdapter_Apply", safety)
        self.assertIn("Motor_Safety_RequestSpeed", adapter)
        self.assertNotIn("Motion_Car_Control", motor_config + control + safety)

    def test_two_wheel_mapping_keeps_m1_and_m3_stopped(self):
        adapter = read("modules/motor/adapter/motor_adapter.c")
        self.assertIn("Motor_Safety_RequestSpeed(0, decision->right_speed", adapter)
        self.assertIn("decision->left_speed)", adapter)
        self.assertIn("Motor_Safety_RequestSpeed(0, 0, 0, 0)", adapter)

    def test_timer_refreshes_motor_watchdog(self):
        timer = read("modules/time/timer.c")
        boot = read("app/boot/app_boot.c")
        self.assertIn("tick_callback();", timer)
        self.assertIn("Motor_Safety_Tick1ms();", boot)
        self.assertIn("BSP_Time_RegisterTick1ms", boot)

    def test_l520_configuration_is_selected(self):
        self.assertIn("Set_Motor(5);", read("app/boot/app_boot.c"))


if __name__ == "__main__":
    unittest.main()
