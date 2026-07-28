from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class ModularArchitectureContract(unittest.TestCase):
    def test_shared_contract_headers_exist(self):
        motion = (ROOT / "shared/motion_request.h").read_text(encoding="utf-8")
        status = (ROOT / "shared/module_status.h").read_text(encoding="utf-8")
        for token in ("MotionRequest", "left_speed", "right_speed", "timestamp_ms", "valid"):
            self.assertIn(token, motion)
        for token in ("timestamp_ms", "sequence", "valid", "health", "ModuleStatus_IsFresh"):
            self.assertIn(token, status)

    def test_required_roots_exist_after_split(self):
        for name in ("app", "config", "modules", "shared"):
            self.assertTrue((ROOT / name).is_dir(), name)

    def test_optional_modules_are_quarantined_under_one_root(self):
        optional = ROOT / "modules/optional"
        for name in ("k230", "ultrasonic", "ybimu", "legacy"):
            self.assertTrue((optional / name).is_dir(), name)
        for name in ("k230_link", "ultrasonic", "ybimu", "legacy_mpu6050"):
            self.assertFalse((ROOT / "modules" / name).exists(), name)

    def test_main_delegates_to_boot_and_tasks(self):
        main = (ROOT / "empty.c").read_text(encoding="utf-8")
        self.assertIn("AppBoot_Init();", main)
        self.assertIn("AppTasks_Init();", main)
        self.assertIn("AppTasks_Poll(Get_Time());", main)
        self.assertNotIn("vTaskStartScheduler();", main)
        self.assertNotIn("App_Main_Init();", main)
        self.assertNotIn("delay_ms(", main)

    def test_boot_keeps_motor_disarmed_and_registers_tick(self):
        boot = (ROOT / "app/boot/app_boot.c").read_text(encoding="utf-8")
        self.assertIn("void AppBoot_Init", boot)
        self.assertIn("Timer_Init()", boot)
        self.assertIn("Motor_Safety_Init()", boot)
        self.assertIn("Set_Motor(5)", boot)
        self.assertIn("BSP_Time_RegisterTick1ms", boot)
        self.assertNotIn("Motor_Safety_Arm()", boot)

    def test_time_module_owns_registered_tick_api(self):
        timer_h = (ROOT / "modules/time/timer.h").read_text(encoding="utf-8")
        timer_c = (ROOT / "modules/time/timer.c").read_text(encoding="utf-8")
        for token in ("Timer_Init", "BSP_Time_GetUs", "BSP_Time_RegisterTick1ms"):
            self.assertIn(token, timer_h + timer_c)
        self.assertIn("tick_callback()", timer_c)

    def test_active_paths_use_new_split_locations(self):
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        safety_runtime = (
            ROOT / "app/safety/safety_runtime.c"
        ).read_text(encoding="utf-8")
        mailbox = (ROOT / "app/mailbox/app_mailbox.h").read_text(encoding="utf-8")
        safety = (ROOT / "app/safety/safety_supervisor.h").read_text(encoding="utf-8")
        self.assertIn(
            "inputs.start_pressed = RunController_IsRunRequested();",
            safety_runtime,
        )
        self.assertNotIn("LineStartGate_Update", tasks)
        self.assertIn("SafetySupervisor_Step", safety_runtime + safety)
        self.assertIn("shared/motion_request.h", mailbox.replace("\\", "/"))
        self.assertIn("shared/safety_decision.h", safety.replace("\\", "/"))

    def test_lower_layers_do_not_include_old_application_paths(self):
        for base in (ROOT / "modules", ROOT / "shared"):
            for path in base.rglob("*.[ch]"):
                text = path.read_text(encoding="utf-8", errors="ignore")
                self.assertNotIn('#include "application/', text, str(path))


if __name__ == "__main__":
    unittest.main()
