from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class ModularArchitectureContract(unittest.TestCase):
    def test_motion_request_contract_exists(self):
        text = (ROOT / "modules/common/motion_request.h").read_text(encoding="utf-8")
        for token in ("MotionRequest", "left_speed", "right_speed", "timestamp_ms", "valid"):
            self.assertIn(token, text)

    def test_module_status_contract_exists(self):
        text = (ROOT / "modules/common/module_status.h").read_text(encoding="utf-8")
        for token in ("timestamp_ms", "sequence", "valid", "health", "ModuleStatus_IsFresh"):
            self.assertIn(token, text)

    def test_required_roots_exist(self):
        for name in ("application", "modules", "bsp"):
            self.assertTrue((ROOT / name).is_dir(), name)

    def test_legacy_bsp_is_removed(self):
        directory_names = {path.name for path in ROOT.iterdir() if path.is_dir()}
        self.assertNotIn("BSP", directory_names)

    def test_lower_layers_do_not_include_application(self):
        for base in (ROOT / "modules", ROOT / "bsp"):
            for path in base.rglob("*.[ch]"):
                text = path.read_text(encoding="utf-8", errors="ignore")
                self.assertNotIn('#include "application/', text, str(path))

    def test_lower_layers_do_not_include_application_only_headers(self):
        application = ROOT / "application"
        application_headers = {
            path.name for path in application.rglob("*.h")
        }
        for base in (ROOT / "modules", ROOT / "bsp"):
            for path in base.rglob("*.[ch]"):
                text = path.read_text(encoding="utf-8", errors="ignore")
                for header in re.findall(r'^\s*#include\s+"([^/"]+)"', text, re.MULTILINE):
                    if header not in application_headers:
                        continue
                    matches = list(ROOT.rglob(header))
                    self.assertTrue(matches, header)
                    self.assertFalse(
                        all(candidate.is_relative_to(application) for candidate in matches),
                        f"{path} includes application-only header {header}",
                    )

    def test_application_owns_legacy_key_event_policy(self):
        key_source = (ROOT / "modules/key/key.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        questions_source = (ROOT / "application/legacy_questions/questions.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        task_source = (ROOT / "application/legacy_task/task.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        self.assertIn("Key_PollEvent", key_source)
        self.assertNotIn("State_Machine", key_source)
        self.assertIn("Legacy_Questions_HandleKey", questions_source)
        self.assertIn("Key_PollEvent", questions_source)
        self.assertIn("AppScheduler_Run", task_source)

    def test_main_delegates_to_application(self):
        main = (ROOT / "empty.c").read_text(encoding="utf-8")
        self.assertIn("App_Main_Init();", main)
        self.assertIn("App_Main_RunOnce();", main)
        self.assertNotIn("LineWalking();", main)
        self.assertNotIn("delay_ms(", main)

    def test_application_scheduler_contract_exists(self):
        header = (ROOT / "application/app_scheduler.h").read_text(encoding="utf-8")
        source = (ROOT / "application/app_scheduler.c").read_text(encoding="utf-8")
        for token in (
            "AppTaskFn",
            "period_ms",
            "last_ms",
            "AppScheduler_Init",
            "AppScheduler_Run",
        ):
            self.assertIn(token, header)
        self.assertIn("now_ms - task->last_ms", source)
        self.assertNotIn("delay_ms(", source)

    def test_application_main_keeps_motor_disarmed(self):
        source = (ROOT / "application/app_main.c").read_text(encoding="utf-8")
        init_start = source.index("void App_Main_Init")
        run_start = source.index("void App_Main_RunOnce")
        init_body = source[init_start:run_start]
        run_body = source[run_start:]
        self.assertIn("Timer_Init()", init_body)
        self.assertIn("Motor_Safety_Init()", init_body)
        self.assertIn("Set_Motor(5)", init_body)
        self.assertIn("AppScheduler_Init(Get_Time())", init_body)
        self.assertNotIn("Motor_Safety_Arm()", source)
        self.assertIn("uint32_t now_ms = Get_Time()", run_body)
        self.assertIn("AppScheduler_Run(now_ms)", run_body)
        self.assertIn("Motor_Safety_Service()", run_body)
        self.assertLess(
            run_body.index("AppScheduler_Run(now_ms)"),
            run_body.index("Motor_Safety_Service()"),
        )

    def test_scheduled_euler_update_is_non_blocking(self):
        source = (ROOT / "modules/legacy_mpu6050/driver/app_mpu6050.c").read_text(
            encoding="utf-8"
        )
        match = re.search(
            r"void\s+Get_EulerAngles\s*\([^)]*\)\s*\{(.*?)\n\}",
            source,
            re.DOTALL,
        )
        self.assertIsNotNone(match)
        self.assertNotIn("delay_ms(", match.group(1))

    def test_default_scheduler_does_not_sample_uninitialized_legacy_mpu(self):
        scheduler = (ROOT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )
        mpu = (ROOT / "modules/legacy_mpu6050/driver/app_mpu6050.c").read_text(
            encoding="utf-8"
        )
        euler = re.search(
            r"void\s+Get_EulerAngles\s*\([^)]*\)\s*\{(.*?)\n\}",
            mpu,
            re.DOTALL,
        )
        self.assertNotIn("Get_EulerAngles", scheduler)
        self.assertIsNotNone(euler)
        self.assertIn("float p = pitch", euler.group(1))
        self.assertIn("if (mpu_dmp_get_data", euler.group(1))

    def test_bsp_time_uses_registered_tick_without_module_dependencies(self):
        timer_h = (ROOT / "bsp/time/timer.h").read_text(encoding="utf-8")
        timer_c = (ROOT / "bsp/time/timer.c").read_text(encoding="utf-8")
        app_main = (ROOT / "application/app_main.c").read_text(encoding="utf-8")
        for forbidden in ("motor_safety.h", "buzzer.h", "app_mpu6050.h", "debug_uart.h"):
            self.assertNotIn(forbidden, timer_h + timer_c)
        self.assertIn("BSP_Time_RegisterTick1ms", timer_h)
        self.assertIn("BSP_Time_RegisterTick1ms", timer_c)
        self.assertIn("tick_callback()", timer_c)
        self.assertIn("Motor_Safety_Tick1ms();", app_main)
        self.assertIn("Buzzer_Handle();", app_main)
        self.assertIn("BSP_Time_RegisterTick1ms", app_main)
        self.assertLess(
            app_main.index("Motor_Safety_Init()"),
            app_main.index("BSP_Time_RegisterTick1ms"),
        )

    def test_automatic_start_uses_non_blocking_safety_services(self):
        scheduler = (ROOT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )
        questions = (ROOT / "application/legacy_questions/questions.c").read_text(
            encoding="utf-8"
        )
        buzzer = (ROOT / "modules/buzzer/buzzer.c").read_text(encoding="utf-8")
        app_main = (ROOT / "application/app_main.c").read_text(encoding="utf-8")
        safety = (ROOT / "modules/motor/motor_safety.c").read_text(
            encoding="utf-8"
        )
        handler = re.search(
            r"void\s+Legacy_Questions_HandleKey\s*\([^)]*\)\s*\{(.*?)\n\}",
            questions,
            re.DOTALL,
        )
        buzzer_service = re.search(
            r"void\s+Buzzer_Handle\s*\([^)]*\)\s*\{(.*?)\n\}",
            buzzer,
            re.DOTALL,
        )
        disarm = re.search(
            r"void\s+Motor_Safety_Disarm\s*\([^)]*\)\s*\{(.*?)\n\}",
            safety,
            re.DOTALL,
        )
        safety_service = re.search(
            r"void\s+Motor_Safety_Service\s*\([^)]*\)\s*\{(.*?)\n\}",
            safety,
            re.DOTALL,
        )
        self.assertNotIn("Key_PollEvent();", scheduler)
        self.assertIn("AppScheduler_Start();", scheduler)
        self.assertNotIn("Motor_Safety_Disarm();", scheduler)
        self.assertIsNotNone(handler)
        self.assertIn("Buzzer_RequestBeeps", handler.group(1))
        self.assertIn("Motor_Safety_Disarm", handler.group(1))
        for forbidden in ("Contrl_Pwm(", "Contrl_Speed(", "Beep_Times(", "delay_ms(", "while"):
            self.assertNotIn(forbidden, handler.group(1))
        self.assertIsNotNone(buzzer_service)
        self.assertNotIn("delay_ms(", buzzer_service.group(1))
        self.assertNotIn("while", buzzer_service.group(1))
        self.assertIn("Buzzer_Handle();", app_main)
        self.assertIsNotNone(disarm)
        self.assertIn("safety_state = MOTOR_SAFETY_DISARMED", disarm.group(1))
        self.assertIn("clear_requested_speed()", disarm.group(1))
        self.assertIn("stop_pending = 1U", disarm.group(1))
        self.assertIsNotNone(safety_service)
        self.assertIn("if (stop_pending != 0U)", safety_service.group(1))
        self.assertIn("apply_speed(0, 0, 0, 0)", safety_service.group(1))


if __name__ == "__main__":
    unittest.main()
