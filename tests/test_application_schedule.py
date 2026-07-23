from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class ApplicationScheduleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scheduler = (PROJECT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )
        cls.main = (PROJECT / "application/app_main.c").read_text(encoding="utf-8")
        cls.syscfg = (PROJECT / "empty.syscfg").read_text(encoding="utf-8")

    def test_minimal_line_following_services_are_registered(self):
        for call in (
            "LineScanner_Service",
            "LineEstimator_Update",
            "LineController_Step",
            "LineRecovery_Step",
            "SafetySupervisor_Step",
            "MotorAdapter_Apply",
        ):
            self.assertIn(call, self.scheduler)
        for period in ("{1U,", "{5U,", "{LINE_KEY_TASK_PERIOD_MS,"):
            self.assertIn(period, self.scheduler)
        for call in (
            "Ultrasonic_Init",
            "Ultrasonic_Service",
            "YbImu_Init",
            "YbImu_Service",
            "K230Link_Init",
            "K230Link_Service",
            "Get_Odometry",
        ):
            self.assertNotIn(call, self.scheduler)

    def test_microsecond_services_use_32_bit_timebase(self):
        self.assertIn('TIMER2.$name              = "MICROSECOND_TIMEBASE"', self.syscfg)
        self.assertIn('TIMER2.peripheral.$assign = "TIMG12"', self.syscfg)
        self.assertIn("BSP_Time_GetUs", self.scheduler)

    def test_boot_is_disarmed_and_k1_controls_run_state(self):
        self.assertNotIn("Motor_Safety_Arm", self.main)
        init = self.scheduler[self.scheduler.index("void AppScheduler_Init"):]
        init = init[:init.index("void AppScheduler_Run")]
        self.assertNotIn("Motor_Safety_Arm", init)
        self.assertIn("KEY_EVENT_SHORT", self.scheduler)
        self.assertIn("KEY_EVENT_PRESS", self.scheduler)
        self.assertIn("Motor_Safety_Arm", self.scheduler)
        self.assertIn("Motor_Safety_Disarm", self.scheduler)
        self.assertIn(
            "power_qualified = LINE_FOLLOWING_POWER_QUALIFIED",
            self.scheduler,
        )

    def test_safety_tick_runs_before_legacy_low_priority_tasks(self):
        safety = self.scheduler.index("AppScheduler_RunSafety")
        legacy = self.scheduler.index("AppScheduler_RunKey")
        self.assertLess(safety, legacy)

    def test_scheduler_tracks_runtime_and_deadline_misses(self):
        header = (PROJECT / "application/app_scheduler.h").read_text(encoding="utf-8")
        for symbol in (
            "max_runtime_us",
            "deadline_miss_count",
            "AppScheduler_GetDiagnostics",
        ):
            self.assertIn(symbol, header + self.scheduler)


if __name__ == "__main__":
    unittest.main()
