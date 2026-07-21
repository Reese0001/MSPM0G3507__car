from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


def read(relative_path: str) -> str:
    return (PROJECT / relative_path).read_text(encoding="utf-8")


class SchedulerContractTests(unittest.TestCase):
    def test_main_loop_runs_scheduler_before_tracking_and_safety(self):
        main = read("empty.c")
        loop = re.search(r"while\s*\(\s*1\s*\)\s*\{(.*?)\n\s*\}", main, re.DOTALL)
        self.assertIsNotNone(loop)
        body = loop.group(1)
        self.assertIn("Scheduler_Run();", body)
        calls = [
            body.index("Scheduler_Run();"),
            body.index("LineWalking();"),
            body.index("Motor_Safety_Service();"),
            body.index("delay_ms(10);"),
        ]
        self.assertEqual(calls, sorted(calls))

    def test_euler_sampling_has_one_dmp_read_and_no_delay(self):
        source = read("BSP/MPU6050/app_mpu6050.c")
        match = re.search(
            r"void\s+Get_EulerAngles\s*\(\s*void\s*\)\s*\{(.*?)\n\s*\}",
            source,
            re.DOTALL,
        )
        self.assertIsNotNone(match)
        body = match.group(1)
        self.assertEqual(body.count("mpu_dmp_get_data("), 1)
        self.assertNotIn("delay_ms", body)

    def test_sensor_and_odometry_task_periods_are_preserved(self):
        task_source = read("BSP/Task/task.c")
        for interval, callback in (
            (5, "Get_EulerAngles"),
            (15, "Get_Odometry"),
            (30, "Get_CalibratedAngles"),
        ):
            self.assertRegex(
                task_source,
                rf"\{{\s*{interval}\s*,\s*0\s*,\s*{callback}\s*\}}",
            )


if __name__ == "__main__":
    unittest.main()
