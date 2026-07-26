from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class AppSchedulerPipelineContract(unittest.TestCase):
    def setUp(self):
        self.source = (ROOT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )

    def test_line_pipeline_order_and_motion_ownership(self):
        calls = [
            "LineFeatureExtractor_Update",
            "LineEstimator_Update",
            "LineTrendDetector_Update",
            "LineEventClassifier_Update",
            "LineController_Step",
            "CornerManeuver_Step",
            "LineRecovery_Step",
        ]
        positions = [self.source.index(call) for call in calls]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("corner_output.owns_motion", self.source)
        self.assertIn("mission_request = corner_output.request", self.source)

    def test_all_new_modules_reset_at_start_and_corner_completion(self):
        for token in (
            "LineFeatureExtractor_Reset",
            "LineEventClassifier_Reset",
            "CornerManeuver_Reset",
            "LineTrendDetector_Reset",
            "LineController_Reset",
            "LineRecovery_Reset",
        ):
            self.assertGreaterEqual(self.source.count(token), 2)

    def test_corner_fault_fails_closed_without_latching(self):
        self.assertIn("corner_output.fault", self.source)
        self.assertNotIn("LINE_RECOVERY_FAULT", self.source)
        self.assertRegex(
            self.source,
            r"if\s*\(corner_fault\)\s*\{\s*"
            r"[\s\S]{0,240}AppScheduler_ResetLineControlHistory\(\);",
        )
        self.assertRegex(
            self.source,
            r"mission_request\.left_speed\s*=\s*0;[\s\S]{0,120}"
            r"mission_request\.right_speed\s*=\s*0;",
        )
        self.assertIn("LED_ON()", self.source)
        self.assertIn("LED_OFF()", self.source)

    def test_latched_faults_are_checked_outside_ready_pipeline(self):
        self.assertIn(
            "CornerManeuver_GetState() == CORNER_MANEUVER_FAULT",
            self.source,
        )

    def test_only_motor_fault_latches_control(self):
        self.assertIn("static bool control_fault_latched = false", self.source)
        self.assertRegex(
            self.source,
            r"if\s*\(Motor_Safety_IsFaultLatched\(\)\s*!=\s*0U\)\s*\{"
            r"\s*control_fault_latched\s*=\s*true;",
        )
        self.assertNotRegex(
            self.source,
            r"if\s*\(corner_fault[\s\S]{0,80}"
            r"control_fault_latched\s*=\s*true;",
        )
        self.assertNotIn("recovery_fault", self.source)
        self.assertRegex(
            self.source,
            r"if\s*\(control_fault_latched\)\s*\{[\s\S]{0,240}"
            r"mission_request\.valid\s*=\s*false;[\s\S]{0,120}LED_ON\(\);",
        )
        self.assertRegex(
            self.source,
            r"static void AppScheduler_Start\(void\)[\s\S]{0,700}"
            r"control_fault_latched\s*=\s*false;",
        )

    def test_integration_keeps_confirmed_pd_and_speed_limits(self):
        config = (
            ROOT / "application/config/line_control_config.h"
        ).read_text(encoding="utf-8")
        safety = (
            ROOT / "application/config/safety_config.h"
        ).read_text(encoding="utf-8")
        self.assertIn("LINE_CONTROL_KP (28.0f)", config)
        self.assertIn("LINE_MAX_FORWARD (400)", config)
        self.assertIn("SAFETY_RUNNING_SPEED_LIMIT (450)", safety)

    def test_optional_mpu6050_is_serviced_and_fails_soft(self):
        for token in (
            "Mpu6050_Init",
            "BSP_I2C_Service",
            "Mpu6050_Service",
            "Mpu6050_GetSnapshot",
            "MPU6050_STATE_CALIBRATING",
            "ModuleStatus_IsFresh",
            "LINE_FOLLOWING_IMU_DEGRADED_LIMIT",
            "CornerManeuver_StepWithYaw",
        ):
            self.assertIn(token, self.source)
        self.assertIn(
            "inputs.imu_required = LINE_FOLLOWING_REQUIRE_IMU != 0",
            self.source,
        )
        self.assertNotIn(
            "inputs.imu_required = LINE_FOLLOWING_USE_IMU != 0",
            self.source,
        )


class RecoveryReachabilityRuntime(unittest.TestCase):
    def test_lost_follow_reaches_recovery_without_corner_authority_starvation(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        harness = ROOT.parent / "tests/recovery_reachability_harness.c"
        corner = ROOT / "application/corner_maneuver.c"
        recovery = ROOT / "application/line_recovery.c"

        self.assertTrue(vsdevcmd.exists(), "Visual Studio host toolchain missing")
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "recovery_reachability_harness.exe"
            compile_command = (
                f'call "{vsdevcmd}" -arch=x64 >nul && '
                f'cl /nologo /W4 /TC /I"{ROOT}" "{harness}" '
                f'"{corner}" "{recovery}" /Fe"{executable}"'
            )
            build = subprocess.run(
                compile_command,
                capture_output=True,
                text=True,
                errors="replace",
                check=False,
                shell=True,
                executable=os.environ["ComSpec"],
            )
            self.assertEqual(
                build.returncode, 0, (build.stdout or "") + build.stderr
            )
            run = subprocess.run(
                [str(executable)], capture_output=True, text=True, check=False
            )
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)


if __name__ == "__main__":
    unittest.main()
