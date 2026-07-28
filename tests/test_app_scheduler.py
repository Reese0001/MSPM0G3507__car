from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class AppRuntimePipelineContract(unittest.TestCase):
    def setUp(self):
        self.tasks = (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        self.sensor = (ROOT / "app/sensor/sensor_runtime.c").read_text(encoding="utf-8")
        self.control = (ROOT / "app/control/control_runtime.c").read_text(encoding="utf-8")
        self.line = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        self.safety = (ROOT / "app/safety/safety_runtime.c").read_text(encoding="utf-8")

    def test_task_shell_delegates_to_small_runtime_modules(self):
        for call in (
            "SensorRuntime_Step",
            "ControlRuntime_RunOnce",
            "SafetyRuntime_Step",
            "RuntimeObserver_Update",
        ):
            self.assertIn(call, self.tasks)
        for forbidden in (
            "LineController_Step",
            "LineRecovery_Step",
            "Motor_Safety_RequestSpeed",
            "Motion_Car_Control",
        ):
            self.assertNotIn(forbidden, self.tasks)

    def test_line_request_pipeline_order_is_owned_by_line_motion(self):
        positions = [
            self.line.index("LinePosition_Reset"),
            self.line.index("LineRecovery_Init"),
            self.line.index("LineLookupControl_Step"),
            self.line.index("LineRecovery_Step"),
        ]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("sample->position.type == LINE_PATTERN_NOISE", self.line)
        self.assertIn("ClearRequest(now_ms, request)", self.line)

    def test_control_publishes_only_valid_line_requests(self):
        self.assertIn("AppLineMotion_BuildRequest", self.control)
        self.assertIn("return false;", self.control)
        self.assertLess(
            self.control.index("AppLineMotion_BuildRequest"),
            self.control.index("AppMailbox_PublishMotionRequest"),
        )

    def test_safety_runtime_is_the_single_motor_authority_bridge(self):
        self.assertIn("RunController_BuildRequest", self.safety)
        self.assertIn("SafetySupervisor_Step", self.safety)
        self.assertIn("MotorAdapter_Apply", self.safety)
        self.assertIn("Motor_Safety_Arm", self.safety)
        self.assertNotIn("LineStartGate_Update", self.safety + self.tasks)

    def test_integration_keeps_confirmed_pd_and_speed_limits(self):
        config = (ROOT / "config/line_control_config.h").read_text(encoding="utf-8")
        safety = (ROOT / "config/safety_config.h").read_text(encoding="utf-8")
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
        ):
            self.assertIn(token, self.line)
        lookup = (ROOT / "modules/line_tracking/controller/line_lookup_control.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("LINE_LOOKUP_IMU_DEGRADED_LIMIT", lookup)
        self.assertIn(
            "inputs.imu_required = LINE_FOLLOWING_REQUIRE_IMU != 0",
            self.safety,
        )


class RecoveryReachabilityRuntime(unittest.TestCase):
    def test_lost_follow_reaches_recovery_without_corner_authority_starvation(self):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        harness = ROOT.parent / "tests/recovery_reachability_harness.c"
        corner = ROOT / "modules/optional/competition/corner_maneuver.c"
        recovery = ROOT / "modules/line_tracking/recovery/line_recovery.c"

        self.assertTrue(vsdevcmd.exists(), "Visual Studio host toolchain missing")
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "recovery_reachability_harness.exe"
            compile_command = (
                f'call "{vsdevcmd}" -arch=x64 >nul && '
                f'cl /nologo /W4 /TC /I"{ROOT}" "{harness}" '
                f'"{corner}" "{recovery}" '
                f'/Fe"{executable}"'
            )
            build = subprocess.run(
                compile_command,
                cwd=temp_dir,
                capture_output=True,
                text=True,
                errors="replace",
                check=False,
                shell=True,
                executable=os.environ["ComSpec"],
            )
            self.assertEqual(build.returncode, 0, (build.stdout or "") + build.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True, check=False)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)


if __name__ == "__main__":
    unittest.main()
