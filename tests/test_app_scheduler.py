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

    def test_task_scheduler_is_fixed_timeslice_table(self):
        for token in (
            "APP_TASK_BASE_TICK_MS",
            "AppTaskSlot",
            "app_task_slots[]",
            "APP_TASK_SAFETY",
            "APP_TASK_SENSOR",
            "APP_TASK_CONTROL",
            "APP_TASK_DISPLAY",
            "AppTaskFunction",
            "run_task_slot",
        ):
            self.assertIn(token, self.tasks)
        self.assertLess(
            self.tasks.index("APP_TASK_SAFETY"),
            self.tasks.index("APP_TASK_SENSOR"),
        )
        self.assertNotIn("APP_SAFETY_PERIOD_MS", self.tasks)
        self.assertNotIn("APP_SENSOR_PERIOD_MS", self.tasks)
        self.assertNotIn("else if (id ==", self.tasks)
        self.assertIn("last_run_ms = now_ms", self.tasks)
        self.assertNotIn("last_run_ms +=", self.tasks)

    def test_line_request_pipeline_order_is_owned_by_line_motion(self):
        self.assertIn("BuildOfficialBaselineRequest", self.line)
        self.assertIn("BuildAssistedRequest", self.line)
        self.assertIn(
            "#if LINE_FOLLOWING_CONTROL_MODE == "
            "LINE_CONTROL_MODE_OFFICIAL_BASELINE",
            self.line,
        )
        self.assertLess(
            self.line.index("LineOfficialControl_Step"),
            self.line.rindex("LineRecovery_Step"),
        )
        self.assertIn("ClearRequest(now_ms, request)", self.line)

    def test_control_publishes_only_valid_line_requests(self):
        self.assertIn("AppLineMotion_BuildRequest", self.control)
        self.assertIn("RunController_IsRunRequested", self.control)
        self.assertIn("return false;", self.control)
        self.assertLess(
            self.control.index("AppLineMotion_BuildRequest"),
            self.control.index("AppMailbox_PublishMotionRequest"),
        )

    def test_safety_runtime_is_the_single_motor_authority_bridge(self):
        self.assertIn("AppMailbox_ReadMotionRequest", self.safety)
        self.assertIn("StopRequest(now_ms, &request)", self.safety)
        self.assertNotIn("RunController_BuildRequest", self.safety)
        self.assertIn("SafetySupervisor_Step", self.safety)
        self.assertIn("MotorAdapter_Apply", self.safety)
        self.assertIn("Motor_Safety_Arm", self.safety)
        self.assertNotIn("LineStartGate_Update", self.safety + self.tasks)

    def test_integration_keeps_damped_pd_and_speed_limits(self):
        config = (ROOT / "config/line_cascade_config.h").read_text(encoding="utf-8")
        safety = (ROOT / "config/safety_config.h").read_text(encoding="utf-8")
        self.assertIn("LINE_CASCADE_POSITION_KP (14.0f)", config)
        self.assertIn("LINE_CASCADE_MAX_COMMAND (140)", config)
        self.assertIn("SAFETY_RUNNING_SPEED_LIMIT (450)", safety)

    def test_official_baseline_uses_only_fresh_ybimu_z_rate(self):
        for token in (
            "YbImu_Init",
            "YbImu_Service",
            "YbImu_GetSnapshot",
            "gyro_rad_s[2]",
            "YBIMU_RAD_TO_DEG",
            "YBIMU_STALE_TIMEOUT_MS",
            "LineOfficialControl_Step",
        ):
            self.assertIn(token, self.line)
        official = self.line[
            self.line.index("BuildOfficialBaselineRequest") :
            self.line.index("BuildAssistedRequest")
        ]
        for forbidden in ("euler_deg", "mag_uT", "quat", "yaw_angle_deg"):
            self.assertNotIn(forbidden, official)
        self.assertIn("BSP_I2C_Service", self.tasks)

    def test_assisted_mpu6050_fallback_remains_available(self):
        for token in (
            "Mpu6050_Init",
            "Mpu6050_Service",
            "Mpu6050_GetSnapshot",
            "MPU6050_STATE_CALIBRATING",
            "ModuleStatus_IsFresh",
            "LineCascadeControl_Step",
            "yaw_angle_deg",
        ):
            self.assertIn(token, self.line)
        self.assertIn(
            "inputs.imu_required = LINE_FOLLOWING_REQUIRE_IMU != 0",
            self.safety,
        )

    def test_sensor_services_imu_state_machine_every_timeslice(self):
        self.assertIn("AppLineMotion_ServiceImu(now_ms);", self.sensor)
        self.assertNotIn("SENSOR_RUNTIME_IMU_SERVICE_CYCLES", self.sensor)
        self.assertNotIn("imu_cycle", self.sensor)


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

