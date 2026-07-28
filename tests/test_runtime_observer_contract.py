from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class RuntimeObserverContract(unittest.TestCase):
    def test_observer_owns_runtime_log_events(self):
        header = (ROOT / "app/log/runtime_observer.h").read_text(encoding="utf-8")
        source = (ROOT / "app/log/runtime_observer.c").read_text(encoding="utf-8")
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        for token in (
            "RuntimeObserver_Init",
            "RuntimeObserver_MarkSafetyLoop",
            "RuntimeObserver_MarkSensorFrame",
            "RuntimeObserver_MarkControlRequest",
            "RuntimeObserver_Update",
            '"TEST RUN"',
            '"SENSOR WAIT"',
            '"SAFETY RUN"',
            '"MOTOR ARMED"',
            '"UART TIMEOUT"',
            '"WATCHDOG"',
            "SafetyRuntime_IsSensorHeartbeatMissing",
            "AppMailbox_ReadImu",
            "Mpu6050_GetState",
            "LineCascadeControl_IsImuUsed",
            '"IMU READY"',
            '"IMU DEG"',
            '"IMU U Y+000 G+000"',
            "format_signed_3",
            "LineRecovery_GetDiagnostics",
            "yaw_delta_deg",
            '"LINE SEEK L"',
            '"LINE SEEK R"',
            '"LINE ALIGN"',
            '"LINE SAFE STOP"',
            '"LINE FOLLOW"',
        ):
            self.assertIn(token, header + source)
        self.assertNotIn('"LINE SEEK F"', source)
        self.assertNotIn('"LINE SEEK ROT"', source)
        self.assertNotIn("RuntimeLog_Push(now_ms", tasks)
        self.assertIn("RuntimeObserver_Update", tasks)
        self.assertNotIn('"TEST RUN"', tasks)


if __name__ == "__main__":
    unittest.main()
