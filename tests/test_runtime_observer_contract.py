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
            "AppMailbox_ReadLineSample",
            "LineOfficialControl_GetDiagnostics",
            "YbImu_GetSnapshot",
            '"IMU BYPASS"',
            '"B%02X P%+d D%c"',
            '"G%+04d C%+04d I%u"',
            '"CMD %03d/%03d"',
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
        self.assertNotIn('"IMU U Y+000 G+000"', source)
        self.assertNotIn("RuntimeLog_Push(now_ms", tasks)
        self.assertIn("RuntimeObserver_Update", tasks)
        self.assertNotIn('"TEST RUN"', tasks)

    def test_baseline_diagnostics_do_not_consume_absolute_attitude(self):
        source = (ROOT / "app/log/runtime_observer.c").read_text(encoding="utf-8")
        start = source.index("static bool observe_baseline")
        baseline = source[start : source.index("#else", start)]
        for forbidden in (
            "yaw_angle_deg",
            "euler_deg",
            "mag_uT",
            "quat",
            "magnetic_heading_healthy",
        ):
            self.assertNotIn(forbidden, baseline)


if __name__ == "__main__":
    unittest.main()
