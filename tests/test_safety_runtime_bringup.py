from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class SafetyRuntimeBringupContract(unittest.TestCase):
    def test_motor_arm_depends_on_motor_config_not_motion_tasks(self):
        source = (ROOT / "app/safety/safety_runtime.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        arm = source[source.index("static void ArmWhenReady"):
                     source.index("void SafetyRuntime_Init")]
        self.assertIn("AppBoot_IsMotorConfigured()", arm)
        self.assertIn("Motor_Safety_IsFaultLatched()", arm)
        self.assertIn("!faulted", arm)
        self.assertNotIn("BootTrace_MotionTasksOnline", arm)

    def test_runtime_exposes_last_decision_for_oled_debug(self):
        header = (ROOT / "app/safety/safety_runtime.h").read_text(
            encoding="utf-8", errors="ignore"
        )
        observer = (ROOT / "app/log/runtime_observer.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        for token in (
            "SafetyRuntimeDiagnostics",
            "last_request",
            "last_decision",
            "arm_waiting_for_config",
            "SafetyRuntime_GetDiagnostics",
        ):
            self.assertIn(token, header)
        for token in (
            "SafetyRuntime_GetDiagnostics",
            '"ARM WAIT CFG"',
            '"REQ L"',
            '"APPROVED"',
            '"REJECT REQ"',
            '"REJECT MOTOR"',
            '"REJECT POWER"',
        ):
            self.assertIn(token, observer)


if __name__ == "__main__":
    unittest.main()
