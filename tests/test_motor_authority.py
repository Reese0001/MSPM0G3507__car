from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class MotorAuthorityContract(unittest.TestCase):
    def test_only_adapter_calls_motor_safety_speed(self):
        callers = []
        for path in ROOT.rglob("*.c"):
            if path.name == "motor_safety.c":
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            if re.search(r"Motor_Safety_RequestSpeed\s*\(", text):
                callers.append(path.as_posix())
        self.assertEqual(1, len(callers), callers)
        self.assertTrue(callers[0].endswith("modules/motor/motor_adapter.c"))

    def test_adapter_maps_only_m2_and_m4(self):
        source = (ROOT / "modules/motor/motor_adapter.c").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "Motor_Safety_RequestSpeed(0, decision->left_speed,",
            source,
        )
        self.assertIn("0, decision->right_speed)", source)
        self.assertIn("MOTOR_ADAPTER_MAX_COMMAND (450)", source)

    def test_boot_path_remains_disarmed(self):
        app_main = (ROOT / "application/app_main.c").read_text(encoding="utf-8")
        self.assertNotIn("Motor_Safety_Arm()", app_main)


if __name__ == "__main__":
    unittest.main()
