"""Regression checks for the accepted car-platform baseline."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "docs" / "setup" / "CAR_PLATFORM_CONTRACT.md"
README = ROOT / "README.md"


class CarPlatformContractTests(unittest.TestCase):
    def test_contract_states_the_200_ms_motor_safety_watchdog_threshold(self):
        text = " ".join(CONTRACT.read_text(encoding="utf-8").split())

        self.assertIn("MOTOR_SAFETY_WATCHDOG_MS=200 ms", text)

    def test_contract_records_accepted_platform_and_safety_boundaries(self):
        text = " ".join(CONTRACT.read_text(encoding="utf-8").split())

        required = (
            "MSPM0G3507_LineFollowing_Car",
            "L-shaped 520",
            "Hall encoder",
            "228 mm",
            "148 mm",
            "102.15 mm",
            "65 mm",
            "M2 is left drive",
            "M4 is right drive",
            "M1/M3 remain zero",
            "MOTOR_TYPE=5",
            "12 V",
            "11-line AB incremental encoder",
            "300 rpm",
            "PA10/PA11",
            "PB6/PB7",
            "115200",
            "PA15/PA16/PA17",
            "PA18 OUT",
            "black line is low level",
            "0x12",
            "MOTOR_SAFETY_WATCHDOG_MS",
            "Motor Safety",
            "0-to-30% soft-start",
            "nominal 11.1 V",
            "full-charge 12.6 V",
            "about 6 A",
            "5-12 V",
            "hard gate",
            "Stall-current",
            "fuse",
            "quick disconnect",
            "raised-wheel",
            "camera has not been purchased",
            "does not change SysConfig for vision",
            "Encoder pulses/rev",
            "wheel effective diameter",
            "not validated",
        )
        for value in required:
            with self.subTest(value=value):
                self.assertIn(value, text)

    def test_readme_links_to_the_platform_contract_and_camera_boundary(self):
        text = " ".join(README.read_text(encoding="utf-8").split())
        self.assertIn("docs/setup/CAR_PLATFORM_CONTRACT.md", text)
        self.assertIn("camera has not been purchased", text)


if __name__ == "__main__":
    unittest.main()
