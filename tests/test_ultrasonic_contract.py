from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class UltrasonicContract(unittest.TestCase):
    def test_public_contract_and_limits(self):
        header_path = ROOT / "modules/ultrasonic/ultrasonic.h"
        config_path = ROOT / "modules/ultrasonic/ultrasonic_config.h"
        self.assertTrue(header_path.exists(), header_path)
        self.assertTrue(config_path.exists(), config_path)

        header = header_path.read_text(encoding="utf-8")
        config = config_path.read_text(encoding="utf-8")
        for token in (
            "UltrasonicSnapshot",
            "ModuleStatus status",
            "distance_mm",
            "pulse_us",
            "Ultrasonic_PulseUsToMm",
            "Ultrasonic_GetSnapshot",
        ):
            self.assertIn(token, header)
        for token in ("60000U", "30000U", "100U", "25000U"):
            self.assertIn(token, config)

    def test_conversion_is_integer_only_and_bounded(self):
        source_path = ROOT / "modules/ultrasonic/ultrasonic.c"
        self.assertTrue(source_path.exists(), source_path)
        source = source_path.read_text(encoding="utf-8")
        self.assertIn("pulse_us < ULTRASONIC_MIN_PULSE_US", source)
        self.assertIn("pulse_us > ULTRASONIC_MAX_PULSE_US", source)
        self.assertIn("(pulse_us * 343U + 1000U) / 2000U", source)
        self.assertNotIn("float", source)
        self.assertNotIn("double", source)


if __name__ == "__main__":
    unittest.main()
