from pathlib import Path
import re
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

    def test_state_machine_is_non_blocking(self):
        source = (ROOT / "modules/ultrasonic/ultrasonic.c").read_text(
            encoding="utf-8"
        )
        header = (ROOT / "modules/ultrasonic/ultrasonic.h").read_text(
            encoding="utf-8"
        )
        for state in (
            "ULTRA_IDLE",
            "ULTRA_TRIGGER_HIGH",
            "ULTRA_WAIT_RISE",
            "ULTRA_WAIT_FALL",
        ):
            self.assertIn(state, source)
        for api in (
            "Ultrasonic_Init",
            "Ultrasonic_Service",
            "Ultrasonic_OnEchoEdge",
        ):
            self.assertIn(api, header)
        self.assertNotIn("delay_ms", source)
        self.assertNotRegex(source, r"while\s*\(")
        self.assertNotIn("Contrl_Speed", source)
        self.assertNotIn("Motor_", source)

    def test_echo_callback_only_captures_edge_state(self):
        source = (ROOT / "modules/ultrasonic/ultrasonic.c").read_text(
            encoding="utf-8"
        )
        signature = "void Ultrasonic_OnEchoEdge"
        self.assertIn(signature, source)
        body = source[source.index(signature):]
        self.assertNotIn("Ultrasonic_PulseUsToMm", body)
        self.assertNotIn("BSP_Ultrasonic_SetTrig", body)
        self.assertNotIn("snapshot", body)
        self.assertIn("captured_ready", body)
        self.assertIn("state = ULTRA_IDLE", body)

    def test_bsp_boundary_is_zero_hardware_stub(self):
        header_path = ROOT / "bsp/bsp_ultrasonic.h"
        source_path = ROOT / "bsp/bsp_ultrasonic.c"
        self.assertTrue(header_path.exists(), header_path)
        self.assertTrue(source_path.exists(), source_path)
        combined = header_path.read_text(encoding="utf-8") + source_path.read_text(
            encoding="utf-8"
        )
        for api in (
            "BSP_Ultrasonic_Init",
            "BSP_Ultrasonic_NowUs",
            "BSP_Ultrasonic_SetTrig",
        ):
            self.assertIn(api, combined)
        self.assertNotIn("ti_msp_dl_config.h", combined)
        self.assertNotIn("PA26", combined)
        self.assertNotIn("PA27", combined)


if __name__ == "__main__":
    unittest.main()
