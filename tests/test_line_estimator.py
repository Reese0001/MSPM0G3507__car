from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class LineScannerContract(unittest.TestCase):
    def test_scanner_is_nonblocking_and_publishes_atomic_snapshot(self):
        source = (ROOT / "modules/line_tracking/line_scanner.c").read_text(
            encoding="utf-8"
        )
        header = (ROOT / "modules/line_tracking/line_scanner.h").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("delay_ms", source)
        self.assertNotIn("delay_us", source)
        self.assertNotRegex(source, r"while\s*\(")
        for token in (
            "LINE_SCAN_SELECT",
            "LINE_SCAN_SETTLE",
            "LINE_SCAN_SAMPLE",
            "LineSensorSnapshot",
            "ModuleStatus status",
            "black_bits",
            "LineScanner_Service",
            "LineScanner_GetSnapshot",
        ):
            self.assertIn(token, source + header)

    def test_mux_settle_time_is_tunable_and_initially_ten_us(self):
        config = (ROOT / "modules/line_tracking/line_tracking_config.h").read_text(
            encoding="utf-8"
        )
        self.assertRegex(config, r"LINE_MUX_SETTLE_US\s+\(10U\)")

    def test_bsp_owns_confirmed_gray_mux_pins(self):
        source = (ROOT / "bsp/bsp_line_mux.c").read_text(encoding="utf-8")
        for pin in ("DL_GPIO_PIN_15", "DL_GPIO_PIN_16", "DL_GPIO_PIN_17", "DL_GPIO_PIN_18"):
            self.assertIn(pin, source)
        self.assertEqual(source.count("DL_GPIO_writePinsVal("), 1)
        self.assertNotIn("delay_", source)


if __name__ == "__main__":
    unittest.main()
