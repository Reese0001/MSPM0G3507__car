from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"
WEIGHTS = (-7, -5, -3, -1, 1, 3, 5, 7)


def weighted_error(bits):
    active = [WEIGHTS[index] for index in range(8) if bits & (1 << index)]
    return None if not active else sum(active) / len(active)


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
        self.assertRegex(config, r"LINE_MUX_SETTLE_US\s+\(50U\)")
        self.assertRegex(config, r"LINE_SENSOR_BLACK_ACTIVE_LEVEL\s+\(1U\)")
        self.assertRegex(config, r"LINE_SENSOR_STALE_MS\s+\(20U\)")

    def test_bsp_owns_confirmed_gray_mux_pins(self):
        source = (ROOT / "bsp/bsp_line_mux.c").read_text(encoding="utf-8")
        for pin in ("DL_GPIO_PIN_15", "DL_GPIO_PIN_16", "DL_GPIO_PIN_17", "DL_GPIO_PIN_18"):
            self.assertIn(pin, source)
        self.assertEqual(source.count("DL_GPIO_writePinsVal("), 1)
        self.assertNotIn("delay_", source)


class LineEstimatorContract(unittest.TestCase):
    def test_all_patterns_are_bounded_and_mirror_symmetric(self):
        for bits in range(1, 256):
            value = weighted_error(bits)
            mirror_bits = int(f"{bits:08b}"[::-1], 2)
            mirror = weighted_error(mirror_bits)
            self.assertGreaterEqual(value, -7)
            self.assertLessEqual(value, 7)
            self.assertAlmostEqual(value + mirror, 0.0)

    def test_estimator_contract_has_trend_confidence_and_events(self):
        header = (ROOT / "modules/line_tracking/line_estimator.h").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "modules/line_tracking/line_estimator.c").read_text(
            encoding="utf-8"
        )
        for token in (
            "LINE_EVENT_NONE",
            "LINE_EVENT_HARD_LEFT",
            "LINE_EVENT_HARD_RIGHT",
            "LINE_EVENT_WIDE_BLACK",
            "LINE_EVENT_LOST",
            "LineEstimate",
            "derivative",
            "predicted_error",
            "confidence",
            "LineEstimator_Update",
        ):
            self.assertIn(token, header + source)
        self.assertIn("ModuleStatus_IsFresh", source)
        self.assertNotIn("Contrl_Speed", source)
        self.assertNotIn("Motion_Car_Control", source)

    def test_empty_and_wide_patterns_have_explicit_events(self):
        source = (ROOT / "modules/line_tracking/line_estimator.c").read_text(
            encoding="utf-8"
        )
        self.assertRegex(source, r"black_bits\s*==\s*0U")
        self.assertIn("LINE_EVENT_LOST", source)
        self.assertIn("LINE_EVENT_WIDE_BLACK", source)


class LineControllerContract(unittest.TestCase):
    def test_normal_follow_correction_never_reverses_inner_wheel(self):
        for base_speed in range(1, 451):
            correction_limit = base_speed * 80 // 100
            self.assertGreaterEqual(base_speed - correction_limit, 0)

    def test_hard_corner_has_separate_low_speed_pivot_commands(self):
        source = (ROOT / "modules/line_tracking/line_controller.c").read_text(
            encoding="utf-8"
        )
        config = (ROOT / "application/config/line_control_config.h").read_text(
            encoding="utf-8"
        )
        recovery = (ROOT / "application/line_recovery.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("LINE_HARD_TURN_FORWARD (40)", config)
        self.assertIn("LINE_HARD_TURN_COMMAND (120)", config)
        self.assertIn("hard_turn_forward", source)
        self.assertIn("hard_turn_command", source)
        self.assertNotIn("if (left < 0 || right < 0)", recovery)

    def test_single_sensor_turn_keeps_both_wheels_within_command_limit(self):
        source = (ROOT / "modules/line_tracking/line_controller.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("absolute_int16(turn)", source)
        self.assertIn("control_config.max_forward - turn_magnitude", source)

    def test_predictive_controller_has_speed_planning_and_slew_limits(self):
        header = (ROOT / "modules/line_tracking/line_controller.h").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "modules/line_tracking/line_controller.c").read_text(
            encoding="utf-8"
        )
        config = (ROOT / "application/config/line_control_config.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "LineControlOutput",
            "forward",
            "turn",
            "valid",
            "LineController_Step",
        ):
            self.assertIn(token, header)
        for token in (
            "predicted_error",
            "confidence",
            "yaw_rate_dps",
            "LINE_TURN_LIMIT_PERCENT",
            "LINE_TURN_SLEW_STEP",
            "LINE_ACCEL_STEP",
            "LINE_DECEL_STEP",
        ):
            self.assertIn(token, source + config)
        for token in (
            "LINE_MAX_FORWARD (450)",
            "LINE_CURVE_FORWARD (270)",
            "LINE_HARD_CURVE_FORWARD (180)",
            "LINE_WIDE_BLACK_FORWARD (120)",
            "LINE_LOW_CONFIDENCE_FORWARD (150)",
            "LINE_ACCEL_STEP (15)",
            "LINE_DECEL_STEP (45)",
        ):
            self.assertIn(token, config)
        self.assertIn("LINE_TURN_LIMIT_PERCENT (80)", config)
        self.assertIn("LINE_TURN_SLEW_STEP (20)", config)
        self.assertIn("turn_slew_step", header)
        self.assertIn("previous_turn", source)
        self.assertIn("slew_turn", source)
        self.assertNotIn("Contrl_Speed", source)
        self.assertNotIn("Motion_Car_Control", source)
        self.assertNotIn("application/", source)
        self.assertIn("LineController_Init", header + source)

    def test_controller_fails_closed_on_lost_or_invalid_estimate(self):
        source = (ROOT / "modules/line_tracking/line_controller.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("LINE_EVENT_LOST", source)
        self.assertIn("output->valid = false", source)


if __name__ == "__main__":
    unittest.main()
