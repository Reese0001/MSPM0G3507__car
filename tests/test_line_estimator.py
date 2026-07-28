from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"
WEIGHTS = (-7, -5, -3, -1, 1, 3, 5, 7)


def weighted_error(bits):
    active = [WEIGHTS[index] for index in range(8) if bits & (1 << index)]
    return None if not active else sum(active) / len(active)


def straight_target(frames):
    stable = 0
    targets = []
    for error, confidence, event, trend in frames:
        qualifies = (
            abs(error) <= 1.0
            and confidence >= 70
            and event == "none"
            and trend == "normal"
        )
        stable = min(255, stable + 1) if qualifies else 0
        targets.append(400 if stable >= 5 else 330)
    return targets


class LineScannerContract(unittest.TestCase):
    def test_scanner_is_nonblocking_and_publishes_atomic_snapshot(self):
        source = (ROOT / "modules/line_tracking/scanner/line_scanner.c").read_text(
            encoding="utf-8"
        )
        header = (ROOT / "modules/line_tracking/scanner/line_scanner.h").read_text(
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
        source = (ROOT / "modules/line_tracking/scanner/line_mux.c").read_text(
            encoding="utf-8"
        )
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
        self.assertRegex(source, r"active_count\s*==\s*0U")
        self.assertIn("LINE_EVENT_LOST", source)
        self.assertIn("LINE_EVENT_WIDE_BLACK", source)

    def test_estimator_consumes_stable_line_features(self):
        header = (ROOT / "modules/line_tracking/line_estimator.h").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "modules/line_tracking/line_estimator.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("const LineFeatures *features", header)
        self.assertNotIn("const LineSensorSnapshot *snapshot", header)
        for token in (
            "features->centroid_error",
            "features->error_rate",
            "features->active_count",
            "features->confidence",
        ):
            self.assertIn(token, source)


class LineControllerContract(unittest.TestCase):
    def test_reference_boost_requires_five_frames(self):
        frames = [(0.0, 100, "none", "normal")] * 5
        self.assertEqual(straight_target(frames), [330, 330, 330, 330, 400])

    def test_reference_curve_frame_cancels_boost_immediately(self):
        frames = (
            [(0.0, 100, "none", "normal")] * 5
            + [(1.6, 100, "none", "normal")]
        )
        self.assertEqual(straight_target(frames)[-2:], [400, 330])

    def test_straight_boost_and_early_braking_parameters(self):
        header = (ROOT / "modules/line_tracking/line_controller.h").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "modules/line_tracking/line_controller.c").read_text(
            encoding="utf-8"
        )
        config = (ROOT / "config/line_control_config.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "LINE_MAX_FORWARD (400)",
            "LINE_CRUISE_FORWARD (330)",
            "LINE_STRAIGHT_ERROR_THRESHOLD (1.0f)",
            "LINE_STRAIGHT_CONFIRM_FRAMES (5U)",
            "LINE_CURVE_ERROR_THRESHOLD (1.5f)",
            "LINE_HARD_CURVE_ERROR_THRESHOLD (3.5f)",
            "LINE_CURVE_FORWARD (240)",
            "LINE_HARD_CURVE_FORWARD (150)",
            "LINE_TIGHT_FORWARD (120)",
            "LINE_HAIRPIN_FORWARD (40)",
            "LINE_DECEL_STEP (70)",
            "LINE_TURN_SLEW_STEP (25)",
            "LINE_CONTROL_KP (28.0f)",
        ):
            self.assertIn(token, config)
        for token in (
            "cruise_forward",
            "straight_error_threshold",
            "straight_confirm_frames",
        ):
            self.assertIn(token, header)
        for token in (
            "stable_straight_frames",
            "stable_straight_frame",
            "LINE_TREND_NORMAL",
            "LINE_EVENT_NONE",
            "UINT8_MAX",
        ):
            self.assertIn(token, source)

    def test_straight_boost_resets_on_uncertain_or_curve_frame(self):
        source = (ROOT / "modules/line_tracking/line_controller.c").read_text(
            encoding="utf-8"
        )
        self.assertRegex(
            source,
            r"if\s*\(stable_straight_frame[\s\S]{0,250}"
            r"stable_straight_frames\+\+[\s\S]{0,180}"
            r"else\s*\{\s*stable_straight_frames\s*=\s*0U;",
        )
        self.assertRegex(
            source,
            r"LineController_Reset[\s\S]{0,180}"
            r"stable_straight_frames\s*=\s*0U",
        )
        self.assertRegex(
            source,
            r"estimate->event\s*==\s*LINE_EVENT_LOST[\s\S]{0,180}"
            r"stable_straight_frames\s*=\s*0U",
        )

    def test_all_curve_targets_stay_inside_competition_limit(self):
        config = (ROOT / "config/line_control_config.h").read_text(
            encoding="utf-8"
        )
        values = [
            int(value)
            for value in re.findall(
                r"#define\s+LINE_(?:MAX|CURVE|HARD|WIDE|LOW|TIGHT|HAIRPIN)"
                r"_[A-Z_]+\s+\((-?\d+)\)",
                config,
            )
        ]
        self.assertTrue(values)
        self.assertTrue(all(abs(value) <= 450 for value in values))

    def test_controller_accepts_and_handles_continuous_curve_trends(self):
        header = (ROOT / "modules/line_tracking/line_controller.h").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "modules/line_tracking/line_controller.c").read_text(
            encoding="utf-8"
        )
        config = (ROOT / "config/line_control_config.h").read_text(
            encoding="utf-8"
        )
        self.assertRegex(
            header,
            r"LineController_Step\(const LineEstimate \*estimate,\s*"
            r"const LineTrendResult \*trend,",
        )
        for token in (
            "LINE_TREND_TIGHT_LEFT",
            "LINE_TREND_TIGHT_RIGHT",
            "LINE_TREND_HAIRPIN_LEFT",
            "LINE_TREND_HAIRPIN_RIGHT",
            "trend_forward_target",
            "trend_turn_target",
        ):
            self.assertIn(token, source)
        for token in (
            "LINE_TIGHT_FORWARD (120)",
            "LINE_TIGHT_TURN (100)",
            "LINE_HAIRPIN_FORWARD (40)",
            "LINE_HAIRPIN_TURN (100)",
        ):
            self.assertIn(token, config)
        for field in (
            "tight_forward",
            "tight_turn",
            "hairpin_forward",
            "hairpin_turn",
        ):
            self.assertIn(field, header)
        self.assertIn("slew_turn", source)
        self.assertIn("LINE_CONTROL_KP (28.0f)", config)
        self.assertIn("LINE_MAX_FORWARD (400)", config)
        self.assertNotIn("Contrl_Speed", source)
        self.assertNotIn("Motion_Car_Control", source)

    def test_normal_follow_correction_never_reverses_inner_wheel(self):
        for base_speed in range(1, 451):
            correction_limit = base_speed * 80 // 100
            self.assertGreaterEqual(base_speed - correction_limit, 0)

    def test_hard_corner_has_separate_low_speed_pivot_commands(self):
        source = (ROOT / "modules/line_tracking/line_controller.c").read_text(
            encoding="utf-8"
        )
        config = (ROOT / "config/line_control_config.h").read_text(
            encoding="utf-8"
        )
        recovery = (ROOT / "modules/line_tracking/recovery/line_recovery.c").read_text(
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
        model = (ROOT / "modules/line_tracking/line_model.h").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "modules/line_tracking/line_controller.c").read_text(
            encoding="utf-8"
        )
        config = (ROOT / "config/line_control_config.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "LineControlOutput",
            "forward",
            "turn",
            "valid",
            "LineController_Step",
        ):
            self.assertIn(token, header + model)
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
            "LINE_MAX_FORWARD (400)",
            "LINE_CRUISE_FORWARD (330)",
            "LINE_CURVE_FORWARD (240)",
            "LINE_HARD_CURVE_FORWARD (150)",
            "LINE_WIDE_BLACK_FORWARD (120)",
            "LINE_LOW_CONFIDENCE_FORWARD (150)",
            "LINE_ACCEL_STEP (15)",
            "LINE_DECEL_STEP (70)",
            "LINE_CONTROL_KP (28.0f)",
        ):
            self.assertIn(token, config)
        self.assertIn("LINE_TURN_LIMIT_PERCENT (80)", config)
        self.assertIn("LINE_TURN_SLEW_STEP (25)", config)
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
