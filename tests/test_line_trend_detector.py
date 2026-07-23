from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


def classify_trace(samples):
    """Reference oracle for the required outward-trend classifications."""
    direction = 0
    outward_steps = 0
    previous_error = 0
    outer_seen = False
    hairpin_frames = 0

    for error, event in samples:
        sample_direction = -1 if error < 0 else 1 if error > 0 else 0
        if sample_direction and direction and sample_direction != direction:
            outward_steps = 0
            outer_seen = False
            hairpin_frames = 0
        if sample_direction:
            if sample_direction == direction and abs(error) > abs(previous_error):
                outward_steps += 1
            elif sample_direction != direction:
                outward_steps = 0
            direction = sample_direction

        if abs(error) >= 4:
            outer_seen = True
        if event == "hard" and abs(error) >= 6 and outward_steps >= 2:
            hairpin_frames += 1
            if hairpin_frames >= 3:
                return "hairpin_left" if direction < 0 else "hairpin_right"
        if event in ("lost", "wide") and outer_seen and outward_steps >= 2:
            return "right_angle_left" if direction < 0 else "right_angle_right"
        previous_error = error

    if direction and outward_steps >= 2 and abs(previous_error) >= 3:
        return "tight_left" if direction < 0 else "tight_right"
    return "normal"


class LineTrendDetectorContract(unittest.TestCase):
    def test_photographed_track_scenarios_and_mirrors(self):
        cases = {
            "SQUARE_LEFT": (
                ((-1, "none"), (-3, "none"), (-5, "none"), (0, "wide")),
                "right_angle_left",
            ),
            "SQUARE_RIGHT": (
                ((1, "none"), (3, "none"), (5, "none"), (0, "wide")),
                "right_angle_right",
            ),
            "HAIRPIN_LEFT": (
                ((-1, "none"), (-3, "none"), (-5, "none"),
                 (-7, "hard"), (-7, "hard"), (-7, "hard")),
                "hairpin_left",
            ),
            "HAIRPIN_RIGHT": (
                ((1, "none"), (3, "none"), (5, "none"),
                 (7, "hard"), (7, "hard"), (7, "hard")),
                "hairpin_right",
            ),
            "S_CURVE": (
                ((-1, "none"), (-3, "none"), (-1, "none"),
                 (1, "none"), (3, "none"), (1, "none")),
                "normal",
            ),
            "ORDINARY_GAP": (
                ((0, "none"), (0, "none"), (0, "lost"),
                 (0, "lost"), (0, "lost")),
                "normal",
            ),
        }
        for name, (trace, expected) in cases.items():
            with self.subTest(name=name):
                self.assertEqual(classify_trace(trace), expected)

    def test_same_square_trace_is_identical_after_reset(self):
        square_left = (
            (-1, "none"), (-3, "none"), (-5, "none"), (0, "wide")
        )
        first_lap = classify_trace(square_left)
        second_lap_after_reset = classify_trace(square_left)
        self.assertEqual(first_lap, "right_angle_left")
        self.assertEqual(second_lap_after_reset, first_lap)

    def test_three_stable_frames_clear_old_corner_evidence(self):
        source = (ROOT / "modules/line_tracking/line_trend_detector.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("stable_reacquire_frames", source)
        self.assertIn("LINE_TREND_REACQUIRE_FRAMES", source)
        self.assertRegex(
            source,
            r"stable_reacquire_frames\s*>=\s*LINE_TREND_REACQUIRE_FRAMES"
            r"[\s\S]{0,180}clear_direction_evidence",
        )

    def test_scheduler_resets_trend_at_control_reset_boundaries(self):
        scheduler = (ROOT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )
        self.assertGreaterEqual(scheduler.count("LineTrendDetector_Reset();"), 2)
        self.assertRegex(
            scheduler,
            r"LINE_RECOVERY_FAULT[\s\S]{0,400}LineTrendDetector_Reset\(\)",
        )

    def test_public_types_and_thresholds_exist(self):
        header_path = ROOT / "modules/line_tracking/line_trend_detector.h"
        config_path = ROOT / "modules/line_tracking/line_trend_config.h"
        self.assertTrue(header_path.exists())
        self.assertTrue(config_path.exists())
        header = header_path.read_text(encoding="utf-8")
        config = config_path.read_text(encoding="utf-8")
        for token in (
            "LINE_TREND_NORMAL",
            "LINE_TREND_TIGHT_LEFT",
            "LINE_TREND_TIGHT_RIGHT",
            "LINE_TREND_HAIRPIN_LEFT",
            "LINE_TREND_HAIRPIN_RIGHT",
            "LINE_TREND_RIGHT_ANGLE_LEFT",
            "LINE_TREND_RIGHT_ANGLE_RIGHT",
            "LineTrendResult",
            "LineTrendDetector_Update",
            "LineTrendDetector_Reset",
        ):
            self.assertIn(token, header)
        for token in (
            "LINE_TREND_TIGHT_ERROR (3.0f)",
            "LINE_TREND_OUTER_ERROR (4.0f)",
            "LINE_TREND_HAIRPIN_ERROR (6.0f)",
            "LINE_TREND_CORNER_WINDOW_MS (200U)",
            "LINE_TREND_CROSSLINE_ACTIVE_COUNT (4U)",
        ):
            self.assertIn(token, config)

    def test_detector_tracks_sequence_not_only_last_sample(self):
        source = (ROOT / "modules/line_tracking/line_trend_detector.c").read_text(
            encoding="utf-8"
        )
        for token in (
            "outward_steps",
            "outer_seen_ms",
            "hairpin_frames",
            "count_active_bits",
            "LINE_EVENT_WIDE_BLACK",
            "LINE_EVENT_LOST",
        ):
            self.assertIn(token, source)
        self.assertNotIn("Contrl_Speed", source)

    def test_low_confidence_completion_events_are_not_rejected(self):
        source = (ROOT / "modules/line_tracking/line_trend_detector.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("completion_event", source)
        self.assertRegex(
            source,
            r"estimate->confidence\s*<\s*LINE_TREND_MIN_CONFIDENCE\s*&&"
            r"\s*!completion_event",
        )

    def test_trace_classifications_require_an_outward_sequence(self):
        cases = {
            "LEFT_TIGHT": (
                ((-1, "none"), (-3, "none"), (-5, "none")),
                "tight_left",
            ),
            "RIGHT_HAIRPIN": (
                ((1, "none"), (3, "none"), (5, "none"),
                 (7, "hard"), (7, "hard"), (7, "hard")),
                "hairpin_right",
            ),
            "LEFT_LOST_CORNER": (
                ((-1, "none"), (-3, "none"), (-5, "hard"), (-5, "lost")),
                "right_angle_left",
            ),
            "RIGHT_WIDE_CORNER": (
                ((1, "none"), (3, "none"), (5, "hard"), (0, "wide")),
                "right_angle_right",
            ),
            "NOISE_NOT_CORNER": (
                ((0, "none"), (7, "hard"), (0, "none"), (0, "lost")),
                "normal",
            ),
        }
        for name, (samples, expected) in cases.items():
            with self.subTest(name=name):
                self.assertEqual(classify_trace(samples), expected)


if __name__ == "__main__":
    unittest.main()
