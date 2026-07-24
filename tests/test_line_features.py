import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"
WEIGHTS = (-7, -5, -3, -1, 1, 3, 5, 7)


def extract(bits):
    active = [i for i in range(8) if bits & (1 << i)]
    groups = sum(
        1 for i in active if i == 0 or not (bits & (1 << (i - 1)))
    )
    return {
        "active_count": len(active),
        "left_count": sum(i < 4 for i in active),
        "right_count": sum(i >= 4 for i in active),
        "span": 0 if not active else active[-1] - active[0] + 1,
        "segment_count": groups,
        "left_edge": bool(bits & 0x01),
        "right_edge": bool(bits & 0x80),
        "centroid": 0.0 if not active else (
            sum(WEIGHTS[i] for i in active) / len(active)
        ),
    }


class LineFeatureContract(unittest.TestCase):
    def test_reference_features_are_mirror_symmetric(self):
        for bits in range(256):
            mirror = int(f"{bits:08b}"[::-1], 2)
            left = extract(bits)
            right = extract(mirror)
            self.assertEqual(left["active_count"], right["active_count"])
            self.assertEqual(left["span"], right["span"])
            self.assertEqual(left["segment_count"], right["segment_count"])
            self.assertAlmostEqual(left["centroid"], -right["centroid"])

    def test_l_shape_side_patterns_keep_direction_evidence(self):
        self.assertLess(extract(0x0F)["centroid"], 0)
        self.assertGreater(extract(0xF0)["centroid"], 0)
        self.assertEqual(extract(0xFF)["centroid"], 0)
        self.assertEqual(extract(0xFF)["active_count"], 8)

    def test_public_feature_interface_is_complete(self):
        header = (ROOT / "modules/line_tracking/line_features.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "LineFeatures",
            "active_count",
            "left_count",
            "right_count",
            "span",
            "segment_count",
            "centroid_error",
            "error_rate",
            "LineFeatureExtractor_Update",
        ):
            self.assertIn(token, header)

    def test_extractor_rejects_stale_and_duplicate_history(self):
        source = (ROOT / "modules/line_tracking/line_features.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("ModuleStatus_IsFresh", source)
        self.assertIn("snapshot->status.sequence", source)
        self.assertIn("previous_sequence", source)
        self.assertIn("previous_timestamp_ms", source)


if __name__ == "__main__":
    unittest.main()
