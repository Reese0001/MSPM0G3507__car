import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


def score(bits):
    left = sum(bool(bits & (1 << i)) for i in range(4))
    right = sum(bool(bits & (1 << i)) for i in range(4, 8))
    edge = 3 * bool(bits & 0x80) - 3 * bool(bits & 0x01)
    return 2 * (right - left) + edge


def classify(trace):
    stable = wide = total = 0
    candidate = False
    for bits in trace:
        count = bits.bit_count()
        if candidate:
            total += score(bits)
            if total <= -4:
                return "left"
            if total >= 4:
                return "right"
        elif 1 <= count <= 3:
            stable = min(255, stable + 1)
            wide = 0
        elif count >= 4 and stable >= 3:
            wide += 1
            if wide >= 2:
                candidate = True
                total += score(bits)
        else:
            stable = wide = 0
    return "pending" if candidate else "normal"


class LineEventClassifierContract(unittest.TestCase):
    def test_direct_straight_to_wide_enters_pending(self):
        self.assertEqual(
            classify([0x18, 0x18, 0x18, 0xFF, 0xFF]),
            "pending",
        )

    def test_forward_probe_resolves_left_and_right_l_shapes(self):
        prefix = [0x18, 0x18, 0x18, 0xFF, 0xFF]
        self.assertEqual(classify(prefix + [0x0F]), "left")
        self.assertEqual(classify(prefix + [0xF0]), "right")

    def test_single_wide_glitch_is_not_a_corner(self):
        self.assertEqual(
            classify([0x18, 0x18, 0x18, 0xFF, 0x18]),
            "normal",
        )

    def test_public_events_and_reset_exist(self):
        header = (
            ROOT / "modules/line_tracking/line_event_classifier.h"
        ).read_text(encoding="utf-8")
        for token in (
            "LINE_PATH_WIDE_PENDING",
            "LINE_PATH_RIGHT_ANGLE_LEFT",
            "LINE_PATH_RIGHT_ANGLE_RIGHT",
            "direction_confidence",
            "LineEventClassifier_Reset",
        ):
            self.assertIn(token, header)


if __name__ == "__main__":
    unittest.main()
