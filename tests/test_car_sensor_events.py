from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"
EVENT_HEADER_PATH = PROJECT / "BSP/CarControl/car_sensor_events.h"
EVENT_SOURCE_PATH = PROJECT / "BSP/CarControl/car_sensor_events.c"
TRACKING_HEADER_PATH = PROJECT / "BSP/Eight_Tracking/app_irtracking.h"
TRACKING_SOURCE_PATH = PROJECT / "BSP/Eight_Tracking/app_irtracking.c"

CAR_LINE_NONE = 0
CAR_LINE_CENTER = 1
CAR_LINE_LEFT_EDGE = 2
CAR_LINE_RIGHT_EDGE = 3
CAR_LINE_CROSS = 4
CAR_LINE_STOP_MARKER = 5


def classify_reference(sensor_bits: int, weighted_error: float, line_valid: bool) -> int:
    """Host-side reference for the pure CarSensor_Classify decision table."""
    if not line_valid:
        return CAR_LINE_NONE
    if sensor_bits == 0xFF:
        return CAR_LINE_STOP_MARKER

    active_count = sensor_bits.bit_count()
    if ((sensor_bits & 0x81) == 0x81) or active_count >= 4:
        return CAR_LINE_CROSS
    if weighted_error <= -3.0:
        return CAR_LINE_LEFT_EDGE
    if weighted_error >= 3.0:
        return CAR_LINE_RIGHT_EDGE
    return CAR_LINE_CENTER


def function_body(source: str, name: str) -> str:
    match = re.search(
        rf"(?:bool|uint8_t|CarLineEvent)\s+{name}\s*\([^)]*\)\s*\{{(.*?)\n\}}",
        source,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError(f"missing function {name}")
    return match.group(1)


class CarSensorEventsTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(EVENT_HEADER_PATH.is_file(), "missing Task 4 public header")
        self.assertTrue(EVENT_SOURCE_PATH.is_file(), "missing Task 4 implementation")
        self.event_header = EVENT_HEADER_PATH.read_text(encoding="utf-8")
        self.event_source = EVENT_SOURCE_PATH.read_text(encoding="utf-8")
        self.tracking_header = TRACKING_HEADER_PATH.read_text(encoding="utf-8")
        self.tracking_source = TRACKING_SOURCE_PATH.read_text(encoding="utf-8")

    def test_exact_sensor_event_api_exists(self):
        for token in (
            "CAR_LINE_NONE = 0",
            "CAR_LINE_CENTER",
            "CAR_LINE_LEFT_EDGE",
            "CAR_LINE_RIGHT_EDGE",
            "CAR_LINE_CROSS",
            "CAR_LINE_STOP_MARKER",
            "uint8_t sensor_bits;",
            "float weighted_error;",
            "bool line_valid;",
            "CarLineEvent event;",
            "bool CarSensor_ReadFrame(CarSensorFrame *frame);",
            "CarLineEvent CarSensor_Classify(const CarSensorFrame *frame);",
        ):
            with self.subTest(token=token):
                self.assertIn(token, self.event_header)

    def test_low_level_black_conversion_preserves_x1_x8_weight_signs(self):
        self.assertIn("uint8_t Tracking_PackBlackSensors(", self.tracking_header)
        pack = function_body(self.tracking_source, "Tracking_PackBlackSensors")
        for sensor, bit in (("x1", 0), ("x2", 1), ("x3", 2), ("x4", 3),
                            ("x5", 4), ("x6", 5), ("x7", 6), ("x8", 7)):
            self.assertRegex(pack, rf"{sensor}\s*==\s*0U.*1U\s*<<\s*{bit}")
        self.assertRegex(
            self.tracking_source,
            r"TRACKING_WEIGHTS\s*\[\s*8\s*\]\s*=\s*"
            r"\{\s*-7\s*,.*7\s*\}",
            "X1 must remain negative and X8 positive",
        )

    def test_read_frame_uses_one_coherent_sample(self):
        body = function_body(self.event_source, "CarSensor_ReadFrame")
        self.assertIn("frame == NULL", body)
        self.assertEqual(1, body.count("Gray_ReadAll("))
        self.assertIn("Tracking_PackBlackSensors", body)
        self.assertIn("Tracking_ComputeWeightedError", body)
        self.assertIn("frame->event = CarSensor_Classify(frame);", body)

    def test_classifies_center_left_right_cross_and_stop_marker(self):
        body = function_body(self.event_source, "CarSensor_Classify")
        self.assertIn("CAR_LINE_NONE", body, "all-white/no-line input")
        self.assertIn("CAR_LINE_CENTER", body, "center line input")
        self.assertIn("CAR_LINE_LEFT_EDGE", body, "left-edge line input")
        self.assertIn("CAR_LINE_RIGHT_EDGE", body, "right-edge line input")
        self.assertIn("CAR_LINE_CROSS", body, "broad multi-channel cross input")
        self.assertRegex(
            body,
            r"sensor_bits\s*==\s*0xFFU\)[\s\S]*CAR_LINE_STOP_MARKER",
            "exactly all eight black channels are the stop-marker candidate",
        )
        self.assertLess(
            body.index("sensor_bits == 0xFFU"),
            body.index("CAR_LINE_CROSS"),
            "all-black must be checked before broad-cross classification",
        )

    def test_executable_classification_reference_decision_table(self):
        cases = (
            ("center", 0x18, 0.0, True, CAR_LINE_CENTER),
            ("left edge", 0x03, -6.0, True, CAR_LINE_LEFT_EDGE),
            ("right edge", 0xC0, 6.0, True, CAR_LINE_RIGHT_EDGE),
            ("all white", 0x00, 0.0, False, CAR_LINE_NONE),
            ("broad cross", 0x3C, 0.0, True, CAR_LINE_CROSS),
            ("all-black stop candidate", 0xFF, 0.0, True,
             CAR_LINE_STOP_MARKER),
            ("invalid frame overrides bits", 0xFF, 0.0, False,
             CAR_LINE_NONE),
        )

        for name, sensor_bits, weighted_error, line_valid, expected in cases:
            with self.subTest(name=name):
                self.assertEqual(
                    expected,
                    classify_reference(sensor_bits, weighted_error, line_valid),
                )

    def test_event_layer_has_no_motor_command_or_blocking_delay(self):
        forbidden = (
            "Motion_Car_Control",
            "Contrl_Speed",
            "Motor_Safety_RequestSpeed",
            "delay_ms",
            "delay_us",
        )
        for token in forbidden:
            with self.subTest(token=token):
                self.assertNotIn(token, self.event_source)


if __name__ == "__main__":
    unittest.main()
