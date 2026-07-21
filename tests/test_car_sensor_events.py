from pathlib import Path
import re
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"
EVENT_HEADER_PATH = PROJECT / "BSP/CarControl/car_sensor_events.h"
EVENT_SOURCE_PATH = PROJECT / "BSP/CarControl/car_sensor_events.c"
TRACKING_HEADER_PATH = PROJECT / "BSP/Eight_Tracking/app_irtracking.h"
TRACKING_SOURCE_PATH = PROJECT / "BSP/Eight_Tracking/app_irtracking.c"
TIARMCLANG = Path(r"C:\ti\ti_cgt_arm_llvm_4.0.2.LTS\bin\tiarmclang.exe")


def source_define_value(source: str, name: str) -> str:
    match = re.search(rf"^#define\s+{name}\s+\(([^)]+)\)", source, re.MULTILINE)
    if match is None:
        raise AssertionError(f"missing source define {name}")
    return match.group(1).rstrip("fFUu")


def source_enum_values(header: str) -> dict[str, int]:
    match = re.search(
        r"typedef\s+enum\s*\{(.*?)\}\s*CarLineEvent\s*;",
        header,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError("missing CarLineEvent enum")

    values = {}
    value = 0
    for item in match.group(1).split(","):
        name, *assignment = item.strip().split("=")
        if assignment:
            value = int(assignment[0].strip(), 0)
        values[name.strip()] = value
        value += 1
    return values


def build_source_bound_decision_model(header: str, source: str):
    """Build a host-side model from the checked-in C decision-table inputs.

    This validates classification cases without claiming to execute an ARM binary.
    The enum values, thresholds, masks, and decision order come from the source.
    """
    body = function_body(source, "CarSensor_Classify")
    events = source_enum_values(header)
    edge_limit = float(source_define_value(source, "CAR_SENSOR_EDGE_ERROR_LIMIT"))
    cross_min_active = int(
        source_define_value(source, "CAR_SENSOR_CROSS_MIN_ACTIVE"), 0
    )
    endpoints_mask = int(
        source_define_value(source, "CAR_SENSOR_ENDPOINTS_MASK"), 0
    )
    rule_markers = (
        "!frame->line_valid",
        "sensor_bits == 0xFFU",
        "CAR_SENSOR_ENDPOINTS_MASK",
        "weighted_error <= -CAR_SENSOR_EDGE_ERROR_LIMIT",
        "weighted_error >= CAR_SENSOR_EDGE_ERROR_LIMIT",
        "return CAR_LINE_CENTER",
    )
    rule_positions = [body.index(marker) for marker in rule_markers]
    if rule_positions != sorted(rule_positions):
        raise AssertionError("CarSensor_Classify decision order changed")

    def classify(sensor_bits: int, weighted_error: float, line_valid: bool) -> int:
        if not line_valid:
            return events["CAR_LINE_NONE"]
        if sensor_bits == 0xFF:
            return events["CAR_LINE_STOP_MARKER"]

        active_count = sensor_bits.bit_count()
        if ((sensor_bits & endpoints_mask) == endpoints_mask or
                active_count >= cross_min_active):
            return events["CAR_LINE_CROSS"]
        if weighted_error <= -edge_limit:
            return events["CAR_LINE_LEFT_EDGE"]
        if weighted_error >= edge_limit:
            return events["CAR_LINE_RIGHT_EDGE"]
        return events["CAR_LINE_CENTER"]

    return classify, events


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

    def test_source_bound_decision_model_exercises_classification_table(self):
        classify, events = build_source_bound_decision_model(
            self.event_header,
            self.event_source,
        )
        cases = (
            ("center", 0x18, 0.0, True, "CAR_LINE_CENTER"),
            ("left edge", 0x03, -6.0, True, "CAR_LINE_LEFT_EDGE"),
            ("right edge", 0xC0, 6.0, True, "CAR_LINE_RIGHT_EDGE"),
            ("all white", 0x00, 0.0, False, "CAR_LINE_NONE"),
            ("broad cross", 0x3C, 0.0, True, "CAR_LINE_CROSS"),
            ("all-black stop candidate", 0xFF, 0.0, True,
             "CAR_LINE_STOP_MARKER"),
            ("invalid frame overrides bits", 0xFF, 0.0, False,
             "CAR_LINE_NONE"),
        )

        for name, sensor_bits, weighted_error, line_valid, expected_name in cases:
            with self.subTest(name=name):
                self.assertEqual(
                    events[expected_name],
                    classify(sensor_bits, weighted_error, line_valid),
                )

    def test_tiarmclang_compiles_event_module_for_cortex_m0plus(self):
        self.assertTrue(TIARMCLANG.is_file(), f"missing compiler: {TIARMCLANG}")

        with tempfile.TemporaryDirectory() as temp_dir:
            temporary = Path(temp_dir)
            stubs = temporary / "include"
            stubs.mkdir()
            (stubs / "ti_msp_dl_config.h").write_text(
                "#include <stdint.h>\n",
                encoding="utf-8",
            )
            (stubs / "app_motor.h").write_text("\n", encoding="utf-8")
            (stubs / "usart.h").write_text("\n", encoding="utf-8")
            object_file = temporary / "car_sensor_events.obj"
            command = (
                str(TIARMCLANG),
                "-c",
                str(EVENT_SOURCE_PATH),
                "-o",
                str(object_file),
                "-mcpu=cortex-m0plus",
                "-mthumb",
                "-mfloat-abi=soft",
                "-std=c11",
                "-I",
                str(stubs),
            )
            result = subprocess.run(
                command,
                cwd=PROJECT,
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertEqual(
                0,
                result.returncode,
                f"tiarmclang failed:\n{result.stdout}\n{result.stderr}",
            )
            self.assertTrue(object_file.is_file(), "ARM object was not emitted")

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
