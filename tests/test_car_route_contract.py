from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"
HEADER = PROJECT / "BSP/CarControl/car_route.h"
SOURCE = PROJECT / "BSP/CarControl/car_route.c"


def function_body(source: str, name: str) -> str:
    match = re.search(rf"(?:void|CarRouteState|int16_t)\s+{name}\s*\([^)]*\)\s*\{{(.*?)\n\}}", source, re.DOTALL)
    if match is None:
        raise AssertionError(f"missing {name}")
    return match.group(1)


class CarRouteContractTests(unittest.TestCase):
    def setUp(self):
        self.header = HEADER.read_text(encoding="utf-8")
        self.source = SOURCE.read_text(encoding="utf-8")

    def test_public_api_and_states(self):
        for token in (
            "CAR_ROUTE_IDLE = 0", "CAR_ROUTE_LINE_FOLLOW",
            "CAR_ROUTE_DRIVE_DISTANCE", "CAR_ROUTE_TURN_ANGLE",
            "CAR_ROUTE_SEARCH_LINE", "CAR_ROUTE_TARGET_APPROACH",
            "CAR_ROUTE_STOPPED", "CAR_ROUTE_FAULT",
            "void CarRoute_Init(void);", "void CarRoute_Start(void);",
            "void CarRoute_Stop(void);", "CarRoute_GetState(void);",
        ):
            self.assertIn(token, self.header)

    def test_start_arms_and_only_enters_line_follow_from_idle_or_stopped(self):
        body = function_body(self.source, "CarRoute_Start")
        self.assertIn("Motor_Safety_IsFaultLatched", body)
        self.assertIn("Motor_Safety_Arm();", body)
        self.assertIn("CAR_ROUTE_IDLE", body)
        self.assertIn("CAR_ROUTE_STOPPED", body)
        self.assertIn("route_state = CAR_ROUTE_LINE_FOLLOW", body)

    def test_invalid_sensor_and_lost_line_latch_fault_and_stop(self):
        body = function_body(self.source, "CarRoute_Run")
        self.assertIn("sensor == (const CarSensorFrame *)0", body)
        self.assertIn("!sensor->line_valid", body)
        self.assertIn("Get_Time() - motion->timestamp_ms", body)
        self.assertIn(">=\n                CAR_MOTION_FEEDBACK_MAX_AGE_MS", body)
        self.assertIn("CarRoute_FaultStop();", body)
        self.assertIn("CarMotion_Stop();", self.source)

    def test_stop_marker_requires_two_consecutive_frames(self):
        self.assertIn("CAR_ROUTE_STOP_MARKER_FRAMES   (2U)", self.source)
        body = function_body(self.source, "CarRoute_Run")
        self.assertIn("stop_marker_frames++", body)
        self.assertIn("stop_marker_frames >= CAR_ROUTE_STOP_MARKER_FRAMES", body)
        self.assertIn("CarRoute_Stop();", body)
        candidate = body[body.index("classified_event == CAR_LINE_STOP_MARKER"):]
        self.assertLess(candidate.index("CarMotion_Stop();"), candidate.index("stop_marker_frames++"))

    def test_distance_turn_fail_closed_when_units_unvalidated(self):
        body = function_body(self.source, "CarRoute_Run")
        self.assertIn("CAR_ROUTE_DRIVE_DISTANCE", body)
        self.assertIn("CAR_ROUTE_TURN_ANGLE", body)
        self.assertIn("!motion->units_valid", body)
        self.assertIn("CarRoute_FaultStop();", body)

    def test_unsupported_states_are_disabled_and_nonblocking(self):
        body = function_body(self.source, "CarRoute_Run")
        self.assertIn("CAR_ROUTE_SEARCH_LINE", body)
        self.assertIn("CAR_ROUTE_TARGET_APPROACH", body)
        for token in ("delay_ms", "delay_us", "Contrl_Speed", "Motor_Safety_RequestSpeed"):
            self.assertNotIn(token, self.source)

    def test_route_uses_car_motion_boundary_only(self):
        self.assertIn('#include "../Motor/motor_safety.h"', self.source)
        self.assertIn("CarMotion_Command(", self.source)
        for token in ("app_motor_usart", "bsp_motor_usart", "Contrl_Speed"):
            self.assertNotIn(token, self.source)


if __name__ == "__main__":
    unittest.main()
