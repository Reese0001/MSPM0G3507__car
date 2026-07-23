from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class SafetySupervisorContract(unittest.TestCase):
    def test_states_inputs_and_decision_exist(self):
        header = (ROOT / "application/safety_supervisor.h").read_text(
            encoding="utf-8"
        )
        common = (ROOT / "modules/common/safety_decision.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "SAFETY_BOOT_SAFE",
            "SAFETY_READY",
            "SAFETY_RUNNING",
            "SAFETY_LIMITED",
            "SAFETY_STOP_LATCHED",
            "SAFETY_FAULT",
            "SafetyInputs",
            "UltrasonicSnapshot ultrasonic",
            "ultrasonic_required",
            "YbImuSnapshot imu",
            "K230VisionSnapshot vision",
            "SafetyDecision",
            "SafetySupervisor_Step",
        ):
            self.assertIn(token, header + common)

    def test_priority_order_and_latched_clear_gate_exist(self):
        source = (ROOT / "application/safety_supervisor.c").read_text(
            encoding="utf-8"
        )
        step = source[source.index("bool SafetySupervisor_Step"):]
        priority = (
            "inputs->motor_fault",
            "SAFETY_OBSTACLE_STOP_MM",
            "inputs->imu_required",
            "SAFETY_OBSTACLE_LIMIT_MM",
            "validate_request",
        )
        positions = [step.index(token) for token in priority]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("SAFETY_CLEAR_SAMPLE_COUNT", source)
        self.assertIn("inputs->reset_pressed", source)
        self.assertIn("clear_sample_count", source)

    def test_ultrasonic_gate_can_be_disabled_for_minimal_hardware(self):
        source = (ROOT / "application/safety_supervisor.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("inputs->ultrasonic_required", source)
        self.assertIn(
            "if (inputs->ultrasonic_required && !ultrasonic_fresh)",
            source,
        )

    def test_timing_and_distance_limits_are_tunable(self):
        config = (ROOT / "application/config/safety_config.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "MOTION_REQUEST_MAX_AGE_MS (50U)",
            "SAFETY_OBSTACLE_STOP_MM (200U)",
            "SAFETY_OBSTACLE_LIMIT_MM (350U)",
            "SAFETY_OBSTACLE_CLEAR_MM (400U)",
            "SAFETY_CLEAR_SAMPLE_COUNT (5U)",
            "SAFETY_RUNNING_SPEED_LIMIT (450)",
        ):
            self.assertIn(token, config)


if __name__ == "__main__":
    unittest.main()
