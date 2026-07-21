from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


def read(relative: str) -> str:
    return (PROJECT / relative).read_text(encoding="utf-8")


class CarMotionContractTests(unittest.TestCase):
    def test_exact_public_api_and_feedback_contract(self):
        header = read("BSP/CarControl/car_motion.h")
        source = read("BSP/CarControl/car_motion.c")
        for declaration in (
            "void CarMotion_Reset(void);",
            "bool CarMotion_ReadFeedback(CarMotionFeedback *feedback);",
            "void CarMotion_Command(int16_t linear_speed, int16_t angular_command);",
            "void CarMotion_Stop(void);",
            "bool CarMotion_DriveDistanceStart(int32_t distance_mm, int16_t speed);",
            "bool CarMotion_DriveDistanceStep(void);",
            "bool CarMotion_TurnAngleStart(float angle_deg, int16_t speed);",
            "bool CarMotion_TurnAngleStep(void);",
        ):
            self.assertIn(declaration, header)
        self.assertIn("bool units_valid;", header)
        self.assertRegex(source, r"feedback->left_ticks\s*=\s*Encoder_Offset\[1\]")
        self.assertRegex(source, r"feedback->right_ticks\s*=\s*Encoder_Offset\[3\]")
        self.assertRegex(source, r"feedback->left_speed_mm_s\s*=\s*g_Speed\[1\]")
        self.assertRegex(source, r"feedback->right_speed_mm_s\s*=\s*g_Speed\[3\]")
        self.assertRegex(source, r"feedback->units_valid\s*=\s*false")

    def test_command_routes_only_through_motion_control_and_stop_is_safe(self):
        source = read("BSP/CarControl/car_motion.c")
        command = re.search(
            r"void\s+CarMotion_Command\s*\([^)]*\)\s*\{(.*?)\n\}",
            source,
            re.DOTALL,
        ).group(1)
        self.assertIn("Motion_Car_Control", command)
        self.assertNotIn("Contrl_", command)
        self.assertNotIn("delay_ms", source)
        stop = re.search(
            r"void\s+CarMotion_Stop\s*\([^)]*\)\s*\{(.*?)\n\}",
            source,
            re.DOTALL,
        ).group(1)
        self.assertIn("Motion_Car_Control(0, 0, 0)", stop)

    def test_two_wheel_mapping_and_unvalidated_units_are_explicit(self):
        motor = read("BSP/Motor/app_motor.c")
        self.assertIn("speed_L1_setup = 0", motor)
        self.assertIn("speed_R1_setup = 0", motor)
        source = read("BSP/CarControl/car_motion.c")
        self.assertIn("CAR_MOTION_UNITS_CONFIRMED 0", source)
        self.assertIn("65 mm", source)
        self.assertNotIn("delay_ms", source)

    def test_start_step_handles_inactive_actions(self):
        source = read("BSP/CarControl/car_motion.c")
        inactive_blocks = re.findall(
            r"if \(motion_action == CAR_MOTION_ACTION_NONE\) \{(.*?)\}",
            source,
            re.DOTALL,
        )
        self.assertEqual(2, len(inactive_blocks))
        for block in inactive_blocks:
            self.assertIn("CarMotion_Stop();", block)
            self.assertIn("return false;", block)


if __name__ == "__main__":
    unittest.main()
