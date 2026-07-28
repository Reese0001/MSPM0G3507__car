from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class LineMotionContract(unittest.TestCase):
    def test_line_motion_owns_line_to_request_logic(self):
        header = (ROOT / "app/line/line_motion.h").read_text(encoding="utf-8")
        source = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        control = (ROOT / "app/control/control_runtime.c").read_text(
            encoding="utf-8"
        )
        for token in (
            "AppLineMotion_Init",
            "AppLineMotion_ServiceImu",
            "AppLineMotion_BuildRequest",
            "line_lookup_control.h",
            "line_direction_predictor.h",
            "LineDirectionPredictor_Reset",
            "LineDirectionPredictor_Record",
            "LineDirectionPredictor_Predict",
            "LineLookupControl_Step",
            "LineCascadeControl_Step",
            "LINE_EVENT_WIDE_BLACK",
            "LineRecovery_Step",
        ):
            self.assertIn(token, header + source)
        self.assertIn("AppLineMotion_BuildRequest", control)
        self.assertNotIn("LineTrendResult", source)
        self.assertNotIn("PositionSign", source)

        positions = [
            source.index("LineDirectionPredictor_Record"),
            source.index("LineDirectionPredictor_Predict"),
            source.index("LineLookupControl_Step"),
            source.index("LineCascadeControl_Step"),
            source.index("LineRecovery_Step"),
        ]
        self.assertEqual(positions, sorted(positions))

    def test_only_position_samples_update_direction_history(self):
        source = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        self.assertRegex(
            source,
            r"if \(sample->position\.type == LINE_PATTERN_POSITION\)\s*\{"
            r"[\s\S]{0,160}LineDirectionPredictor_Record"
            r"\(sample->position\.stable_position\)",
        )

    def test_build_links_each_active_line_module_once(self):
        makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
        cproject = (ROOT / ".cproject").read_text(encoding="utf-8")
        for source in (
            "modules/line_tracking/controller/line_lookup_control.c",
            "modules/line_tracking/controller/line_cascade_control.c",
            "modules/line_tracking/prediction/line_direction_predictor.c",
        ):
            self.assertEqual(makefile.count(source), 1)
        self.assertIn("-Imodules/line_tracking/prediction", makefile)
        self.assertIn(
            '${PROJECT_ROOT}/modules/line_tracking/prediction', cproject
        )
        self.assertNotRegex(
            cproject,
            r'excluding="[^"]*(line_lookup_control|line_cascade_control|'
            r'line_direction_predictor)\.c',
        )

    def test_noise_and_bad_inputs_do_not_publish_valid_line_request(self):
        source = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        self.assertIn("sample == 0", source)
        self.assertIn("request == 0", source)
        self.assertIn("LINE_PATTERN_NOISE", source)
        self.assertIn("request->valid = false", source)
        self.assertIn("AppMailbox_ReadImu", source)
        self.assertIn("yaw_angle_deg", source)


if __name__ == "__main__":
    unittest.main()
