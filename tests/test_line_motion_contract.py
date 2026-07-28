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

    def test_position_and_wide_samples_update_direction_history(self):
        source = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        self.assertIn("control_position = sample->position.stable_position;", source)
        self.assertIn("sample->position.type == LINE_PATTERN_WIDE", source)
        self.assertIn("control_position != 0", source)
        self.assertIn("LineDirectionPredictor_Record(control_position)", source)

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

    def test_pattern_bounds_and_startup_hold_fail_closed_before_noise(self):
        source = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        build = source[source.index("bool AppLineMotion_BuildRequest") :]
        bounds = "(unsigned int)sample->position.type >"
        self.assertIn(bounds, build)
        bounds_at = build.index(bounds)
        self.assertIn(
            "(unsigned int)LINE_PATTERN_NOISE",
            build[bounds_at : bounds_at + 120],
        )
        self.assertLess(bounds_at, build.index("event_by_pattern["))
        self.assertLess(
            build.index("ImuStartupHold(now_ms)"),
            build.index("sample->position.type == LINE_PATTERN_NOISE"),
        )
        hold = build[
            build.index("if (ImuStartupHold(now_ms))") :
            build.index("if (ImuStartupHold(now_ms))") + 320
        ]
        for reset in (
            "LinePosition_Reset()",
            "LineDirectionPredictor_Reset()",
            "LineRecovery_Reset()",
            "LineCascadeControl_Init(now_ms)",
        ):
            self.assertIn(reset, hold)


if __name__ == "__main__":
    unittest.main()
