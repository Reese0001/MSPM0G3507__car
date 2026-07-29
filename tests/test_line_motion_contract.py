from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class LineMotionContract(unittest.TestCase):
    def test_line_motion_owns_mode_dispatch_and_recovery(self):
        header = (ROOT / "app/line/line_motion.h").read_text(encoding="utf-8")
        source = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        control = (ROOT / "app/control/control_runtime.c").read_text(
            encoding="utf-8"
        )
        for token in (
            "AppLineMotion_Init",
            "AppLineMotion_ServiceImu",
            "AppLineMotion_BuildRequest",
            "BuildOfficialBaselineRequest",
            "BuildAssistedRequest",
            "LineOfficialControl_Step",
            "LineRecovery_Step",
        ):
            self.assertIn(token, header + source)
        self.assertIn("AppLineMotion_BuildRequest", control)
        self.assertNotIn("LineTrendResult", source)
        self.assertNotIn("PositionSign", source)

        official = source[
            source.index("BuildOfficialBaselineRequest") :
            source.index("BuildAssistedRequest")
        ]
        self.assertLess(
            official.index("LineOfficialControl_Step"),
            official.index("LineRecovery_Step"),
        )

        assisted = source[source.index("BuildAssistedRequest") :]
        positions = [
            assisted.index("LineDirectionPredictor_Record"),
            assisted.index("LineDirectionPredictor_Predict"),
            assisted.index("LineLookupControl_Step"),
            assisted.index("LineCascadeControl_Step"),
            assisted.index("LineRecovery_Step"),
        ]
        self.assertEqual(positions, sorted(positions))

    def test_official_controller_owns_reliable_direction(self):
        source = (
            ROOT / "modules/line_tracking/controller/line_official_control.c"
        ).read_text(encoding="utf-8")
        self.assertIn("position->stable_position", source)
        self.assertIn("position->candidate_position", source)
        self.assertIn("position->type == LINE_PATTERN_WIDE", source)
        self.assertIn("control_position != 0", source)
        self.assertIn("diagnostics.direction = control_position < 0", source)

    def test_build_links_each_active_line_module_once(self):
        makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
        cproject = (ROOT / ".cproject").read_text(encoding="utf-8")
        for source in (
            "modules/line_tracking/controller/line_lookup_control.c",
            "modules/line_tracking/controller/line_cascade_control.c",
            "modules/line_tracking/controller/line_official_control.c",
            "modules/line_tracking/prediction/line_direction_predictor.c",
        ):
            self.assertEqual(makefile.count(source), 1)
        self.assertIn("-Imodules/line_tracking/prediction", makefile)
        self.assertIn("${PROJECT_ROOT}/modules/line_tracking/prediction", cproject)
        self.assertNotRegex(
            cproject,
            r'excluding="[^"]*(line_lookup_control|line_cascade_control|'
            r'line_official_control|line_direction_predictor)\.c',
        )

    def test_bad_inputs_fail_and_noise_has_a_bounded_hold(self):
        source = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        official = (
            ROOT / "modules/line_tracking/controller/line_official_control.c"
        ).read_text(encoding="utf-8")
        config = (ROOT / "config/line_lookup_config.h").read_text(encoding="utf-8")
        self.assertIn("sample == 0", source)
        self.assertIn("request == 0", source)
        self.assertIn("request->valid = false", source)
        self.assertIn("LINE_PATTERN_NOISE", official)
        self.assertIn("LINE_NOISE_HOLD_MS", official + config)
        self.assertIn("set_lost(out, sequence, now_ms)", official)

    def test_pattern_bounds_and_legacy_startup_hold_are_independent(self):
        source = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        official = (
            ROOT / "modules/line_tracking/controller/line_official_control.c"
        ).read_text(encoding="utf-8")
        assisted = source[source.index("BuildAssistedRequest") :]
        bounds = "(unsigned int)sample->position.type >"
        self.assertIn(bounds, assisted)
        bounds_at = assisted.index(bounds)
        self.assertIn(
            "(unsigned int)LINE_PATTERN_NOISE",
            assisted[bounds_at : bounds_at + 120],
        )
        self.assertIn("(unsigned int)position->type >", official)
        self.assertLess(
            assisted.index("ImuStartupHold(now_ms)"),
            assisted.index("sample->position.type == LINE_PATTERN_NOISE"),
        )
        hold = assisted[
            assisted.index("if (ImuStartupHold(now_ms))") :
            assisted.index("if (ImuStartupHold(now_ms))") + 320
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
