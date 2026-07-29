from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class OfficialBaselineProfileTests(unittest.TestCase):
    def test_official_baseline_is_default_and_assisted_mode_remains(self):
        profile = (ROOT / "config/line_following_profile.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("LINE_CONTROL_MODE_ASSISTED", profile)
        self.assertIn("LINE_CONTROL_MODE_OFFICIAL_BASELINE", profile)
        self.assertIn(
            "#define LINE_FOLLOWING_CONTROL_MODE "
            "(LINE_CONTROL_MODE_OFFICIAL_BASELINE)",
            profile,
        )

    def test_cli_and_ccs_build_include_ybimu(self):
        makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
        cproject = (ROOT / ".cproject").read_text(encoding="utf-8")
        for path in (
            "modules/optional/ybimu/ybimu.c",
            "modules/optional/ybimu/ybimu_protocol.c",
        ):
            self.assertIn(path, makefile)
        self.assertIn("-Imodules/optional/ybimu", makefile)
        exclusion = cproject[cproject.index('<entry excluding="') : cproject.index(
            '" flags="VALUE_WORKSPACE_PATH'
        )]
        self.assertNotIn("modules/optional/ybimu", exclusion)
        self.assertIn("${PROJECT_ROOT}/modules/optional/ybimu", cproject)


if __name__ == "__main__":
    unittest.main()
