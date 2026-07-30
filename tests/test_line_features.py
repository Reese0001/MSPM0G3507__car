import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"
WEIGHTS = (-7, -5, -3, -1, 1, 3, 5, 7)


def extract(bits):
    active = [i for i in range(8) if bits & (1 << i)]
    groups = sum(
        1 for i in active if i == 0 or not (bits & (1 << (i - 1)))
    )
    return {
        "active_count": len(active),
        "left_count": sum(i < 4 for i in active),
        "right_count": sum(i >= 4 for i in active),
        "span": 0 if not active else active[-1] - active[0] + 1,
        "segment_count": groups,
        "left_edge": bool(bits & 0x01),
        "right_edge": bool(bits & 0x80),
        "centroid": 0.0 if not active else (
            sum(WEIGHTS[i] for i in active) / len(active)
        ),
    }


class LineFeatureContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        program_files_x86 = Path(
            os.environ.get("ProgramFiles(x86)", r"C:\\Program Files (x86)")
        )
        vswhere = program_files_x86 / "Microsoft Visual Studio/Installer/vswhere.exe"
        installation = subprocess.run(
            [str(vswhere), "-latest", "-products", "*", "-property", "installationPath"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        devcmd = Path(installation) / "Common7/Tools/VsDevCmd.bat"
        cls.tempdir = tempfile.TemporaryDirectory()
        harness = Path(cls.tempdir.name) / "line_features_harness.c"
        executable = Path(cls.tempdir.name) / "line_features_harness.exe"
        harness.write_text(
            r'''
#include "line_features.h"

static LineSensorSnapshot make_snapshot(uint8_t bits, uint16_t sequence,
                                        uint32_t timestamp_ms)
{
    LineSensorSnapshot snapshot = {
        {timestamp_ms, sequence, true, MODULE_HEALTH_OK}, bits
    };
    return snapshot;
}

int main(void)
{
    LineFeatures out = {0};
    LineSensorSnapshot snapshot;

    LineFeatureExtractor_Init();
    snapshot = make_snapshot(0x01U, 1U, 100U);
    if (LineFeatureExtractor_Update(&snapshot, 121U, &out)) {
        return 1;
    }

    LineFeatureExtractor_Reset();
    snapshot = make_snapshot(0x01U, 10U, 100U);
    if (!LineFeatureExtractor_Update(&snapshot, 100U, &out) ||
        out.centroid_error != -7.0f) {
        return 2;
    }
    snapshot = make_snapshot(0x80U, 10U, 100U);
    if (!LineFeatureExtractor_Update(&snapshot, 100U, &out) ||
        out.error_rate != 0.0f) {
        return 3;
    }
    snapshot = make_snapshot(0x00U, 11U, 105U);
    if (!LineFeatureExtractor_Update(&snapshot, 105U, &out) ||
        out.centroid_error != -7.0f) {
        return 4;
    }

    LineFeatureExtractor_Reset();
    snapshot = make_snapshot(0x01U, 20U, 200U);
    if (!LineFeatureExtractor_Update(&snapshot, 200U, &out)) {
        return 5;
    }
    snapshot = make_snapshot(0x55U, 21U, 205U);
    if (!LineFeatureExtractor_Update(&snapshot, 205U, &out) ||
        out.confidence != 15U) {
        return 6;
    }

    LineFeatureExtractor_Reset();
    snapshot = make_snapshot(0x55U, 30U, 300U);
    if (!LineFeatureExtractor_Update(&snapshot, 300U, &out) ||
        out.confidence != 30U) {
        return 7;
    }
    LineFeatureExtractor_Reset();
    snapshot = make_snapshot(0x01U, 40U, 400U);
    if (!LineFeatureExtractor_Update(&snapshot, 400U, &out)) {
        return 8;
    }
    snapshot = make_snapshot(0x80U, 41U, 405U);
    if (!LineFeatureExtractor_Update(&snapshot, 405U, &out) ||
        out.confidence != 85U) {
        return 9;
    }
    return 0;
}
''',
            encoding="utf-8",
        )
        compile_script = Path(cls.tempdir.name) / "compile_harness.bat"
        compile_script.write_text(
            "@echo off\n"
            f'call "{devcmd}" -arch=x64 -host_arch=x64 >nul\n'
            "if errorlevel 1 exit /b 1\n"
            "cl.exe /nologo /std:c11 /utf-8 /W4 /WX "
            f'/I"{ROOT / "modules/optional/competition/line_tracking"}" '
            f'/I"{ROOT / "modules/line_tracking"}" '
            f'"{ROOT / "modules/optional/competition/line_tracking/line_features.c"}" '
            f'"{harness}" /Fe"{executable}"\n',
            encoding="utf-8",
        )
        result = subprocess.run(
            ["cmd", "/d", "/c", str(compile_script)],
            cwd=cls.tempdir.name,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        cls.harness = executable

    @classmethod
    def tearDownClass(cls):
        cls.tempdir.cleanup()

    def test_reference_features_are_mirror_symmetric(self):
        for bits in range(256):
            mirror = int(f"{bits:08b}"[::-1], 2)
            left = extract(bits)
            right = extract(mirror)
            self.assertEqual(left["active_count"], right["active_count"])
            self.assertEqual(left["span"], right["span"])
            self.assertEqual(left["segment_count"], right["segment_count"])
            self.assertAlmostEqual(left["centroid"], -right["centroid"])

    def test_l_shape_side_patterns_keep_direction_evidence(self):
        self.assertLess(extract(0x0F)["centroid"], 0)
        self.assertGreater(extract(0xF0)["centroid"], 0)
        self.assertEqual(extract(0xFF)["centroid"], 0)
        self.assertEqual(extract(0xFF)["active_count"], 8)

    def test_public_feature_interface_is_complete(self):
        header = (ROOT / "modules/optional/competition/line_tracking/line_features.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "LineFeatures",
            "active_count",
            "left_count",
            "right_count",
            "span",
            "segment_count",
            "centroid_error",
            "error_rate",
            "LineFeatureExtractor_Update",
        ):
            self.assertIn(token, header)

    def test_extractor_rejects_stale_and_duplicate_history(self):
        source = (ROOT / "modules/optional/competition/line_tracking/line_features.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("ModuleStatus_IsFresh", source)
        self.assertIn("snapshot->status.sequence", source)
        self.assertIn("previous_sequence", source)
        self.assertIn("previous_timestamp_ms", source)

    def test_c_feature_extractor_enforces_temporal_contract(self):
        result = subprocess.run([str(self.harness)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_active_line_motion_does_not_depend_on_old_feature_pipeline(self):
        line_motion = (ROOT / "modules/line_tracking/line_follower.c").read_text(encoding="utf-8")
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        self.assertIn("LineFollower_Step", tasks)
        for forbidden in (
            "LineFeatureExtractor_Update",
            "LineEstimator_Update(&line_features",
            "LineRecovery_Step",
            "LineCascadeControl_Step",
            "LineLookupControl_Step",
            "LineDirectionPredictor_Predict",
            "AppScheduler_",
        ):
            self.assertNotIn(forbidden, line_motion + tasks)


if __name__ == "__main__":
    unittest.main()

