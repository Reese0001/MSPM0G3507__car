import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


def score(bits):
    left = sum(bool(bits & (1 << i)) for i in range(4))
    right = sum(bool(bits & (1 << i)) for i in range(4, 8))
    edge = 3 * bool(bits & 0x80) - 3 * bool(bits & 0x01)
    return 2 * (right - left) + edge


def classify(trace):
    stable = wide = total = 0
    candidate = False
    for bits in trace:
        count = bits.bit_count()
        if candidate:
            total += score(bits)
            if total <= -4:
                return "left"
            if total >= 4:
                return "right"
        elif 1 <= count <= 3:
            stable = min(255, stable + 1)
            wide = 0
        elif count >= 4 and stable >= 3:
            wide += 1
            if wide >= 2:
                candidate = True
        else:
            stable = wide = 0
    return "pending" if candidate else "normal"


class LineEventClassifierContract(unittest.TestCase):
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
        harness = Path(cls.tempdir.name) / "line_event_classifier_harness.c"
        executable = Path(cls.tempdir.name) / "line_event_classifier_harness.exe"
        harness.write_text(
            r'''
#include <stdio.h>

#include "line_event_classifier.h"

static LineFeatures make_features(uint8_t bits, uint16_t sequence,
                                  uint32_t timestamp_ms)
{
    LineFeatures features = {0};
    uint8_t index;
    uint8_t first = 0U;
    uint8_t last = 0U;

    features.status = (ModuleStatus){
        timestamp_ms, sequence, true, MODULE_HEALTH_OK
    };
    features.black_bits = bits;
    features.confidence = 100U;
    for (index = 0U; index < 8U; index++) {
        if ((bits & (uint8_t)(1U << index)) != 0U) {
            if (features.active_count == 0U) {
                first = index;
            }
            last = index;
            features.active_count++;
            if (index < 4U) {
                features.left_count++;
            } else {
                features.right_count++;
            }
        }
    }
    if (features.active_count != 0U) {
        features.span = last - first + 1U;
    }
    features.left_edge = (bits & 0x01U) != 0U;
    features.right_edge = (bits & 0x80U) != 0U;
    return features;
}

static bool update_bits(uint8_t bits, uint16_t sequence, uint32_t now_ms,
                        LinePathEvent *out)
{
    LineFeatures features = make_features(bits, sequence, now_ms);
    LineEstimate estimate = {0};
    LineTrendResult trend = {0};

    estimate.status = (ModuleStatus){
        now_ms, sequence, true, MODULE_HEALTH_OK
    };
    trend.status = (ModuleStatus){
        now_ms, sequence, true, MODULE_HEALTH_OK
    };
    return LineEventClassifier_Update(
        &features, &estimate, &trend, now_ms, out);
}

static int require(bool condition, int code)
{
    if (!condition) {
        (void)fprintf(stderr, "harness check %d failed\n", code);
        return code;
    }
    return 0;
}

int main(void)
{
    LinePathEvent out = {0};
    LineFeatures stale_features;
    LineEstimate estimate = {0};
    LineTrendResult trend = {0};
    int failure;

    LineEventClassifier_Init();
    failure = require(update_bits(0x18U, 1U, 100U, &out) &&
                      out.type == LINE_PATH_NORMAL, 1);
    if (failure != 0) return failure;
    failure = require(update_bits(0x18U, 2U, 105U, &out) &&
                      out.type == LINE_PATH_NORMAL, 2);
    if (failure != 0) return failure;
    failure = require(update_bits(0x18U, 3U, 110U, &out) &&
                      out.type == LINE_PATH_NORMAL, 3);
    if (failure != 0) return failure;
    failure = require(update_bits(0xFFU, 4U, 115U, &out) &&
                      out.type == LINE_PATH_NORMAL, 4);
    if (failure != 0) return failure;
    failure = require(update_bits(0xF8U, 5U, 120U, &out) &&
                      out.type == LINE_PATH_WIDE_PENDING &&
                      out.direction == 0 && out.direction_confidence == 0U, 5);
    if (failure != 0) return failure;
    failure = require(update_bits(0xF0U, 6U, 125U, &out) &&
                      out.type == LINE_PATH_RIGHT_ANGLE_RIGHT &&
                      out.direction == 1, 6);
    if (failure != 0) return failure;
    failure = require(update_bits(0x0FU, 7U, 130U, &out) &&
                      out.type == LINE_PATH_RIGHT_ANGLE_RIGHT &&
                      out.direction == 1, 7);
    if (failure != 0) return failure;

    LineEventClassifier_Reset();
    (void)update_bits(0x18U, 1U, 200U, &out);
    (void)update_bits(0x18U, 2U, 205U, &out);
    (void)update_bits(0x18U, 3U, 210U, &out);
    (void)update_bits(0xFFU, 4U, 215U, &out);
    failure = require(update_bits(0xFFU, 5U, 220U, &out) &&
                      out.type == LINE_PATH_WIDE_PENDING, 8);
    if (failure != 0) return failure;
    failure = require(update_bits(0x1CU, 6U, 225U, &out) &&
                      out.type == LINE_PATH_WIDE_PENDING &&
                      out.direction_confidence == 2U, 9);
    if (failure != 0) return failure;
    failure = require(update_bits(0x01U, 6U, 230U, &out) &&
                      out.type == LINE_PATH_WIDE_PENDING &&
                      out.direction_confidence == 2U, 10);
    if (failure != 0) return failure;
    failure = require(update_bits(0x1CU, 7U, 235U, &out) &&
                      out.type == LINE_PATH_RIGHT_ANGLE_LEFT &&
                      out.direction == -1, 11);
    if (failure != 0) return failure;

    LineEventClassifier_Reset();
    (void)update_bits(0x18U, 1U, 300U, &out);
    (void)update_bits(0x18U, 2U, 305U, &out);
    (void)update_bits(0x18U, 3U, 310U, &out);
    (void)update_bits(0xFFU, 4U, 315U, &out);
    (void)update_bits(0xFFU, 5U, 320U, &out);
    failure = require(update_bits(0x00U, 6U, 325U, &out) &&
                      out.type == LINE_PATH_WIDE_PENDING, 12);
    if (failure != 0) return failure;

    stale_features = make_features(0x18U, 20U, 400U);
    estimate.status = (ModuleStatus){421U, 20U, true, MODULE_HEALTH_OK};
    trend.status = (ModuleStatus){421U, 20U, true, MODULE_HEALTH_OK};
    failure = require(!LineEventClassifier_Update(
                          &stale_features, &estimate, &trend, 421U, &out) &&
                      out.type == LINE_PATH_INVALID && !out.status.valid, 13);
    if (failure != 0) return failure;
    failure = require(!LineEventClassifier_Update(
                          0, &estimate, &trend, 421U, &out) &&
                      out.type == LINE_PATH_INVALID && !out.status.valid, 14);
    if (failure != 0) return failure;

    LineEventClassifier_Reset();
    failure = require(update_bits(0x18U, 1U, 500U, &out) &&
                      out.type == LINE_PATH_NORMAL, 15);
    return failure;
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
            f'/I"{ROOT / "modules/line_tracking"}" '
            f'"{ROOT / "modules/line_tracking/line_event_classifier.c"}" '
            f'"{harness}" /Fe"{executable}"\n',
            encoding="utf-8",
        )
        result = subprocess.run(
            ["cmd", "/d", "/c", str(compile_script)],
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

    def test_direct_straight_to_wide_enters_pending(self):
        self.assertEqual(
            classify([0x18, 0x18, 0x18, 0xFF, 0xFF]),
            "pending",
        )

    def test_asymmetric_confirmation_frame_still_enters_pending(self):
        self.assertEqual(
            classify([0x18, 0x18, 0x18, 0xFF, 0xF8]),
            "pending",
        )

    def test_forward_probe_resolves_left_and_right_l_shapes(self):
        prefix = [0x18, 0x18, 0x18, 0xFF, 0xFF]
        self.assertEqual(classify(prefix + [0x0F]), "left")
        self.assertEqual(classify(prefix + [0xF0]), "right")

    def test_single_wide_glitch_is_not_a_corner(self):
        self.assertEqual(
            classify([0x18, 0x18, 0x18, 0xFF, 0x18]),
            "normal",
        )

    def test_public_events_and_reset_exist(self):
        header = (
            ROOT / "modules/line_tracking/line_event_classifier.h"
        ).read_text(encoding="utf-8")
        for token in (
            "LINE_PATH_WIDE_PENDING",
            "LINE_PATH_RIGHT_ANGLE_LEFT",
            "LINE_PATH_RIGHT_ANGLE_RIGHT",
            "direction_confidence",
            "LineEventClassifier_Reset",
        ):
            self.assertIn(token, header)

    def test_c_classifier_enforces_temporal_event_contract(self):
        result = subprocess.run([str(self.harness)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
