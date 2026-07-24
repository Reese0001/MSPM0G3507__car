# Single-Line Mixed Right-Angle Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Detect and negotiate left/right mixed 90-degree corners on one continuous black line without ever commanding whole-vehicle reverse recovery.

**Architecture:** Add a fixed-cost feature extractor and a temporal path-event classifier ahead of the existing trend and PD controllers. A dedicated corner state machine owns motion during right-angle maneuvers, while a simplified recovery state machine handles ordinary line loss using forward arcs and bounded pivot search only. Existing safety supervision remains the final motor-command authority.

**Tech Stack:** C11-style embedded C, TI DriverLib, TI Arm Clang 4.0.4 LTS for Cortex-M0+, Python `unittest` contract/reference tests, CCS Theia/SysConfig, UniFlash Intel HEX and TI-TXT artifacts.

## Global Constraints

- Work only in `D:\DevProject\MSPM0G3507__car\.worktrees\line-following-burn` on branch `codex/line-following-burn`; never modify `main`.
- Hardware in scope is the eight-channel line sensor and two L-type 520 drive motors; MPU6050, ultrasonic, K230, and wireless modules remain disabled.
- The track is one continuous black line with left/right mixed 90-degree corners; do not implement T-junction or crossroad route selection.
- Recovery must never command both wheels negative; whole-vehicle backtracking is removed.
- A pivot may command one wheel positive and the other negative only after the existing 120 ms direction-change pause.
- Every requested wheel command must stay inside `[-450, 450]`.
- Preserve soft-start, the 200 ms motor-command watchdog, request expiry, emergency stop, stale-sensor stop, and fault latching.
- The 5 ms line-control task must remain nonblocking and use fixed memory; no dynamic allocation, trigonometric S-curve, or blocking delay.
- Keep the initial normal-follow proportional gain at `KP = 28.0f` and `KI = 0`.
- Each task below ends in one focused Git commit. Do not stage known unrelated untracked files in `firmware/`, `diagnostics/`, `995bfa4_snapshot.zip`, or `tests/test_led_diagnostic.py`.

---

## File Structure

### New files

- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_features.h` — public immutable feature-frame type and extractor API.
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_features.c` — O(8) bit-pattern feature extraction and error-rate history.
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_feature_config.h` — feature freshness and confidence constants.
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_event_classifier.h` — temporal path-event API.
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_event_classifier.c` — wide-feature confirmation and left/right evidence accumulation.
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_event_config.h` — confirmation and direction-score thresholds.
- `MSPM0G3507_LineFollowing_Car/application/corner_maneuver.h` — bounded corner state-machine API.
- `MSPM0G3507_LineFollowing_Car/application/corner_maneuver.c` — forward probe, brake, commit, seek, and settle behavior.
- `MSPM0G3507_LineFollowing_Car/application/config/corner_maneuver_config.h` — corner speeds and timing.
- `tests/test_line_features.py` — feature extractor contract and pure reference oracle.
- `tests/test_line_event_classifier.py` — mixed-corner temporal classification tests.
- `tests/test_corner_maneuver.py` — corner ownership, timing, mirror, and safety tests.

### Modified files

- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_estimator.h`
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_estimator.c`
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.h`
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.c`
- `MSPM0G3507_LineFollowing_Car/application/line_recovery.h`
- `MSPM0G3507_LineFollowing_Car/application/line_recovery.c`
- `MSPM0G3507_LineFollowing_Car/application/config/line_recovery_config.h`
- `MSPM0G3507_LineFollowing_Car/application/app_scheduler.c`
- `tests/test_line_estimator.py`
- `tests/test_line_trend_detector.py`
- `tests/test_line_recovery.py`
- `tests/test_app_scheduler.py`

---

## Exact TI Compile Helper

For every TI-compile step in this plan, start in the worktree root and define
this PowerShell helper in the current shell:

```powershell
function Invoke-TiCompile {
    param([string[]]$Sources)

    $compiler = 'D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmclang.exe'
    $project = (Resolve-Path 'MSPM0G3507_LineFollowing_Car').Path
    $build = Join-Path $project 'Build_LineFollowing'
    $sdk = 'C:\ti\mspm0_sdk_2_10_00_04'
    $includes = @(
        $project,
        $build,
        (Join-Path $sdk 'source'),
        (Join-Path $sdk 'source\third_party\CMSIS\Core\Include'),
        (Join-Path $project 'application'),
        (Join-Path $project 'modules\common'),
        (Join-Path $project 'modules\motor'),
        (Join-Path $project 'modules\line_tracking'),
        (Join-Path $project 'modules\led'),
        (Join-Path $project 'modules\buzzer'),
        (Join-Path $project 'modules\key'),
        (Join-Path $project 'modules\ultrasonic'),
        (Join-Path $project 'modules\ybimu'),
        (Join-Path $project 'modules\k230_link'),
        (Join-Path $project 'bsp'),
        (Join-Path $project 'bsp\time')
    )
    $base = @(
        '-mcpu=cortex-m0plus', '-mthumb', '-mfloat-abi=soft',
        '-O2', '-gdwarf-3', '-Wall',
        ('@' + (Join-Path $build 'device.opt'))
    )
    foreach ($include in $includes) {
        $base += '-I' + $include
    }
    foreach ($source in $Sources) {
        $objectName = ($source -replace '[\\/.]', '_') -replace '_c$', '.o'
        $objectPath = Join-Path $build ('obj_minimal\' + $objectName)
        & $compiler @base '-c' (Join-Path $project $source) '-o' $objectPath
        if ($LASTEXITCODE -ne 0) {
            throw ('Compile failed: ' + $source)
        }
    }
    Write-Output ('compiled_units=' + $Sources.Count)
}
```

The expected result of every invocation is the requested `compiled_units=N`
line with no compiler diagnostics.

---

### Task 1: Extract Stable Line Features Before Estimation

**Files:**

- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_features.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_features.c`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_feature_config.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_estimator.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_estimator.c`
- Create: `tests/test_line_features.py`
- Modify: `tests/test_line_estimator.py`

**Interfaces:**

- Consumes: `LineSensorSnapshot` and `ModuleStatus_IsFresh()`.
- Produces:

```c
typedef struct {
    ModuleStatus status;
    uint8_t black_bits;
    uint8_t active_count;
    uint8_t left_count;
    uint8_t right_count;
    uint8_t span;
    uint8_t segment_count;
    bool left_edge;
    bool right_edge;
    float centroid_error;
    float error_rate;
    uint8_t confidence;
} LineFeatures;

void LineFeatureExtractor_Init(void);
void LineFeatureExtractor_Reset(void);
bool LineFeatureExtractor_Update(const LineSensorSnapshot *snapshot,
                                 uint32_t now_ms,
                                 LineFeatures *out);
bool LineEstimator_Update(const LineFeatures *features, uint32_t now_ms);
```

- Later tasks consume `LineFeatures` without reading raw GPIO or reinterpreting black/white polarity.

- [ ] **Step 1: Write the failing feature tests**

Create `tests/test_line_features.py`:

```python
import unittest
from pathlib import Path

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
        header = (ROOT / "modules/line_tracking/line_features.h").read_text(
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
        source = (ROOT / "modules/line_tracking/line_features.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("ModuleStatus_IsFresh", source)
        self.assertIn("snapshot->status.sequence", source)
        self.assertIn("previous_sequence", source)
        self.assertIn("previous_timestamp_ms", source)


if __name__ == "__main__":
    unittest.main()
```

In `tests/test_line_estimator.py`, change estimator contract assertions from
`const LineSensorSnapshot *snapshot` to `const LineFeatures *features`, and
assert that `line_estimator.c` reads `features->centroid_error`,
`features->error_rate`, `features->active_count`, and `features->confidence`.

- [ ] **Step 2: Run tests and verify the intended failure**

Run:

```powershell
python -m unittest tests.test_line_features tests.test_line_estimator
```

Expected: FAIL because `line_features.{h,c}` do not exist and the estimator
still consumes `LineSensorSnapshot`.

- [ ] **Step 3: Add the feature API and configuration**

Create `line_feature_config.h`:

```c
#ifndef LINE_FEATURE_CONFIG_H
#define LINE_FEATURE_CONFIG_H

#define LINE_FEATURE_STALE_MS (20U)
#define LINE_FEATURE_JUMP_ERROR (3.0f)
#define LINE_FEATURE_GROUP_PENALTY (20)
#define LINE_FEATURE_WIDE_PENALTY (10)
#define LINE_FEATURE_JUMP_PENALTY (15)

#endif
```

Create `line_features.h` with the `LineFeatures` declaration and exact
function signatures from the Interfaces section. Include `<stdbool.h>`,
`<stdint.h>`, and `"line_scanner.h"`.

- [ ] **Step 4: Implement fixed-cost feature extraction**

In `line_features.c`, use the existing weight convention and these helpers:

```c
static const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
static float previous_error;
static uint32_t previous_timestamp_ms;
static uint16_t previous_sequence;
static bool has_history;

static uint8_t clamp_confidence(int16_t value)
{
    if (value < 0) {
        return 0U;
    }
    if (value > 100) {
        return 100U;
    }
    return (uint8_t)value;
}

void LineFeatureExtractor_Reset(void)
{
    previous_error = 0.0f;
    previous_timestamp_ms = 0U;
    previous_sequence = 0U;
    has_history = false;
}

void LineFeatureExtractor_Init(void)
{
    LineFeatureExtractor_Reset();
}
```

`LineFeatureExtractor_Update()` must:

1. Reject null output, null snapshot, and stale status.
2. Copy the snapshot status and `black_bits`.
3. Walk exactly eight bits once to compute counts, first/last index,
   contiguous segments, weighted sum, and edge flags.
4. Set `span = 0` for no active bits.
5. Preserve `previous_error` as the lost-frame centroid when no bit is active.
6. Calculate error rate only when sequence and timestamp both advance.
7. Penalize multiple segments, more than three active bits, and jumps larger
   than `LINE_FEATURE_JUMP_ERROR`.
8. Update history only for new, nonempty frames.

Use:

```c
if (out->active_count != 0U) {
    out->centroid_error =
        (float)weighted_sum / (float)out->active_count;
} else {
    out->centroid_error = has_history ? previous_error : 0.0f;
}
out->error_rate = 0.0f;
if (has_history &&
    snapshot->status.sequence != previous_sequence &&
    snapshot->status.timestamp_ms != previous_timestamp_ms) {
    uint32_t delta_ms =
        snapshot->status.timestamp_ms - previous_timestamp_ms;
    out->error_rate =
        (out->centroid_error - previous_error) * 1000.0f /
        (float)delta_ms;
}
```

- [ ] **Step 5: Make the estimator consume features**

Change `LineEstimator_Update()` to accept `const LineFeatures *features`.
Replace its bit loop and private confidence function with:

```c
if (features == 0 ||
    !ModuleStatus_IsFresh(&features->status, now_ms,
                          LINE_FEATURE_STALE_MS)) {
    publish_fault(now_ms);
    return false;
}

latest_estimate.status = features->status;
latest_estimate.status.sequence++;
latest_estimate.error = features->centroid_error;
latest_estimate.derivative = features->error_rate;
latest_estimate.predicted_error = clamp_error(
    features->centroid_error +
    features->error_rate * LINE_PREDICTION_HORIZON_S);
latest_estimate.confidence = features->confidence;
```

For `active_count == 0`, publish `LINE_EVENT_LOST` with degraded health.
Keep `LINE_EVENT_WIDE_BLACK` at `active_count >= 6U` for backward-compatible
speed limiting; the new event classifier in Task 2 owns the 4-light corner
meaning. Preserve hard-left/right thresholds at centroid `-4.0f/+4.0f`.

- [ ] **Step 6: Run focused tests and compile the two modules**

Run:

```powershell
python -m unittest tests.test_line_features tests.test_line_estimator
python -m unittest discover -s tests -p 'test_*.py'
git diff --check
```

Expected: all tests PASS.

After defining the Exact TI Compile Helper, run:

```powershell
Invoke-TiCompile @(
  'modules\line_tracking\line_features.c',
  'modules\line_tracking\line_estimator.c'
)
```

Expected: `compiled_units=2` with no diagnostics.

- [ ] **Step 7: Commit the feature layer**

```powershell
git add -- `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_features.h `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_features.c `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_feature_config.h `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_estimator.h `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_estimator.c `
  tests/test_line_features.py `
  tests/test_line_estimator.py
git commit -m "feat: extract temporal line features"
```

---

### Task 2: Classify Direct Wide-Feature Corners and Direction

**Files:**

- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_event_classifier.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_event_classifier.c`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_event_config.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.c`
- Create: `tests/test_line_event_classifier.py`
- Modify: `tests/test_line_trend_detector.py`

**Interfaces:**

- Consumes: fresh `LineFeatures`, `LineEstimate`, and `LineTrendResult`.
- Produces:

```c
typedef enum {
    LINE_PATH_NORMAL = 0,
    LINE_PATH_WIDE_PENDING,
    LINE_PATH_RIGHT_ANGLE_LEFT,
    LINE_PATH_RIGHT_ANGLE_RIGHT,
    LINE_PATH_LOST,
    LINE_PATH_INVALID
} LinePathEventType;

typedef struct {
    ModuleStatus status;
    LinePathEventType type;
    int8_t direction;
    uint8_t direction_confidence;
} LinePathEvent;

void LineEventClassifier_Init(void);
void LineEventClassifier_Reset(void);
bool LineEventClassifier_Update(const LineFeatures *features,
                                const LineEstimate *estimate,
                                const LineTrendResult *trend,
                                uint32_t now_ms,
                                LinePathEvent *out);
```

- `LINE_PATH_WIDE_PENDING` lets Task 3 begin forward probing before direction
  is known. Confirmed left/right remains latched until explicit reset.

- [ ] **Step 1: Write failing temporal classification tests**

Create `tests/test_line_event_classifier.py`:

```python
import unittest
from pathlib import Path

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
                total += score(bits)
        else:
            stable = wide = 0
    return "pending" if candidate else "normal"


class LineEventClassifierContract(unittest.TestCase):
    def test_direct_straight_to_wide_enters_pending(self):
        self.assertEqual(
            classify([0x18, 0x18, 0x18, 0xFF, 0xFF]),
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


if __name__ == "__main__":
    unittest.main()
```

Update `tests/test_line_trend_detector.py`: direct straight-to-wide is no
longer expected to produce a right-angle trend. Trend tests continue to cover
tight/hairpin curves and outward-sequence compatibility; the new classifier
owns direct wide corners.

- [ ] **Step 2: Run the focused tests and observe failure**

```powershell
python -m unittest tests.test_line_event_classifier tests.test_line_trend_detector
```

Expected: FAIL because the path-event module does not exist.

- [ ] **Step 3: Add exact classifier thresholds and API**

Create `line_event_config.h`:

```c
#ifndef LINE_EVENT_CONFIG_H
#define LINE_EVENT_CONFIG_H

#define LINE_EVENT_STABLE_SINGLE_FRAMES (3U)
#define LINE_EVENT_WIDE_CONFIRM_FRAMES (2U)
#define LINE_EVENT_WIDE_ACTIVE_COUNT (4U)
#define LINE_EVENT_WIDE_MIN_SPAN (4U)
#define LINE_EVENT_DIRECTION_THRESHOLD (4)
#define LINE_EVENT_SIDE_COUNT_WEIGHT (2)
#define LINE_EVENT_EDGE_WEIGHT (3)
#define LINE_EVENT_MIN_CONFIDENCE (40U)
#define LINE_EVENT_STALE_MS (20U)

#endif
```

Create `line_event_classifier.h` with the exact types and signatures from the
Interfaces section.

- [ ] **Step 4: Implement saturating confirmation and direction evidence**

Use module state:

```c
static uint8_t stable_single_frames;
static uint8_t wide_frames;
static uint16_t previous_sequence;
static int16_t direction_score;
static bool corner_candidate;
static LinePathEventType latched_corner;
```

Use this per-frame evidence:

```c
static int16_t frame_direction_score(const LineFeatures *features)
{
    int16_t score =
        (int16_t)(features->right_count - features->left_count) *
        LINE_EVENT_SIDE_COUNT_WEIGHT;
    if (features->right_edge) {
        score += LINE_EVENT_EDGE_WEIGHT;
    }
    if (features->left_edge) {
        score -= LINE_EVENT_EDGE_WEIGHT;
    }
    return score;
}
```

Rules inside `LineEventClassifier_Update()`:

- Reject stale/null inputs with `LINE_PATH_INVALID`.
- Return the previous classification without changing counters when
  `features->status.sequence == previous_sequence`.
- If a corner is latched, republish it until reset.
- Count stable single-line frames only for `active_count` 1～3 and confidence
  at least 40.
- Count wide frames only after stable evidence and only when
  `active_count >= 4` and `span >= 4`.
- Two consecutive wide frames enter `LINE_PATH_WIDE_PENDING`.
- While pending, add `frame_direction_score()` on every new frame.
- Score `<= -4` latches left; score `>= +4` latches right.
- A lost frame while pending keeps pending; it must not become ordinary loss.
- Outside a candidate, zero active bits publish `LINE_PATH_LOST`.
- Reset all evidence in `LineEventClassifier_Reset()`.

Direction confidence is:

```c
uint16_t magnitude = direction_score < 0 ?
    (uint16_t)(-direction_score) : (uint16_t)direction_score;
out->direction_confidence =
    magnitude >= 100U ? 100U : (uint8_t)magnitude;
```

- [ ] **Step 5: Remove right-angle ownership from trend detection**

Keep the public right-angle enum values temporarily for source compatibility,
but delete the branch that creates them from `line_trend_detector.c`.
`line_trend_detector` must continue classifying:

- `LINE_TREND_NORMAL`
- `LINE_TREND_TIGHT_LEFT/RIGHT`
- `LINE_TREND_HAIRPIN_LEFT/RIGHT`

Update tests to assert that `active_count >= 4` no longer requires outward
steps and is consumed by `LineEventClassifier_Update()` instead.

- [ ] **Step 6: Run tests and compile the classifier**

```powershell
python -m unittest tests.test_line_event_classifier tests.test_line_trend_detector
python -m unittest discover -s tests -p 'test_*.py'
git diff --check
```

Expected: all tests PASS. After defining the Exact TI Compile Helper, run:

```powershell
Invoke-TiCompile @(
  'modules\line_tracking\line_event_classifier.c',
  'modules\line_tracking\line_trend_detector.c'
)
```

Expected: `compiled_units=2` with no diagnostics.

- [ ] **Step 7: Commit event classification**

```powershell
git add -- `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_event_classifier.h `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_event_classifier.c `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_event_config.h `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.h `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.c `
  tests/test_line_event_classifier.py `
  tests/test_line_trend_detector.py
git commit -m "feat: classify mixed right-angle events"
```

---

### Task 3: Add a Dedicated Forward-Probe Corner State Machine

**Files:**

- Create: `MSPM0G3507_LineFollowing_Car/application/corner_maneuver.h`
- Create: `MSPM0G3507_LineFollowing_Car/application/corner_maneuver.c`
- Create: `MSPM0G3507_LineFollowing_Car/application/config/corner_maneuver_config.h`
- Create: `tests/test_corner_maneuver.py`

**Interfaces:**

- Consumes: fresh `LineFeatures`, `LinePathEvent`, normal
  `LineControlOutput`, emergency-stop flag, and `now_ms`.
- Produces:

```c
typedef enum {
    CORNER_MANEUVER_FOLLOW = 0,
    CORNER_MANEUVER_FORWARD_PROBE,
    CORNER_MANEUVER_BRAKE,
    CORNER_MANEUVER_COMMIT,
    CORNER_MANEUVER_SEEK,
    CORNER_MANEUVER_SETTLE,
    CORNER_MANEUVER_FAULT
} CornerManeuverState;

typedef struct {
    MotionRequest request;
    bool owns_motion;
    bool completed;
    bool fault;
} CornerManeuverOutput;

void CornerManeuver_Init(void);
void CornerManeuver_Reset(void);
CornerManeuverState CornerManeuver_GetState(void);
bool CornerManeuver_Step(const LineFeatures *features,
                         const LinePathEvent *path_event,
                         const LineControlOutput *follow,
                         bool emergency_stop,
                         uint32_t now_ms,
                         CornerManeuverOutput *out);
```

- `completed` is a one-cycle pulse used by Task 5 to reset event/trend/PD
  histories after stable reacquisition.

- [ ] **Step 1: Write failing corner state-machine tests**

Create `tests/test_corner_maneuver.py`:

```python
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class CornerManeuverContract(unittest.TestCase):
    def setUp(self):
        self.header = (ROOT / "application/corner_maneuver.h").read_text(
            encoding="utf-8"
        )
        self.source = (ROOT / "application/corner_maneuver.c").read_text(
            encoding="utf-8"
        )
        self.config = (
            ROOT / "application/config/corner_maneuver_config.h"
        ).read_text(encoding="utf-8")

    def test_states_cover_probe_brake_commit_seek_settle(self):
        for token in (
            "CORNER_MANEUVER_FORWARD_PROBE",
            "CORNER_MANEUVER_BRAKE",
            "CORNER_MANEUVER_COMMIT",
            "CORNER_MANEUVER_SEEK",
            "CORNER_MANEUVER_SETTLE",
            "CORNER_MANEUVER_FAULT",
        ):
            self.assertIn(token, self.header)

    def test_probe_is_forward_only_and_bounded(self):
        self.assertIn("CORNER_PROBE_COMMAND (100)", self.config)
        self.assertIn("CORNER_PROBE_MAX_MS (80U)", self.config)
        self.assertRegex(
            self.source,
            r"publish_request\(CORNER_PROBE_COMMAND,\s*"
            r"CORNER_PROBE_COMMAND",
        )

    def test_left_and_right_pivots_are_mirrors(self):
        self.assertIn("CORNER_INNER_COMMAND (-80)", self.config)
        self.assertIn("CORNER_OUTER_COMMAND (120)", self.config)
        self.assertIn("corner_direction < 0", self.source)

    def test_completion_requires_three_new_centered_frames(self):
        self.assertIn("CORNER_REACQUIRE_FRAMES (3U)", self.config)
        self.assertIn("last_feature_sequence", self.source)
        self.assertIn("reacquire_frames", self.source)

    def test_every_state_has_a_total_timeout(self):
        self.assertIn("CORNER_TOTAL_TIMEOUT_MS (2000U)", self.config)
        self.assertIn("enter_fault", self.source)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test and verify failure**

```powershell
python -m unittest tests.test_corner_maneuver
```

Expected: FAIL because the corner module does not exist.

- [ ] **Step 3: Add exact initial corner parameters**

Create `corner_maneuver_config.h`:

```c
#ifndef CORNER_MANEUVER_CONFIG_H
#define CORNER_MANEUVER_CONFIG_H

#define CORNER_PROBE_COMMAND (100)
#define CORNER_PROBE_MAX_MS (80U)
#define CORNER_BRAKE_MS (120U)
#define CORNER_COMMIT_MS (100U)
#define CORNER_SEEK_MAX_MS (900U)
#define CORNER_SETTLE_MS (300U)
#define CORNER_TOTAL_TIMEOUT_MS (2000U)
#define CORNER_INNER_COMMAND (-80)
#define CORNER_OUTER_COMMAND (120)
#define CORNER_REACQUIRE_FRAMES (3U)
#define CORNER_CENTER_ERROR (3.0f)
#define CORNER_MIN_CONFIDENCE (40U)
#define CORNER_FEATURE_STALE_MS (20U)

#endif
```

Create `corner_maneuver.h` with the exact Interfaces declarations and includes
for motion request, line controller, line event classifier, and line features.

- [ ] **Step 4: Implement nonblocking corner ownership**

Use module state:

```c
static CornerManeuverState state;
static int8_t corner_direction;
static uint8_t reacquire_frames;
static uint16_t last_feature_sequence;
static uint32_t maneuver_started_ms;
static uint32_t state_started_ms;
static bool completion_pending;
```

Implement request helpers:

```c
static void set_pivot(uint32_t now_ms, MotionRequest *request)
{
    if (corner_direction < 0) {
        publish_request(CORNER_INNER_COMMAND,
                        CORNER_OUTER_COMMAND, now_ms, request);
    } else {
        publish_request(CORNER_OUTER_COMMAND,
                        CORNER_INNER_COMMAND, now_ms, request);
    }
}
```

State transitions:

- `FOLLOW + WIDE_PENDING` → `FORWARD_PROBE`, request `100/100`.
- `FOLLOW + confirmed left/right` → `BRAKE`, request `0/0`.
- `FORWARD_PROBE + confirmed direction` → `BRAKE`.
- `FORWARD_PROBE` lasting 80 ms without direction → `FAULT`.
- `BRAKE` holds `0/0` for 120 ms, then enters `COMMIT`.
- `COMMIT` applies mirrored `-80/120` or `120/-80` for 100 ms.
- `SEEK` continues the same pivot for at most 900 ms.
- Three new frames with 1～3 active bits, confidence at least 40, and
  `abs(centroid_error) <= 3` enter `SETTLE`.
- `SETTLE` publishes the normal follow output for 300 ms, then sets
  `completed=true` for one cycle and returns to `FOLLOW`.
- Null/stale input, emergency stop, total timeout, or seek timeout enters
  `FAULT`, publishes an invalid zero request, and sets `fault=true`.

Use `features->status.sequence != last_feature_sequence` before incrementing
`reacquire_frames`. Do not use blocking delays.

- [ ] **Step 5: Run tests and TI-compile the corner module**

```powershell
python -m unittest tests.test_corner_maneuver
python -m unittest discover -s tests -p 'test_*.py'
git diff --check
```

Expected: all tests PASS. After defining the Exact TI Compile Helper, run:

```powershell
Invoke-TiCompile @('application\corner_maneuver.c')
```

Expected: `compiled_units=1` with no diagnostics.

- [ ] **Step 6: Commit the corner maneuver**

```powershell
git add -- `
  MSPM0G3507_LineFollowing_Car/application/corner_maneuver.h `
  MSPM0G3507_LineFollowing_Car/application/corner_maneuver.c `
  MSPM0G3507_LineFollowing_Car/application/config/corner_maneuver_config.h `
  tests/test_corner_maneuver.py
git commit -m "feat: add forward-probe corner maneuver"
```

---

### Task 4: Remove Backtracking From Ordinary Line Recovery

**Files:**

- Modify: `MSPM0G3507_LineFollowing_Car/application/line_recovery.h`
- Modify: `MSPM0G3507_LineFollowing_Car/application/line_recovery.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/config/line_recovery_config.h`
- Modify: `tests/test_line_recovery.py`

**Interfaces:**

- Keep `LineRecovery_Step()` signature unchanged for scheduler compatibility.
- Remove right-angle ownership and all whole-vehicle reverse behavior.
- New states:

```c
typedef enum {
    LINE_RECOVERY_FOLLOW = 0,
    LINE_RECOVERY_LOSS_CONFIRM,
    LINE_RECOVERY_FORWARD_SEARCH,
    LINE_RECOVERY_ROTATION_PAUSE,
    LINE_RECOVERY_ROTATE_SEARCH,
    LINE_RECOVERY_ALIGN,
    LINE_RECOVERY_FAULT
} LineRecoveryState;
```

- [ ] **Step 1: Replace backtrack tests with no-reverse properties**

Rewrite `tests/test_line_recovery.py` assertions to require:

```python
def test_recovery_has_no_backtrack_state_or_request(self):
    self.assertNotIn("LINE_RECOVERY_BACKTRACK", self.header)
    self.assertNotIn("set_backtrack_request", self.source)
    self.assertNotIn("LINE_BACKTRACK_MS", self.config)

def test_recovery_uses_forward_then_pause_then_rotate(self):
    for token in (
        "LINE_RECOVERY_FORWARD_SEARCH",
        "LINE_RECOVERY_ROTATION_PAUSE",
        "LINE_RECOVERY_ROTATE_SEARCH",
        "LINE_RECOVERY_ALIGN",
    ):
        self.assertIn(token, self.header)
    for token in (
        "LINE_FORWARD_SEARCH_MS (500U)",
        "LINE_ROTATION_PAUSE_MS (120U)",
        "LINE_ROTATE_SEARCH_MS (700U)",
        "LINE_ROTATE_INNER_COMMAND (-60)",
        "LINE_ROTATE_OUTER_COMMAND (100)",
    ):
        self.assertIn(token, self.config)

def test_no_recovery_helper_commands_both_wheels_negative(self):
    pairs = [
        tuple(map(int, pair))
        for pair in re.findall(
            r"publish_request\(\s*(-?\d+|LINE_[A-Z_]+),\s*"
            r"(-?\d+|LINE_[A-Z_]+)",
            self.source,
        )
        if all(value.lstrip("-").isdigit() for value in pair)
    ]
    self.assertFalse(any(left < 0 and right < 0 for left, right in pairs))
    self.assertNotRegex(
        self.source,
        r"publish_request\(\s*-LINE_SEARCH_[A-Z_]+,\s*"
        r"-LINE_SEARCH_[A-Z_]+",
    )
```

Keep existing tests for stale data, emergency stop, three-frame reacquisition,
alignment timing, command limits, and direction locking during one recovery.

- [ ] **Step 2: Run tests and verify expected failures**

```powershell
python -m unittest tests.test_line_recovery
```

Expected: FAIL because backtracking states and commands still exist.

- [ ] **Step 3: Replace recovery configuration**

Use:

```c
#define LINE_LOSS_CONFIRM_COUNT (3U)
#define LINE_REACQUIRE_COUNT (3U)
#define LINE_FORWARD_SEARCH_MS (500U)
#define LINE_ROTATION_PAUSE_MS (120U)
#define LINE_ROTATE_SEARCH_MS (700U)
#define LINE_ALIGN_DURATION_MS (300U)
#define LINE_RECOVERY_TOTAL_TIMEOUT_MS (2000U)
#define LINE_RECOVERY_ESTIMATE_STALE_MS (20U)
#define LINE_RECOVERY_MIN_CONFIDENCE (40U)
#define LINE_RECOVERY_CENTER_ERROR (3.0f)

#define LINE_SEARCH_INNER_COMMAND (80)
#define LINE_SEARCH_OUTER_COMMAND (120)
#define LINE_SEARCH_STRAIGHT_COMMAND (100)
#define LINE_ROTATE_INNER_COMMAND (-60)
#define LINE_ROTATE_OUTER_COMMAND (100)
```

Delete `LINE_BACKTRACK_MS`, `LINE_CORNER_*`, and obsolete yaw-limit constants.

- [ ] **Step 4: Implement forward-only recovery transitions**

Delete `trend_is_right_angle()`, `set_backtrack_request()`, and
`set_corner_request()`. `FOLLOW` now only follows or enters loss confirmation.

Use:

```c
static void set_forward_search(uint32_t now_ms, MotionRequest *request)
{
    if (recovery_direction < 0) {
        publish_request(LINE_SEARCH_INNER_COMMAND,
                        LINE_SEARCH_OUTER_COMMAND, now_ms, request);
    } else if (recovery_direction > 0) {
        publish_request(LINE_SEARCH_OUTER_COMMAND,
                        LINE_SEARCH_INNER_COMMAND, now_ms, request);
    } else {
        publish_request(LINE_SEARCH_STRAIGHT_COMMAND,
                        LINE_SEARCH_STRAIGHT_COMMAND, now_ms, request);
    }
}

static void set_rotate_search(uint32_t now_ms, MotionRequest *request)
{
    if (recovery_direction < 0) {
        publish_request(LINE_ROTATE_INNER_COMMAND,
                        LINE_ROTATE_OUTER_COMMAND, now_ms, request);
    } else if (recovery_direction > 0) {
        publish_request(LINE_ROTATE_OUTER_COMMAND,
                        LINE_ROTATE_INNER_COMMAND, now_ms, request);
    } else {
        invalidate_request(request, now_ms);
    }
}
```

Transitions:

- Confirm loss over three new estimate frames.
- Forward search for 500 ms.
- Hold `0/0` for 120 ms before a wheel reverses.
- Rotate search for at most 700 ms.
- Three trustworthy new frames enter alignment.
- Alignment runs for 300 ms, then returns to follow.
- No direction evidence at rotation time or any timeout enters fault.

- [ ] **Step 5: Run tests and compile recovery**

```powershell
python -m unittest tests.test_line_recovery
python -m unittest discover -s tests -p 'test_*.py'
git diff --check
```

Expected: all tests PASS. After defining the Exact TI Compile Helper, run:

```powershell
Invoke-TiCompile @('application\line_recovery.c')
```

Expected: `compiled_units=1` with no diagnostics.

- [ ] **Step 6: Commit no-backtrack recovery**

```powershell
git add -- `
  MSPM0G3507_LineFollowing_Car/application/line_recovery.h `
  MSPM0G3507_LineFollowing_Car/application/line_recovery.c `
  MSPM0G3507_LineFollowing_Car/application/config/line_recovery_config.h `
  tests/test_line_recovery.py
git commit -m "fix: remove reverse line recovery"
```

---

### Task 5: Integrate Feature, Event, Corner, and Recovery Ownership

**Files:**

- Modify: `MSPM0G3507_LineFollowing_Car/application/app_scheduler.c`
- Modify: `tests/test_app_scheduler.py`
- Modify: `tests/test_line_estimator.py`

**Interfaces:**

- Pipeline:

```text
snapshot -> features -> estimate -> trend -> path event -> PD
                                                   |
                                      corner owns? yes -> request
                                                   no
                                                   v
                                                recovery
```

- Completion resets corner-specific histories; fault invalidates the mission
  request before safety supervision.

- [ ] **Step 1: Write failing scheduler pipeline tests**

Add to `tests/test_app_scheduler.py`:

```python
def test_line_pipeline_order_and_motion_ownership(self):
    source = (ROOT / "application/app_scheduler.c").read_text(
        encoding="utf-8"
    )
    calls = [
        "LineFeatureExtractor_Update",
        "LineEstimator_Update",
        "LineTrendDetector_Update",
        "LineEventClassifier_Update",
        "LineController_Step",
        "CornerManeuver_Step",
        "LineRecovery_Step",
    ]
    positions = [source.index(call) for call in calls]
    self.assertEqual(positions, sorted(positions))
    self.assertIn("corner_output.owns_motion", source)
    self.assertIn("mission_request = corner_output.request", source)

def test_all_new_modules_reset_at_start_and_corner_completion(self):
    source = (ROOT / "application/app_scheduler.c").read_text(
        encoding="utf-8"
    )
    for token in (
        "LineFeatureExtractor_Reset",
        "LineEventClassifier_Reset",
        "CornerManeuver_Reset",
        "LineTrendDetector_Reset",
        "LineController_Reset",
        "LineRecovery_Reset",
    ):
        self.assertGreaterEqual(source.count(token), 2)

def test_corner_or_recovery_fault_fails_closed(self):
    source = (ROOT / "application/app_scheduler.c").read_text(
        encoding="utf-8"
    )
    self.assertIn("corner_output.fault", source)
    self.assertIn("LineRecovery_GetState() == LINE_RECOVERY_FAULT", source)
    self.assertRegex(
        source,
        r"mission_request\.left_speed\s*=\s*0;[\s\S]{0,120}"
        r"mission_request\.right_speed\s*=\s*0;",
    )
    self.assertIn("LED_ON()", source)
    self.assertIn("LED_OFF()", source)

def test_integration_keeps_confirmed_pd_and_speed_limits(self):
    config = (
        ROOT / "application/config/line_control_config.h"
    ).read_text(encoding="utf-8")
    safety = (
        ROOT / "application/config/safety_config.h"
    ).read_text(encoding="utf-8")
    self.assertIn("LINE_CONTROL_KP (28.0f)", config)
    self.assertIn("LINE_MAX_FORWARD (400)", config)
    self.assertIn("SAFETY_RUNNING_SPEED_LIMIT (450)", safety)
```

- [ ] **Step 2: Run integration contracts and observe failure**

```powershell
python -m unittest tests.test_app_scheduler tests.test_line_estimator
```

Expected: FAIL because the scheduler still calls the old estimator directly
from `LineSensorSnapshot` and has no event/corner ownership.

- [ ] **Step 3: Add scheduler state and initialization**

Add includes for `corner_maneuver.h`, `line_features.h`,
`line_event_classifier.h`, and `../modules/led/led.h`. Add:

```c
static LineFeatures line_features = {0};
static LinePathEvent path_event = {0};
static CornerManeuverOutput corner_output = {0};
```

Initialize all three modules in `AppScheduler_Init()`. Reset them in
`AppScheduler_Start()` together with the existing controller, trend, recovery,
and safety reset. Call `LED_OFF()` at start/reset. On a corner or recovery
fault call `LED_ON()` before publishing the invalid zero mission request.
This gives D1 a deterministic stuck/fault indication while D2 retains its
existing heartbeat.

- [ ] **Step 4: Replace the line-control pipeline**

Inside `AppScheduler_RunLineControl()` use:

```c
bool feature_ready = false;
bool estimate_ready = false;
bool trend_ready = false;
bool event_ready = false;

if (LineScanner_GetSnapshot(&scanner)) {
    feature_ready = LineFeatureExtractor_Update(
        &scanner, now_ms, &line_features);
}
if (feature_ready) {
    estimate_ready = LineEstimator_Update(&line_features, now_ms) &&
                     LineEstimator_Get(&line_estimate);
}
if (estimate_ready) {
    trend_ready = LineTrendDetector_Update(
        &line_estimate, &scanner, now_ms, &line_trend);
}
if (trend_ready) {
    event_ready = LineEventClassifier_Update(
        &line_features, &line_estimate, &line_trend, now_ms, &path_event);
}
```

When `event_ready`, run the normal controller first, then:

```c
(void)CornerManeuver_Step(
    &line_features, &path_event, &line_control, false,
    now_ms, &corner_output);
if (corner_output.owns_motion) {
    mission_request = corner_output.request;
} else {
    (void)LineRecovery_Step(
        &line_estimate, &line_trend, &line_control,
        0.0F, false, false, now_ms, &mission_request);
}
```

On `corner_output.completed`, reset feature/event/trend/controller/recovery
histories, but do not reinitialize the motor safety layer. On corner or
recovery fault, publish zero invalid motion, reset algorithm state, and let
the safety supervisor fail closed.

- [ ] **Step 5: Run complete regression and static safety scans**

```powershell
python -m unittest discover -s tests -p 'test_*.py'
rg -n "BACKTRACK|set_backtrack_request|LINE_BACKTRACK_MS" `
  MSPM0G3507_LineFollowing_Car/application `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking
git diff --check
```

Expected:

- all tests PASS;
- `rg` finds no active backtrack symbol;
- `git diff --check` exits 0.

- [ ] **Step 6: TI-compile every changed active source**

After defining the Exact TI Compile Helper, run:

```powershell
Invoke-TiCompile @(
  'application\app_scheduler.c',
  'application\corner_maneuver.c',
  'application\line_recovery.c',
  'modules\line_tracking\line_features.c',
  'modules\line_tracking\line_estimator.c',
  'modules\line_tracking\line_event_classifier.c',
  'modules\line_tracking\line_trend_detector.c',
  'modules\line_tracking\line_controller.c'
)
```

Expected: `compiled_units=8` with no diagnostics.

- [ ] **Step 7: Commit scheduler integration**

```powershell
git add -- `
  MSPM0G3507_LineFollowing_Car/application/app_scheduler.c `
  tests/test_app_scheduler.py `
  tests/test_line_estimator.py
git commit -m "feat: integrate mixed-corner control pipeline"
```

---

### Task 6: Rebuild, Compare, and Deliver UniFlash Firmware

**Files:**

- Generate: `MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.out`
- Generate: `MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.hex`
- Generate: `MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.txt`
- Replace untracked delivery copies in `firmware/`.

**Interfaces:**

- Consumes: all five committed implementation tasks.
- Produces: byte-equivalent Intel HEX and TI-TXT images for UniFlash.

- [ ] **Step 1: Verify a clean tracked implementation**

```powershell
python -m unittest discover -s tests -p 'test_*.py'
git diff --check
git status --short
git log -8 --oneline
```

Expected: all tests PASS; no tracked changes remain; only known untracked
snapshot, diagnostics, firmware, and local LED diagnostic files remain.

- [ ] **Step 2: Compile all active firmware units**

After defining the Exact TI Compile Helper, run:

```powershell
Invoke-TiCompile @(
  'application\app_main.c',
  'application\app_scheduler.c',
  'application\corner_maneuver.c',
  'application\line_recovery.c',
  'application\safety_supervisor.c',
  'bsp\bsp_line_mux.c',
  'bsp\delay.c',
  'bsp\time\timer.c',
  'empty.c',
  'modules\buzzer\buzzer.c',
  'modules\key\key.c',
  'modules\led\led.c',
  'modules\line_tracking\line_controller.c',
  'modules\line_tracking\line_estimator.c',
  'modules\line_tracking\line_event_classifier.c',
  'modules\line_tracking\line_features.c',
  'modules\line_tracking\line_scanner.c',
  'modules\line_tracking\line_trend_detector.c',
  'modules\motor\app_motor.c',
  'modules\motor\app_motor_usart.c',
  'modules\motor\bsp_motor_usart.c',
  'modules\motor\motor_adapter.c',
  'modules\motor\motor_safety.c'
)
```

Expected: `compiled_units=23`.

- [ ] **Step 3: Link the fresh OUT**

Run:

```powershell
$toolDir = 'D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin'
$libDir = 'D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\lib'
$build = (Resolve-Path 'MSPM0G3507_LineFollowing_Car\Build_LineFollowing').Path
$project = Split-Path $build -Parent
$objects = Get-ChildItem (Join-Path $build 'obj_minimal') -Filter '*.o' |
    Where-Object {
        $_.Name -notlike 'C__*' -and
        $_.Name -notlike 'D__*' -and
        $_.Name -ne 'application_motion_primitives.o'
    } |
    ForEach-Object { $_.FullName }
$required = @(
    'application_corner_maneuver.o',
    'modules_line_tracking_line_features.o',
    'modules_line_tracking_line_event_classifier.o'
)
foreach ($name in $required) {
    if (-not ($objects -match [regex]::Escape($name))) {
        throw ('Fresh object missing: ' + $name)
    }
}
$linkArgs = @(
    ('-I' + $libDir),
    '-o', (Join-Path $build 'MSPM0G3507_LineFollowing_Car.out'),
    ('-m' + (Join-Path $build 'MSPM0G3507_LineFollowing_Car.map')),
    '-iC:/ti/mspm0_sdk_2_10_00_04/source',
    ('-i' + $project),
    ('-i' + $build),
    ('-i' + $libDir),
    '--diag_wrap=off',
    '--display_error_number',
    '--warn_sections',
    ('--xml_link_info=' +
      (Join-Path $build 'MSPM0G3507_LineFollowing_Car_linkInfo.xml')),
    '--rom_model'
)
$linkArgs += $objects
$linkArgs += @(
    ('-l' + (Join-Path $build 'device_linker.cmd')),
    ('-l' + (Join-Path $build 'device.cmd.genlibs')),
    '-llibc.a',
    '--start-group',
    '-llibc++.a',
    '-llibc++abi.a',
    '-llibc.a',
    '-llibsys.a',
    '-llibsysbm.a',
    '-llibclang_rt.builtins.a',
    '-llibclang_rt.profile.a',
    '--end-group',
    '--cg_opt_level=2'
)
& (Join-Path $toolDir 'tiarmlnk.exe') @linkArgs
if ($LASTEXITCODE -ne 0) {
    throw 'Link failed'
}
```

Require exit code 0 and fresh map, XML link report, and OUT timestamps.

- [ ] **Step 4: Generate Intel HEX and TI-TXT**

```powershell
& 'D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmobjcopy.exe' `
  -O ihex `
  MSPM0G3507_LineFollowing_Car\Build_LineFollowing\MSPM0G3507_LineFollowing_Car.out `
  MSPM0G3507_LineFollowing_Car\Build_LineFollowing\MSPM0G3507_LineFollowing_Car.hex

Push-Location MSPM0G3507_LineFollowing_Car\Build_LineFollowing
& 'D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmhex.exe' `
  --ti_txt `
  --outfile=MSPM0G3507_LineFollowing_Car.txt `
  MSPM0G3507_LineFollowing_Car.out
Pop-Location
```

Require HEX to start with `:` and end with `:00000001FF`; TI-TXT must start
with an address record and end with `q`.

- [ ] **Step 5: Copy, hash, and byte-compare delivery images**

```powershell
Copy-Item `
  MSPM0G3507_LineFollowing_Car\Build_LineFollowing\MSPM0G3507_LineFollowing_Car.hex `
  firmware\MSPM0G3507_LineFollowing_Car.hex -Force
Copy-Item `
  MSPM0G3507_LineFollowing_Car\Build_LineFollowing\MSPM0G3507_LineFollowing_Car.txt `
  firmware\MSPM0G3507_LineFollowing_Car.txt -Force
Get-FileHash firmware\MSPM0G3507_LineFollowing_Car.hex,`
  firmware\MSPM0G3507_LineFollowing_Car.txt -Algorithm SHA256
```

Parse both files into address-to-byte dictionaries and require:

```text
address_sets_equal True
diff_count 0
```

- [ ] **Step 6: Perform the physical safety handoff**

Before placing the car on the track:

1. Raise both drive wheels.
2. Flash the new TI-TXT with UniFlash.
3. Confirm soft-start and physical left/right polarity.
4. Confirm every single-wheel direction reversal includes a stopped interval.
5. Present left and right L-shaped sensor patterns by hand.
6. Confirm no recovery step drives both wheels backward.
7. Low-speed test one left corner, then one right corner.
8. Test alternating left/right corners for three laps.
9. Restore the 400 straight target only after low-speed corner tests pass.

No Git commit is required for untracked firmware delivery copies. If any
source or parameter changes during physical testing, make that change in a
new focused task and commit.
