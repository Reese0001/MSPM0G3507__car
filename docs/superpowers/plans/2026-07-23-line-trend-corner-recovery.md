# Line Trend Corner Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 使用八路灰度连续帧稳定识别普通弯、连续紧弯、发卡弯、连续方形 90°直角和全灭型直角，并在普通丢线时执行前进、停车、沿原圆弧倒退恢复。

**Architecture:** 新增只负责分类的 `line_trend_detector`，由调度器把灰度快照和估计值送入该模块。`line_controller` 负责仍能看到黑线的普通/紧弯/发卡闭环，`line_recovery` 只负责已确认直角与普通丢线恢复，最终命令仍经过现有安全监督和电机安全层。

**Tech Stack:** C11、TI DriverLib、TI Arm Clang 4.0.4、MSPM0 SDK 2.10.00.04、Python `unittest`、CCS Theia、UniFlash。

## Global Constraints

- 只在 `D:\DevProject\MSPM0G3507__car\.worktrees\line-following-burn` 工作，不修改 `main` 工作树。
- 每个任务先观察测试失败，再写最小实现；每个任务完成后单独 Git 提交。
- 电机命令绝对值不得超过 450；保留软启动、200 ms 看门狗、急停和请求过期保护。
- 任何正反切换前停车至少 120 ms。
- `KP` 保持 28，最高直线命令保持 350。
- 未安装 MPU6050、超声波和视觉模块时，对应 profile 开关继续为 0。
- C/H 文件保持仓库现有编码；新增设计、测试和计划文件使用 UTF-8。
- 最终同时生成完整单文件 Intel HEX 和 TI-TXT，优先用 TXT 进行 UniFlash 首次验证。

---

## File Structure

**Create**

- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.h`：趋势分类接口和结果类型。
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.c`：有限历史、方向趋势和多圈重置逻辑。
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_config.h`：所有趋势阈值和确认帧数。
- `tests/test_line_trend_detector.py`：趋势接口、左右镜像、噪声、方形直角和多圈契约。

**Modify**

- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.h`
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.c`
- `MSPM0G3507_LineFollowing_Car/application/config/line_control_config.h`
- `MSPM0G3507_LineFollowing_Car/application/line_recovery.h`
- `MSPM0G3507_LineFollowing_Car/application/line_recovery.c`
- `MSPM0G3507_LineFollowing_Car/application/config/line_recovery_config.h`
- `MSPM0G3507_LineFollowing_Car/application/app_scheduler.c`
- `MSPM0G3507_LineFollowing_Car/application/motion_primitives.h`
- `MSPM0G3507_LineFollowing_Car/application/motion_primitives.c`
- `tests/test_line_estimator.py`
- `tests/test_line_recovery.py`
- `tests/test_motion_primitives.py`

---

### Task 1: Add the Line Trend Detector

**Files:**

- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_config.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.c`
- Create: `tests/test_line_trend_detector.py`

**Interfaces:**

- Consumes: `LineEstimate`, `LineSensorSnapshot`, `uint32_t now_ms`.
- Produces:

```c
typedef enum {
    LINE_TREND_NORMAL = 0,
    LINE_TREND_TIGHT_LEFT,
    LINE_TREND_TIGHT_RIGHT,
    LINE_TREND_HAIRPIN_LEFT,
    LINE_TREND_HAIRPIN_RIGHT,
    LINE_TREND_RIGHT_ANGLE_LEFT,
    LINE_TREND_RIGHT_ANGLE_RIGHT
} LineTrendType;

typedef struct {
    ModuleStatus status;
    LineTrendType type;
    int8_t direction;
} LineTrendResult;

void LineTrendDetector_Init(void);
void LineTrendDetector_Reset(void);
bool LineTrendDetector_Update(const LineEstimate *estimate,
                              const LineSensorSnapshot *snapshot,
                              uint32_t now_ms,
                              LineTrendResult *result);
```

- Configuration:

```c
#define LINE_TREND_TIGHT_ERROR (3.0f)
#define LINE_TREND_OUTER_ERROR (4.0f)
#define LINE_TREND_HAIRPIN_ERROR (6.0f)
#define LINE_TREND_OUTWARD_STEPS (2U)
#define LINE_TREND_HAIRPIN_FRAMES (3U)
#define LINE_TREND_REACQUIRE_FRAMES (3U)
#define LINE_TREND_CORNER_WINDOW_MS (200U)
#define LINE_TREND_MIN_CONFIDENCE (40U)
#define LINE_TREND_CROSSLINE_ACTIVE_COUNT (4U)
```

- [ ] **Step 1: Write the failing trend contract tests**

Create `tests/test_line_trend_detector.py` with tests that assert:

```python
class LineTrendDetectorContract(unittest.TestCase):
    def test_public_types_and_thresholds_exist(self):
        header_path = ROOT / "modules/line_tracking/line_trend_detector.h"
        config_path = ROOT / "modules/line_tracking/line_trend_config.h"
        self.assertTrue(header_path.exists())
        self.assertTrue(config_path.exists())
        header = header_path.read_text(encoding="utf-8")
        config = config_path.read_text(encoding="utf-8")
        for token in (
            "LINE_TREND_NORMAL",
            "LINE_TREND_TIGHT_LEFT",
            "LINE_TREND_TIGHT_RIGHT",
            "LINE_TREND_HAIRPIN_LEFT",
            "LINE_TREND_HAIRPIN_RIGHT",
            "LINE_TREND_RIGHT_ANGLE_LEFT",
            "LINE_TREND_RIGHT_ANGLE_RIGHT",
            "LineTrendResult",
            "LineTrendDetector_Update",
            "LineTrendDetector_Reset",
        ):
            self.assertIn(token, header)
        for token in (
            "LINE_TREND_TIGHT_ERROR (3.0f)",
            "LINE_TREND_OUTER_ERROR (4.0f)",
            "LINE_TREND_HAIRPIN_ERROR (6.0f)",
            "LINE_TREND_CORNER_WINDOW_MS (200U)",
            "LINE_TREND_CROSSLINE_ACTIVE_COUNT (4U)",
        ):
            self.assertIn(token, config)

    def test_detector_tracks_sequence_not_only_last_sample(self):
        source = (ROOT / "modules/line_tracking/line_trend_detector.c").read_text(
            encoding="utf-8"
        )
        for token in (
            "outward_steps",
            "outer_seen_ms",
            "hairpin_frames",
            "count_active_bits",
            "LINE_EVENT_WIDE_BLACK",
            "LINE_EVENT_LOST",
        ):
            self.assertIn(token, source)
        self.assertNotIn("Contrl_Speed", source)
```

Add table-driven Python trace cases for these expected classifications:

```python
LEFT_TIGHT = ((-1, "none"), (-3, "none"), (-5, "none"))
RIGHT_HAIRPIN = ((1, "none"), (3, "none"), (5, "none"),
                 (7, "hard"), (7, "hard"), (7, "hard"))
LEFT_LOST_CORNER = ((-1, "none"), (-3, "none"), (-5, "hard"), (-5, "lost"))
RIGHT_WIDE_CORNER = ((1, "none"), (3, "none"), (5, "hard"), (0, "wide"))
NOISE_NOT_CORNER = ((0, "none"), (7, "hard"), (0, "none"), (0, "lost"))
```

The test reference classifier must require same-sign outward movement at least twice before accepting `lost` or `wide` as a right angle.

- [ ] **Step 2: Run the trend tests and observe failure**

Run:

```powershell
python -m unittest tests.test_line_trend_detector
```

Expected: FAIL because `line_trend_detector.h`, `.c`, and config do not exist.

- [ ] **Step 3: Implement the minimal detector**

Implement a fixed-size state machine with module-static fields:

```c
static int8_t trend_direction;
static uint8_t same_direction_frames;
static uint8_t outward_steps;
static uint8_t hairpin_frames;
static float previous_error;
static float maximum_absolute_error;
static uint32_t outer_seen_ms;
static bool outer_seen;
```

Required behavior:

- stale or invalid inputs return `false` and publish an invalid result;
- a sign reversal clears old outward and hairpin evidence before processing the new side;
- one equal-magnitude frame is allowed without clearing `outward_steps`;
- right-angle classification requires prior outward steps and an outer sample inside 200 ms;
- `WIDE_BLACK` or at least four active bits can complete a continuous square corner;
- `LOST` can complete an all-off right angle;
- `LineTrendDetector_Reset()` clears every static field.

- [ ] **Step 4: Run tests and TI-compile the new module**

Run:

```powershell
python -m unittest tests.test_line_trend_detector
python -m unittest discover -s tests
```

Expected: all tests PASS.

Compile `line_trend_detector.c` with the existing TI flags:

```powershell
& 'D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmclang.exe' `
  -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O2 -gdwarf-3 -Wall `
  '@MSPM0G3507_LineFollowing_Car\Build_LineFollowing\device.opt' `
  -IMSPM0G3507_LineFollowing_Car `
  -IMSPM0G3507_LineFollowing_Car\Build_LineFollowing `
  -IC:\ti\mspm0_sdk_2_10_00_04\source `
  -IC:\ti\mspm0_sdk_2_10_00_04\source\third_party\CMSIS\Core\Include `
  -IMSPM0G3507_LineFollowing_Car\modules\common `
  -IMSPM0G3507_LineFollowing_Car\modules\line_tracking `
  -c MSPM0G3507_LineFollowing_Car\modules\line_tracking\line_trend_detector.c `
  -o MSPM0G3507_LineFollowing_Car\Build_LineFollowing\obj_minimal\modules_line_tracking_line_trend_detector.o
```

Expected: exit code 0 with no compiler diagnostics.

- [ ] **Step 5: Commit the detector**

```powershell
git add -- MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_config.h MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.h MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.c tests/test_line_trend_detector.py
git commit -m "feat: classify line curvature from sensor trends"
```

---

### Task 2: Add Continuous Tight-Curve and Hairpin Control

**Files:**

- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/config/line_control_config.h`
- Modify: `tests/test_line_estimator.py`

**Interfaces:**

- Consumes the Task 1 `LineTrendResult`.
- Changes:

```c
bool LineController_Step(const LineEstimate *estimate,
                         const LineTrendResult *trend,
                         float yaw_rate_dps,
                         bool yaw_fresh,
                         uint32_t now_ms,
                         LineControlOutput *output);
```

- Adds `tight_forward`, `tight_turn`, `hairpin_forward`, and `hairpin_turn` to `LineControlConfig`.
- Initial values:

```c
#define LINE_TIGHT_FORWARD (140)
#define LINE_TIGHT_TURN (100)
#define LINE_HAIRPIN_FORWARD (40)
#define LINE_HAIRPIN_TURN (100)
```

These produce initial left/right targets `40/240` for a left tight curve and `-60/140` for a left hairpin before the existing turn slew is applied.

- [ ] **Step 1: Write failing controller contract tests**

Extend `tests/test_line_estimator.py` to require the new signature, config values, and explicit handling of:

```c
LINE_TREND_TIGHT_LEFT
LINE_TREND_TIGHT_RIGHT
LINE_TREND_HAIRPIN_LEFT
LINE_TREND_HAIRPIN_RIGHT
```

Assert that normal control still uses `LINE_CONTROL_KP (28.0f)`, `LINE_MAX_FORWARD (350)`, and `slew_turn`, and that tight-curve code does not call motor drivers directly.

- [ ] **Step 2: Run the focused test and observe failure**

```powershell
python -m unittest tests.test_line_estimator.LineControllerContract
```

Expected: FAIL because the controller does not accept or handle `LineTrendResult`.

- [ ] **Step 3: Implement trend-aware targets**

Add helpers:

```c
static bool trend_is_left(LineTrendType type);
static bool trend_is_right(LineTrendType type);
static int16_t trend_forward_target(LineTrendType type);
static int16_t trend_turn_target(LineTrendType type);
```

Behavior:

- normal trends retain the existing KP/KD calculation;
- tight trends select forward 140 and signed turn target 100;
- hairpin trends select forward 40 and signed turn target 100;
- right-angle trends do not create a new controller mode because recovery owns them;
- `slew_turn()` remains active during entry and exit;
- invalid/stale trend data falls back to normal estimate control, not to a motor stop;
- lost line still makes controller output invalid and resets forward/turn history.

- [ ] **Step 4: Run tests and TI-compile controller**

```powershell
python -m unittest tests.test_line_estimator
python -m unittest discover -s tests
```

Expected: all tests PASS.

Compile:

```powershell
& 'D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmclang.exe' `
  -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O2 -gdwarf-3 -Wall `
  '@MSPM0G3507_LineFollowing_Car\Build_LineFollowing\device.opt' `
  -IMSPM0G3507_LineFollowing_Car `
  -IMSPM0G3507_LineFollowing_Car\Build_LineFollowing `
  -IC:\ti\mspm0_sdk_2_10_00_04\source `
  -IC:\ti\mspm0_sdk_2_10_00_04\source\third_party\CMSIS\Core\Include `
  -IMSPM0G3507_LineFollowing_Car\modules\common `
  -IMSPM0G3507_LineFollowing_Car\modules\line_tracking `
  -c MSPM0G3507_LineFollowing_Car\modules\line_tracking\line_controller.c `
  -o MSPM0G3507_LineFollowing_Car\Build_LineFollowing\obj_minimal\modules_line_tracking_line_controller.o
```

Expected: exit code 0.

- [ ] **Step 5: Commit continuous-curve control**

```powershell
git add -- MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.h MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.c MSPM0G3507_LineFollowing_Car/application/config/line_control_config.h tests/test_line_estimator.py
git commit -m "feat: control tight curves and hairpins"
```

---

### Task 3: Replace Last-Sample Recovery with Trend-Aware Recovery

**Files:**

- Modify: `MSPM0G3507_LineFollowing_Car/application/line_recovery.h`
- Modify: `MSPM0G3507_LineFollowing_Car/application/line_recovery.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/config/line_recovery_config.h`
- Modify: `tests/test_line_recovery.py`

**Interfaces:**

- Consumes `LineTrendResult` from Task 1.
- Changes:

```c
bool LineRecovery_Step(const LineEstimate *line,
                       const LineTrendResult *trend,
                       const LineControlOutput *follow,
                       float yaw_deg,
                       bool yaw_fresh,
                       bool emergency_stop,
                       uint32_t now_ms,
                       MotionRequest *request);
```

- Recovery states:

```c
typedef enum {
    LINE_RECOVERY_FOLLOW = 0,
    LINE_RECOVERY_LOSS_CONFIRM,
    LINE_RECOVERY_CORNER_PIVOT,
    LINE_RECOVERY_FORWARD_SEARCH,
    LINE_RECOVERY_REVERSAL_PAUSE,
    LINE_RECOVERY_BACKTRACK,
    LINE_RECOVERY_ALIGN,
    LINE_RECOVERY_FAULT
} LineRecoveryState;
```

- Timings and commands:

```c
#define LINE_LOSS_CONFIRM_COUNT (3U)
#define LINE_REACQUIRE_COUNT (3U)
#define LINE_FORWARD_SEARCH_MS (500U)
#define LINE_REVERSAL_PAUSE_MS (120U)
#define LINE_BACKTRACK_MS (700U)
#define LINE_ALIGN_DURATION_MS (300U)
#define LINE_RECOVERY_TOTAL_TIMEOUT_MS (3000U)
#define LINE_SEARCH_INNER_COMMAND (80)
#define LINE_SEARCH_OUTER_COMMAND (120)
#define LINE_CORNER_INNER_COMMAND (-80)
#define LINE_CORNER_OUTER_COMMAND (120)
```

- [ ] **Step 1: Write failing recovery sequence tests**

Replace last-sample assertions in `tests/test_line_recovery.py` with contracts requiring:

- right-angle trend immediately selects `LINE_RECOVERY_CORNER_PIVOT`;
- ordinary loss selects forward search, then reversal pause, then backtrack;
- pause commands are exactly zero and remain valid so the motor watchdog is refreshed;
- left backtrack outputs `-80/-120`; right backtrack outputs `-120/-80`;
- corner pivot outputs `-80/120` or `120/-80`;
- reacquisition requires three fresh frames and `confidence >= 40`;
- total timeout and emergency stop enter fault with invalid zero request;
- `last_seen_error` and `LINE_SHARP_SEARCH_ERROR` are absent after migration.

- [ ] **Step 2: Run focused tests and observe failure**

```powershell
python -m unittest tests.test_line_recovery
```

Expected: FAIL on missing new states, trend input, timings, and backtrack commands.

- [ ] **Step 3: Implement the recovery state machine**

Use module state:

```c
static LineRecoveryState recovery_state;
static int8_t recovery_direction;
static uint8_t loss_count;
static uint8_t reacquire_count;
static uint16_t last_line_sequence;
static uint32_t recovery_started_ms;
static uint32_t state_started_ms;
```

Transition rules:

- trend right angle from FOLLOW enters CORNER_PIVOT immediately;
- ordinary LOST needs three new loss frames before FORWARD_SEARCH;
- FORWARD_SEARCH lasts 500 ms;
- REVERSAL_PAUSE outputs `0/0`, valid, for 120 ms;
- BACKTRACK uses exact negated arc commands for 700 ms;
- trustworthy center line is `event != LOST`, `confidence >= 40`, and `absolute(error) <= 3`;
- three consecutive trustworthy frames enter ALIGN;
- ALIGN uses controller output for 300 ms, then returns FOLLOW;
- any stale estimate, emergency stop, or total 3000 ms timeout enters FAULT;
- `LineRecovery_Reset()` clears direction, counters, sequence, and both timers.

- [ ] **Step 4: Run tests and TI-compile recovery**

```powershell
python -m unittest tests.test_line_recovery
python -m unittest discover -s tests
```

Expected: all tests PASS.

Compile `application\line_recovery.c` with the Task 1 TI command plus:

```text
-IMSPM0G3507_LineFollowing_Car\application
source: application\line_recovery.c
output: Build_LineFollowing\obj_minimal\application_line_recovery.o
```

Expected: exit code 0.

- [ ] **Step 5: Commit trend-aware recovery**

```powershell
git add -- MSPM0G3507_LineFollowing_Car/application/line_recovery.h MSPM0G3507_LineFollowing_Car/application/line_recovery.c MSPM0G3507_LineFollowing_Car/application/config/line_recovery_config.h tests/test_line_recovery.py
git commit -m "feat: recover line with timed arc backtracking"
```

---

### Task 4: Integrate Trend State into Scheduler and Motion Primitives

**Files:**

- Modify: `MSPM0G3507_LineFollowing_Car/application/app_scheduler.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/motion_primitives.h`
- Modify: `MSPM0G3507_LineFollowing_Car/application/motion_primitives.c`
- Modify: `tests/test_motion_primitives.py`
- Modify: `tests/test_line_trend_detector.py`

**Interfaces:**

- Scheduler owns:

```c
static LineTrendResult line_trend;
```

- `MotionContext` adds:

```c
LineTrendResult line_trend;
```

- Data order every 5 ms:

```text
LineScanner snapshot
  -> LineEstimator_Update
  -> LineTrendDetector_Update
  -> LineController_Step
  -> LineRecovery_Step
  -> SafetySupervisor_Step (1 ms task consumes latest request)
```

- [ ] **Step 1: Write failing integration tests**

Extend tests to assert:

- scheduler includes `line_trend_detector.h`;
- `LineTrendDetector_Update` appears before `LineController_Step`;
- the same `line_trend` object is passed to controller and recovery;
- scheduler init calls `LineTrendDetector_Init`;
- start/reset and recovery fault call `LineTrendDetector_Reset`;
- `MotionContext` contains `LineTrendResult line_trend`;
- motion primitives pass injected trend to controller and recovery;
- no legacy `LineWalking`, blocking delay, or direct motor command is introduced.

- [ ] **Step 2: Run focused tests and observe failure**

```powershell
python -m unittest tests.test_motion_primitives tests.test_line_trend_detector
```

Expected: FAIL because scheduler and motion context do not expose trend state.

- [ ] **Step 3: Implement integration**

In the scheduler:

- preserve the scanner snapshot long enough to call the detector;
- publish an invalid motion request if estimate or trend update fails;
- initialize and reset trend state at every existing controller/recovery reset boundary;
- do not change the 5 ms line task or 1 ms safety task periods.

In motion primitives:

- use only `context->line_trend`; do not create hidden global trend state;
- update controller/recovery calls to their Task 2/3 signatures;
- keep other five primitives behavior unchanged.

- [ ] **Step 4: Run all tests and TI-compile integration units**

```powershell
python -m unittest discover -s tests
```

Expected: all tests PASS.

TI-compile both integration units:

```powershell
$compiler = 'D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmclang.exe'
$project = (Resolve-Path 'MSPM0G3507_LineFollowing_Car').Path
$build = Join-Path $project 'Build_LineFollowing'
$sdk = 'C:\ti\mspm0_sdk_2_10_00_04'
$includes = @(
    $project, $build, (Join-Path $sdk 'source'),
    (Join-Path $sdk 'source\third_party\CMSIS\Core\Include'),
    (Join-Path $project 'application'),
    (Join-Path $project 'modules\common'),
    (Join-Path $project 'modules\motor'),
    (Join-Path $project 'modules\line_tracking'),
    (Join-Path $project 'modules\led'),
    (Join-Path $project 'modules\buzzer'),
    (Join-Path $project 'bsp'),
    (Join-Path $project 'bsp\time')
)
$compileArgs = @(
    '-mcpu=cortex-m0plus', '-mthumb', '-mfloat-abi=soft',
    '-O2', '-gdwarf-3', '-Wall', ('@' + (Join-Path $build 'device.opt'))
)
foreach ($include in $includes) { $compileArgs += '-I' + $include }
$jobs = @(
    @('application\app_scheduler.c', 'application_app_scheduler.o'),
    @('application\motion_primitives.c', 'application_motion_primitives.o')
)
foreach ($job in $jobs) {
    & $compiler @compileArgs '-c' (Join-Path $project $job[0]) `
      '-o' (Join-Path $build ('obj_minimal\' + $job[1]))
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
```

Expected: both compilation commands exit 0.

- [ ] **Step 5: Commit scheduler integration**

```powershell
git add -- MSPM0G3507_LineFollowing_Car/application/app_scheduler.c MSPM0G3507_LineFollowing_Car/application/motion_primitives.h MSPM0G3507_LineFollowing_Car/application/motion_primitives.c tests/test_motion_primitives.py tests/test_line_trend_detector.py
git commit -m "feat: integrate line trends into motion scheduling"
```

---

### Task 5: Add Multi-Lap, Mirror, Noise, and Safety Scenario Coverage

**Files:**

- Modify: `tests/test_line_trend_detector.py`
- Modify: `tests/test_line_recovery.py`
- Modify: `tests/test_line_estimator.py`

**Interfaces:**

- Uses only public thresholds and state names from Tasks 1–4.
- Produces a regression suite for both photographed track types.

- [ ] **Step 1: Add failing scenario contracts**

Add explicit trace tables for:

```python
SQUARE_LEFT = (-1, -3, -5, "wide", -5, -3, -1)
SQUARE_RIGHT = (1, 3, 5, "wide", 5, 3, 1)
HAIRPIN_LEFT = (-1, -3, -5, -7, -7, -7, -5, -3, -1)
HAIRPIN_RIGHT = (1, 3, 5, 7, 7, 7, 5, 3, 1)
S_CURVE = (-1, -3, -1, 1, 3, 1)
ORDINARY_GAP = (0, 0, "lost", "lost", "lost")
```

Require:

- square traces classify as right angle;
- hairpin traces classify as hairpin and never LOST recovery;
- S curve clears the old direction before building the opposite direction;
- ordinary gap never classifies as a corner;
- running `SQUARE_LEFT`, reset, then `SQUARE_LEFT` again produces identical result;
- all configured motor commands are inside ±450;
- reversal pause is at least 120 ms.

- [ ] **Step 2: Run scenario tests and observe failure**

```powershell
python -m unittest tests.test_line_trend_detector tests.test_line_recovery tests.test_line_estimator
```

Expected: at least one new scenario assertion fails until reset/exit boundaries exactly match the specification.

- [ ] **Step 3: Make only boundary corrections**

Permitted corrections in this task:

- clear trend state after three-frame stable reacquisition;
- clear direction on a genuine sign reversal;
- require prior outward trend before accepting wide/lost corner completion;
- preserve turn slew during tight/hairpin exit;
- keep all numeric values unchanged unless a test reveals an internal contradiction.

Do not tune real-car speed or KP in this task.

- [ ] **Step 4: Run the complete regression suite**

```powershell
python -m unittest discover -s tests
git diff --check
```

Expected: all tests PASS and `git diff --check` exits 0.

- [ ] **Step 5: Commit scenario coverage**

```powershell
git add -- tests/test_line_trend_detector.py tests/test_line_recovery.py tests/test_line_estimator.py MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_trend_detector.c MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.c MSPM0G3507_LineFollowing_Car/application/line_recovery.c
git commit -m "test: cover repeated corners and recovery traces"
```

Stage only files that actually changed.

---

### Task 6: Build, Generate UniFlash Images, and Verify Handoff

**Files:**

- Generated: `MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.out`
- Generated: `MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.hex`
- Generated: `MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.txt`
- Copy generated artifacts to untracked `firmware/`.

**Interfaces:**

- Consumes all object files from Tasks 1–4.
- Produces one complete Intel HEX and one equivalent TI-TXT.

- [ ] **Step 1: Run fresh verification before linking**

```powershell
python -m unittest discover -s tests
git diff --check
git status --short
```

Expected: all tests PASS; only intended tracked changes and pre-existing untracked artifacts are present.

- [ ] **Step 2: Link the firmware**

Link every current minimal object, including the new trend detector:

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
if (-not ($objects -match 'modules_line_tracking_line_trend_detector.o')) {
    throw 'line trend detector object missing'
}
$linkArgs = @(
    '-I' + $libDir,
    '-o', (Join-Path $build 'MSPM0G3507_LineFollowing_Car.out'),
    '-m' + (Join-Path $build 'MSPM0G3507_LineFollowing_Car.map'),
    '-iC:/ti/mspm0_sdk_2_10_00_04/source',
    '-i' + $project,
    '-i' + $build,
    '-i' + $libDir,
    '--diag_wrap=off',
    '--display_error_number',
    '--warn_sections',
    '--xml_link_info=' + (Join-Path $build 'MSPM0G3507_LineFollowing_Car_linkInfo.xml'),
    '--rom_model'
) + $objects + @(
    '-l' + (Join-Path $build 'device_linker.cmd'),
    '-l' + (Join-Path $build 'device.cmd.genlibs'),
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
exit $LASTEXITCODE
```

Expected: exit code 0 and a fresh `.out` timestamp.

- [ ] **Step 3: Generate complete HEX and TI-TXT**

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

Expected:

- HEX starts with `:` and ends with `:00000001FF`;
- TXT starts with `@0000` and ends with `q`;
- parsed HEX and TXT address/value maps are identical.

- [ ] **Step 4: Copy artifacts and verify hashes**

```powershell
Copy-Item MSPM0G3507_LineFollowing_Car\Build_LineFollowing\MSPM0G3507_LineFollowing_Car.hex firmware\MSPM0G3507_LineFollowing_Car.hex -Force
Copy-Item MSPM0G3507_LineFollowing_Car\Build_LineFollowing\MSPM0G3507_LineFollowing_Car.txt firmware\MSPM0G3507_LineFollowing_Car.txt -Force
Get-FileHash firmware\MSPM0G3507_LineFollowing_Car.hex,firmware\MSPM0G3507_LineFollowing_Car.txt -Algorithm SHA256
```

Compare every programmed byte:

```powershell
@'
from pathlib import Path

def parse_ihex(path):
    memory, upper = {}, 0
    for line in Path(path).read_text().splitlines():
        raw = bytes.fromhex(line[1:])
        size = raw[0]
        address = (raw[1] << 8) | raw[2]
        record_type = raw[3]
        if record_type == 0:
            for offset, value in enumerate(raw[4:4 + size]):
                memory[upper + address + offset] = value
        elif record_type == 4:
            upper = int.from_bytes(raw[4:6], 'big') << 16
        elif record_type == 2:
            upper = int.from_bytes(raw[4:6], 'big') << 4
    return memory

def parse_ti_txt(path):
    memory, address = {}, None
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        if line == 'q':
            break
        if line.startswith('@'):
            address = int(line[1:], 16)
            continue
        for token in line.split():
            memory[address] = int(token, 16)
            address += 1
    return memory

hex_memory = parse_ihex('firmware/MSPM0G3507_LineFollowing_Car.hex')
txt_memory = parse_ti_txt('firmware/MSPM0G3507_LineFollowing_Car.txt')
differences = [
    address for address in sorted(set(hex_memory) | set(txt_memory))
    if hex_memory.get(address) != txt_memory.get(address)
]
print('address_sets_equal', set(hex_memory) == set(txt_memory))
print('diff_count', len(differences))
raise SystemExit(0 if set(hex_memory) == set(txt_memory) and not differences else 1)
'@ | python -
```

Require:

```text
address_sets_equal True
diff_count 0
```

- [ ] **Step 5: Perform staged physical verification**

Checklist:

1. wheels raised; flash TI-TXT; Verify succeeds;
2. D2 heartbeat flashes and D1 fault LED stays off;
3. left/right tight and reverse directions match commands;
4. reversal includes a visible stop;
5. test continuous square at low speed;
6. test continuous hairpins at low speed;
7. test ordinary gap and arc backtracking;
8. run two complete laps and record the first failing segment if behavior differs.

Do not change more than one tuning constant per follow-up commit.

- [ ] **Step 6: Final repository and artifact verification**

```powershell
python -m unittest discover -s tests
git log -6 --oneline
git status --short
```

Expected: all tests PASS; tracked work is committed; only known untracked diagnostics, firmware, snapshot, and local diagnostic test remain.
