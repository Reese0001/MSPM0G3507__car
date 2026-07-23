# Straight Boost and Early Corner Braking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Raise average lap speed with a five-frame confidence-gated straight boost while braking earlier and running slower through curves to prevent outside-line overshoot.

**Architecture:** Extend only the modular `line_controller` configuration and state. The controller counts consecutive fresh, centered, high-confidence normal-trend frames, selects 400 only after five frames, otherwise cruises at 330, and immediately resets the boost counter when a curve or uncertain frame appears. Existing recovery, scheduler periods, safety supervision, motor protocol, and hardware profile remain unchanged.

**Tech Stack:** C11, TI DriverLib, TI Arm Clang 4.0.4, MSPM0 SDK 2.10.00.04, Python `unittest`, CCS Theia, UniFlash.

## Global Constraints

- Work only in `D:\DevProject\MSPM0G3507__car\.worktrees\line-following-burn`; do not modify the main worktree.
- Keep `LINE_CONTROL_KP` at `28.0f`.
- Keep every wheel command inside `+/-450`.
- Preserve motor soft-start, the 100 ms watchdog, 50 ms request expiry, emergency stop, and the 120 ms direction-change pause.
- Keep MPU6050, ultrasonic, and vision profile switches disabled.
- Do not modify lost-line backtracking, right-angle recovery, motor UART protocol, pin assignments, or scheduler periods.
- Write a failing test before production changes, then run focused tests, all tests, TI compilation, and a fresh link.
- Commit the completed controller behavior separately before generating firmware artifacts.

---

## File Structure

**Modify**

- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.h` — add straight-boost configuration fields.
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.c` — own the saturating straight-frame counter and target selection.
- `MSPM0G3507_LineFollowing_Car/application/config/line_control_config.h` — expose all tuning parameters in one editable location.
- `tests/test_line_estimator.py` — enforce parameter, reset, fallback, symmetry, and command-limit contracts.

**Generate**

- `MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.out`
- `MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.hex`
- `MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.txt`
- `firmware/MSPM0G3507_LineFollowing_Car.hex`
- `firmware/MSPM0G3507_LineFollowing_Car.txt`

---

### Task 1: Add Confidence-Gated Straight Boost and Early Curve Braking

**Files:**

- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/config/line_control_config.h`
- Test: `tests/test_line_estimator.py`

**Interfaces:**

- Extend `LineControlConfig` with:

```c
int16_t cruise_forward;
float straight_error_threshold;
uint8_t straight_confirm_frames;
```

- Keep the public step signature unchanged:

```c
bool LineController_Step(const LineEstimate *estimate,
                         const LineTrendResult *trend,
                         float yaw_rate_dps,
                         bool yaw_fresh,
                         uint32_t now_ms,
                         LineControlOutput *output);
```

- Add configuration constants:

```c
#define LINE_MAX_FORWARD (400)
#define LINE_CRUISE_FORWARD (330)
#define LINE_STRAIGHT_ERROR_THRESHOLD (1.0f)
#define LINE_STRAIGHT_CONFIRM_FRAMES (5U)
#define LINE_CURVE_FORWARD (240)
#define LINE_HARD_CURVE_FORWARD (150)
#define LINE_TIGHT_FORWARD (120)
#define LINE_HAIRPIN_FORWARD (40)
#define LINE_DECEL_STEP (70)
#define LINE_TURN_SLEW_STEP (25)
```

- [ ] **Step 1: Write the failing straight-boost contract tests**

Add these tests to `LineControllerContract` in `tests/test_line_estimator.py`:

```python
def test_straight_boost_and_early_braking_parameters(self):
    header = (ROOT / "modules/line_tracking/line_controller.h").read_text(
        encoding="utf-8"
    )
    source = (ROOT / "modules/line_tracking/line_controller.c").read_text(
        encoding="utf-8"
    )
    config = (ROOT / "application/config/line_control_config.h").read_text(
        encoding="utf-8"
    )
    for token in (
        "LINE_MAX_FORWARD (400)",
        "LINE_CRUISE_FORWARD (330)",
        "LINE_STRAIGHT_ERROR_THRESHOLD (1.0f)",
        "LINE_STRAIGHT_CONFIRM_FRAMES (5U)",
        "LINE_CURVE_ERROR_THRESHOLD (1.5f)",
        "LINE_HARD_CURVE_ERROR_THRESHOLD (3.5f)",
        "LINE_CURVE_FORWARD (240)",
        "LINE_HARD_CURVE_FORWARD (150)",
        "LINE_TIGHT_FORWARD (120)",
        "LINE_HAIRPIN_FORWARD (40)",
        "LINE_DECEL_STEP (70)",
        "LINE_TURN_SLEW_STEP (25)",
        "LINE_CONTROL_KP (28.0f)",
    ):
        self.assertIn(token, config)
    for token in (
        "cruise_forward",
        "straight_error_threshold",
        "straight_confirm_frames",
    ):
        self.assertIn(token, header)
    for token in (
        "stable_straight_frames",
        "stable_straight_frame",
        "LINE_TREND_NORMAL",
        "LINE_EVENT_NONE",
        "UINT8_MAX",
    ):
        self.assertIn(token, source)

def test_straight_boost_resets_on_uncertain_or_curve_frame(self):
    source = (ROOT / "modules/line_tracking/line_controller.c").read_text(
        encoding="utf-8"
    )
    self.assertRegex(
        source,
        r"if\s*\(stable_straight_frame[\s\S]{0,250}"
        r"stable_straight_frames\+\+[\s\S]{0,180}"
        r"else\s*\{\s*stable_straight_frames\s*=\s*0U;",
    )
    self.assertRegex(
        source,
        r"LineController_Reset[\s\S]{0,180}"
        r"stable_straight_frames\s*=\s*0U",
    )
    self.assertRegex(
        source,
        r"estimate->event\s*==\s*LINE_EVENT_LOST[\s\S]{0,180}"
        r"stable_straight_frames\s*=\s*0U",
    )
```

Add a pure reference selector near the existing test helpers:

```python
def straight_target(frames):
    stable = 0
    targets = []
    for error, confidence, event, trend in frames:
        qualifies = (
            abs(error) <= 1.0
            and confidence >= 70
            and event == "none"
            and trend == "normal"
        )
        stable = min(255, stable + 1) if qualifies else 0
        targets.append(400 if stable >= 5 else 330)
    return targets
```

Add behavior tests:

```python
def test_reference_boost_requires_five_frames(self):
    frames = [(0.0, 100, "none", "normal")] * 5
    self.assertEqual(straight_target(frames), [330, 330, 330, 330, 400])

def test_reference_curve_frame_cancels_boost_immediately(self):
    frames = (
        [(0.0, 100, "none", "normal")] * 5
        + [(1.6, 100, "none", "normal")]
    )
    self.assertEqual(straight_target(frames)[-2:], [400, 330])
```

Update the existing parameter assertions in the same test class so they expect
the new values rather than the superseded `350`, `270`, `180`, `45`, and `20`
constants. Keep the existing assertions for KP 28, command clamping,
motor independence, and turn slew usage.

- [ ] **Step 2: Run the focused tests and observe the expected failure**

Run:

```powershell
python -m unittest tests.test_line_estimator.LineControllerContract
```

Expected: FAIL because the new constants, configuration fields, and
`stable_straight_frames` logic do not exist.

- [ ] **Step 3: Add configuration fields and values**

In `line_controller.h`, insert `cruise_forward` after `max_forward`, and add
`straight_error_threshold` plus `straight_confirm_frames` near the other
thresholds:

```c
typedef struct {
    int16_t max_forward;
    int16_t cruise_forward;
    int16_t curve_forward;
    /* existing command fields remain in their current order */
    float steering_polarity;
    float straight_error_threshold;
    float curve_error_threshold;
    float hard_curve_error_threshold;
    float high_yaw_rate_dps;
    uint8_t low_confidence;
    uint8_t medium_confidence;
    uint8_t straight_confirm_frames;
    uint32_t estimate_stale_ms;
} LineControlConfig;
```

In `line_control_config.h`, use:

```c
#define LINE_MAX_FORWARD (400)
#define LINE_CRUISE_FORWARD (330)
#define LINE_CURVE_FORWARD (240)
#define LINE_HARD_CURVE_FORWARD (150)
#define LINE_TIGHT_FORWARD (120)
#define LINE_HAIRPIN_FORWARD (40)
#define LINE_DECEL_STEP (70)
#define LINE_TURN_SLEW_STEP (25)
#define LINE_STRAIGHT_ERROR_THRESHOLD (1.0f)
#define LINE_STRAIGHT_CONFIRM_FRAMES (5U)
#define LINE_CURVE_ERROR_THRESHOLD (1.5f)
#define LINE_HARD_CURVE_ERROR_THRESHOLD (3.5f)
```

Insert the new values into `LineControlConfig_Default()` in exactly the same
order as the struct fields.

- [ ] **Step 4: Implement the saturating straight-frame counter**

Add module state and helper logic to `line_controller.c`:

```c
static uint8_t stable_straight_frames = 0U;

static bool stable_straight_frame(const LineEstimate *estimate,
                                  const LineTrendResult *trend,
                                  bool trend_fresh)
{
    return trend_fresh &&
           trend->type == LINE_TREND_NORMAL &&
           estimate->event == LINE_EVENT_NONE &&
           estimate->confidence >= control_config.medium_confidence &&
           absolute_value(estimate->predicted_error) <=
               control_config.straight_error_threshold;
}

static bool update_straight_boost(const LineEstimate *estimate,
                                  const LineTrendResult *trend,
                                  bool trend_fresh)
{
    if (stable_straight_frame(estimate, trend, trend_fresh)) {
        if (stable_straight_frames < UINT8_MAX) {
            stable_straight_frames++;
        }
    } else {
        stable_straight_frames = 0U;
    }
    return stable_straight_frames >= control_config.straight_confirm_frames;
}
```

Change the speed planner signature and initial target:

```c
static int16_t plan_target_speed(const LineEstimate *estimate,
                                 float yaw_rate_dps,
                                 bool yaw_fresh,
                                 bool straight_boost)
{
    float curve = absolute_value(estimate->predicted_error);
    int16_t target = straight_boost ?
        control_config.max_forward : control_config.cruise_forward;
    /* retain the existing curve, confidence, yaw, hard-turn, and wide rules */
}
```

In `LineController_Step()`, compute `trend_fresh`, then:

```c
straight_boost = update_straight_boost(estimate, trend, trend_fresh);
```

Pass `straight_boost` to `plan_target_speed()`. Tight and hairpin trends keep
their dedicated targets and still use `slew_turn()`.

Clear `stable_straight_frames` in:

```c
LineController_Init()
LineController_Reset()
```

Also clear it in the invalid/lost-estimate branch of `LineController_Step()`.

Extend configuration validation:

```c
settings->cruise_forward <= 0 ||
settings->cruise_forward > settings->max_forward ||
settings->straight_error_threshold <= 0.0f ||
settings->straight_error_threshold >= settings->curve_error_threshold ||
settings->straight_confirm_frames == 0U
```

- [ ] **Step 5: Run focused and complete regression tests**

Run:

```powershell
python -m unittest tests.test_line_estimator
python -m unittest discover -s tests -p 'test_*.py'
git diff --check
```

Expected: all tests PASS and `git diff --check` exits 0.

- [ ] **Step 6: TI-compile the controller**

Run:

```powershell
$compiler = 'D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmclang.exe'
$project = (Resolve-Path 'MSPM0G3507_LineFollowing_Car').Path
$build = Join-Path $project 'Build_LineFollowing'
$sdk = 'C:\ti\mspm0_sdk_2_10_00_04'
$args = @(
    '-mcpu=cortex-m0plus', '-mthumb', '-mfloat-abi=soft',
    '-O2', '-gdwarf-3', '-Wall',
    ('@' + (Join-Path $build 'device.opt')),
    ('-I' + $project),
    ('-I' + $build),
    ('-I' + (Join-Path $sdk 'source')),
    ('-I' + (Join-Path $sdk 'source\third_party\CMSIS\Core\Include')),
    ('-I' + (Join-Path $project 'application')),
    ('-I' + (Join-Path $project 'modules\common')),
    ('-I' + (Join-Path $project 'modules\line_tracking')),
    '-c',
    (Join-Path $project 'modules\line_tracking\line_controller.c'),
    '-o',
    (Join-Path $build 'obj_minimal\modules_line_tracking_line_controller.o')
)
& $compiler @args
exit $LASTEXITCODE
```

Expected: exit code 0 with no compiler diagnostics.

- [ ] **Step 7: Commit the controller behavior**

```powershell
git add -- `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.h `
  MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.c `
  MSPM0G3507_LineFollowing_Car/application/config/line_control_config.h `
  tests/test_line_estimator.py
git commit -m "tune: boost straights and brake earlier for curves"
```

---

### Task 2: Rebuild and Verify UniFlash Firmware

**Files:**

- Generate: `MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.out`
- Generate: `MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.hex`
- Generate: `MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.txt`
- Replace generated delivery copies in untracked `firmware/`.

**Interfaces:**

- Input: committed controller source from Task 1.
- Output: equivalent complete Intel HEX and TI-TXT images for UniFlash.

- [ ] **Step 1: Run fresh pre-build verification**

```powershell
python -m unittest discover -s tests -p 'test_*.py'
git diff --check
git status --short
```

Expected: all tests pass; tracked controller work is committed; only the known
untracked snapshot, diagnostics, firmware directory, and local LED diagnostic
test remain.

- [ ] **Step 2: Recompile all active firmware units**

Use the same TI Arm Clang flags and include paths as the current successful
minimal build. Compile these 20 sources to their matching
`Build_LineFollowing\obj_minimal\*.o` names:

```text
application/app_main.c
application/app_scheduler.c
application/line_recovery.c
application/safety_supervisor.c
bsp/bsp_line_mux.c
bsp/delay.c
bsp/time/timer.c
empty.c
modules/buzzer/buzzer.c
modules/key/key.c
modules/led/led.c
modules/line_tracking/line_controller.c
modules/line_tracking/line_estimator.c
modules/line_tracking/line_scanner.c
modules/line_tracking/line_trend_detector.c
modules/motor/app_motor.c
modules/motor/app_motor_usart.c
modules/motor/bsp_motor_usart.c
modules/motor/motor_adapter.c
modules/motor/motor_safety.c
```

Run:

```powershell
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
$jobs = @(
    @('application\app_main.c', 'application_app_main.o'),
    @('application\app_scheduler.c', 'application_app_scheduler.o'),
    @('application\line_recovery.c', 'application_line_recovery.o'),
    @('application\safety_supervisor.c', 'application_safety_supervisor.o'),
    @('bsp\bsp_line_mux.c', 'bsp_bsp_line_mux.o'),
    @('bsp\delay.c', 'bsp_delay.o'),
    @('bsp\time\timer.c', 'bsp_time_timer.o'),
    @('empty.c', 'empty.o'),
    @('modules\buzzer\buzzer.c', 'modules_buzzer_buzzer.o'),
    @('modules\key\key.c', 'modules_key_key.o'),
    @('modules\led\led.c', 'modules_led_led.o'),
    @('modules\line_tracking\line_controller.c',
      'modules_line_tracking_line_controller.o'),
    @('modules\line_tracking\line_estimator.c',
      'modules_line_tracking_line_estimator.o'),
    @('modules\line_tracking\line_scanner.c',
      'modules_line_tracking_line_scanner.o'),
    @('modules\line_tracking\line_trend_detector.c',
      'modules_line_tracking_line_trend_detector.o'),
    @('modules\motor\app_motor.c', 'modules_motor_app_motor.o'),
    @('modules\motor\app_motor_usart.c', 'modules_motor_app_motor_usart.o'),
    @('modules\motor\bsp_motor_usart.c', 'modules_motor_bsp_motor_usart.o'),
    @('modules\motor\motor_adapter.c', 'modules_motor_motor_adapter.o'),
    @('modules\motor\motor_safety.c', 'modules_motor_motor_safety.o')
)
foreach ($job in $jobs) {
    & $compiler @base `
      '-c' (Join-Path $project $job[0]) `
      '-o' (Join-Path $build ('obj_minimal\' + $job[1]))
    if ($LASTEXITCODE -ne 0) {
        throw ('Compile failed: ' + $job[0])
    }
}
Write-Output ('compiled_units=' + $jobs.Count)
```

Expected: 20 successful compilation exits and no diagnostics.

- [ ] **Step 3: Link the fresh OUT**

Collect `obj_minimal\*.o`, excluding names matching `C__*`, `D__*`, and
`application_motion_primitives.o`. Link with:

```text
device_linker.cmd
device.cmd.genlibs
libc.a
libc++.a
libc++abi.a
libsys.a
libsysbm.a
libclang_rt.builtins.a
libclang_rt.profile.a
```

Run `tiarmlnk.exe` with `--rom_model`, the current library paths, and the
existing map/XML outputs. Expected: exit code 0 and a fresh
`MSPM0G3507_LineFollowing_Car.out`.

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
if (-not ($objects -match 'modules_line_tracking_line_controller.o')) {
    throw 'fresh line controller object missing'
}
$linkArgs = @()
$linkArgs += '-I' + $libDir
$linkArgs += '-o'
$linkArgs += Join-Path $build 'MSPM0G3507_LineFollowing_Car.out'
$linkArgs += '-m' + (Join-Path $build 'MSPM0G3507_LineFollowing_Car.map')
$linkArgs += '-iC:/ti/mspm0_sdk_2_10_00_04/source'
$linkArgs += '-i' + $project
$linkArgs += '-i' + $build
$linkArgs += '-i' + $libDir
$linkArgs += '--diag_wrap=off'
$linkArgs += '--display_error_number'
$linkArgs += '--warn_sections'
$linkArgs += '--xml_link_info=' +
    (Join-Path $build 'MSPM0G3507_LineFollowing_Car_linkInfo.xml')
$linkArgs += '--rom_model'
$linkArgs += $objects
$linkArgs += '-l' + (Join-Path $build 'device_linker.cmd')
$linkArgs += '-l' + (Join-Path $build 'device.cmd.genlibs')
$linkArgs += '-llibc.a'
$linkArgs += '--start-group'
$linkArgs += '-llibc++.a'
$linkArgs += '-llibc++abi.a'
$linkArgs += '-llibc.a'
$linkArgs += '-llibsys.a'
$linkArgs += '-llibsysbm.a'
$linkArgs += '-llibclang_rt.builtins.a'
$linkArgs += '-llibclang_rt.profile.a'
$linkArgs += '--end-group'
$linkArgs += '--cg_opt_level=2'
& (Join-Path $toolDir 'tiarmlnk.exe') @linkArgs
exit $LASTEXITCODE
```

- [ ] **Step 4: Generate HEX and TI-TXT**

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

Expected: HEX begins with `:` and ends with `:00000001FF`; TI-TXT begins with
`@0000` and ends with `q`.

- [ ] **Step 5: Copy and compare every programmed byte**

```powershell
Copy-Item `
  MSPM0G3507_LineFollowing_Car\Build_LineFollowing\MSPM0G3507_LineFollowing_Car.hex `
  firmware\MSPM0G3507_LineFollowing_Car.hex -Force
Copy-Item `
  MSPM0G3507_LineFollowing_Car\Build_LineFollowing\MSPM0G3507_LineFollowing_Car.txt `
  firmware\MSPM0G3507_LineFollowing_Car.txt -Force
Get-FileHash `
  firmware\MSPM0G3507_LineFollowing_Car.hex,`
  firmware\MSPM0G3507_LineFollowing_Car.txt `
  -Algorithm SHA256
```

Parse both formats into address/value maps and require:

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
raise SystemExit(
    0 if set(hex_memory) == set(txt_memory) and not differences else 1
)
'@ | python -
```

Require:

```text
address_sets_equal True
diff_count 0
```

- [ ] **Step 6: Final verification and physical test handoff**

```powershell
python -m unittest discover -s tests -p 'test_*.py'
git log -5 --oneline
git status --short
```

Physical test order:

1. Raise the drive wheels and flash the TI-TXT with UniFlash.
2. Verify D1 does not latch a fault and D2 heartbeat remains active.
3. Confirm both wheels accelerate smoothly and commands have the correct sign.
4. Test the photographed continuous curve at low battery-safe speed.
5. Test the square right angle.
6. Run two full laps and compare the same outside-line corner on both laps.
7. If tuning is needed, change only one constant in a new commit.
