# H Question 2 Four-Channel Line Following Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the retired eight-channel gray sensor with the confirmed four-channel infrared module and complete H-question-2 start, one-lap timing, and A-line stopping.

**Architecture:** A direct GPIO scanner reads PA24--PA27 simultaneously and publishes a four-bit logical left-to-right frame. The existing decoder, follower, lap state machine, dashboard, and safe Drive authority consume it; only the scanner owns GPIO reads.

**Tech Stack:** TI MSPM0 DriverLib/SysConfig, TI Arm Clang, C11 MSVC harnesses, Python unittest.

## Global Constraints

- X1 -> PA24, X2 -> PA25, X3 -> PA26, X4 -> PA27; installed header order facing forward is GND/X4/X3/X2/X1/VCC.
- PA15--PA18 and the old address-mux scanner must not participate at runtime.
- Preserve Drive's soft-start, 5 ms send interval, 200 ms watchdog, direction interlock, and immediate stop path.
- Initial motor test: raised drive wheels and accessible 12.6 V disconnect.
- Preserve existing source-file encoding.

---

### Task 1: Establish four-channel decode and lap-marker semantics

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_tracking_config.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/decoder/line_position.[ch]`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/lap_tracker.[ch]`
- Modify: `tests/line_position_harness.c`, `tests/lap_tracker_harness.c`, `tests/test_line_estimator.py`, `tests/test_clean_line_runtime.py`

**Interfaces:**
- Consumes: bit 0 = left outer, bit 1 = left inner, bit 2 = right inner, bit 3 = right outer.
- Produces: `LinePosition_Update(uint8_t)` position -3..+3 and `LapTracker_Update(uint8_t,uint32_t)` returning true once after departure and return.

- [ ] **Step 1: Write failing host tests**

Add assertions for `0x01=-3`, `0x02=-1`, `0x04=+1`, `0x08=+3`, `0x03=-2`, `0x0C=+2`, and `0x0F=LINE_PATTERN_WIDE`. Extend the lap harness: start on `0x0F`, feed three `0x06` frames, wait >=3000 ms, feed three `0x0F` frames, assert one true result and frozen elapsed time.

- [ ] **Step 2: Verify the tests fail**

Run: `python -m unittest tests.test_clean_line_runtime.CleanLineRuntime.test_lap_timer_ignores_start_marker_then_stops_on_return tests.test_line_estimator -v`

Expected: FAIL because the eight-channel decoder and six-bit marker threshold remain.

- [ ] **Step 3: Implement the smallest four-bit decoder**

Use exactly this logical table and weight vector in `line_position.c`:

```c
static const int8_t position_by_bits[16] = {
    [0x01] = -3, [0x03] = -2, [0x02] = -1,
    [0x06] = 0, [0x04] = 1, [0x0C] = 2, [0x08] = 3
};
static const int8_t sensor_position[4] = {-3, -1, 1, 3};
```

Limit classification loops to four bits: zero = lost, one contiguous run of one/two = position, contiguous three/four = wide, separated runs = noise. Set `LAP_MARKER_MIN_BITS (4U)`; preserve three-frame leave/return debounce and minimum time. Define `FOUR_LINE_BLACK_ACTIVE_LEVEL (1U)`, `FOUR_LINE_STALE_MS (40U)`, and `FOUR_LINE_SAMPLE_PERIOD_MS (2U)`.

- [ ] **Step 4: Verify pass and commit**

Run the Step 2 command; expected PASS.

```powershell
git add -- MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_tracking_config.h MSPM0G3507_LineFollowing_Car/modules/line_tracking/decoder/line_position.c MSPM0G3507_LineFollowing_Car/modules/line_tracking/decoder/line_position.h MSPM0G3507_LineFollowing_Car/modules/line_tracking/lap_tracker.c MSPM0G3507_LineFollowing_Car/modules/line_tracking/lap_tracker.h tests/line_position_harness.c tests/lap_tracker_harness.c tests/test_line_estimator.py tests/test_clean_line_runtime.py
git commit -m "feat: decode four-channel line and lap marker"
```

### Task 2: Add direct PA24--PA27 scanner and SysConfig ownership

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/scanner/four_line_scanner.[ch]`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/scanner/line_scanner.[ch]`, `empty.syscfg`, `Makefile`
- Modify: `tests/line_scanner_timebase_harness.c`, `tests/test_line_scanner_timebase.py`, `tests/test_sysconfig_contract.py`

**Interfaces:**
- Produces: `void FourLineScanner_Sample(uint32_t now_ms)` and `bool FourLineScanner_GetSnapshot(LineSensorSnapshot *out)`.
- Compatibility: existing `LineScanner_*` functions delegate to the new scanner so the app pipeline remains unchanged.

- [ ] **Step 1: Write failing contracts**

Require `four_line_scanner.c` to contain `DL_GPIO_PIN_24` through `DL_GPIO_PIN_27`, exactly one `DL_GPIO_readPins(GPIOA`, and no `BSP_LineMux_`. Require SysConfig GPIO inputs `LINE_X1`--`LINE_X4` on PA24--PA27 and no `GRAY_ADDR`/ `GRAY_DATA`.

- [ ] **Step 2: Verify failure**

Run: `python -m unittest tests.test_line_scanner_timebase tests.test_sysconfig_contract -v`

Expected: FAIL because the old mux owns PA15--PA18.

- [ ] **Step 3: Implement direct sampling**

Use one GPIO read and preserve the confirmed wiring while publishing left-to-right logical bits:

```c
uint8_t raw = ((levels & DL_GPIO_PIN_27) != 0U ? 0x01U : 0U) |
              ((levels & DL_GPIO_PIN_26) != 0U ? 0x02U : 0U) |
              ((levels & DL_GPIO_PIN_25) != 0U ? 0x04U : 0U) |
              ((levels & DL_GPIO_PIN_24) != 0U ? 0x08U : 0U);
return FOUR_LINE_BLACK_ACTIVE_LEVEL != 0U ? raw : (uint8_t)(raw ^ 0x0FU);
```

Publish status health `MODULE_HEALTH_OK`, timestamp `now_ms`, and increment sequence once per sample. Remove old SysConfig GPIOs, add named PA24--PA27 inputs, and replace `line_mux.c` in Makefile `SOURCES` with `four_line_scanner.c`.

- [ ] **Step 4: Verify pass and commit**

Run: `python -m unittest tests.test_line_scanner_timebase tests.test_sysconfig_contract -v`

Run: `& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C MSPM0G3507_LineFollowing_Car syscfg`

Expected: both commands exit 0.

```powershell
git add -- MSPM0G3507_LineFollowing_Car/modules/line_tracking/scanner/four_line_scanner.c MSPM0G3507_LineFollowing_Car/modules/line_tracking/scanner/four_line_scanner.h MSPM0G3507_LineFollowing_Car/modules/line_tracking/scanner/line_scanner.c MSPM0G3507_LineFollowing_Car/modules/line_tracking/scanner/line_scanner.h MSPM0G3507_LineFollowing_Car/empty.syscfg MSPM0G3507_LineFollowing_Car/Makefile tests/line_scanner_timebase_harness.c tests/test_line_scanner_timebase.py tests/test_sysconfig_contract.py
git commit -m "feat: sample four infrared line sensors directly"
```

### Task 3: Integrate following, frozen time display, and stop request

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_follower.[ch]`
- Modify: `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/display/dashboard.[ch]`
- Modify: `tests/line_follower_clean_harness.c`, `tests/test_clean_line_runtime.py`

**Interfaces:**
- Consumes: four-bit `LinePositionResult`, fresh IMU data, K1, and `LapTracker_Update`.
- Produces: fresh `MotionRequest`; after lap completion one invalid zero request reaches `Drive_SetTarget`, and dashboard displays frozen `STOP` time and X1--X4 raw state.

- [ ] **Step 1: Write failing behavior tests**

Assert a left-outer line (`0x01`) produces `left_speed > right_speed`, a right-outer line (`0x08`) produces `right_speed > left_speed`, and dashboard source has labels `X1`, `X2`, `X3`, `X4`, `WAIT K1 SAFE`, `RUN `, and `STOP `.

- [ ] **Step 2: Verify failure**

Run: `python -m unittest tests.test_clean_line_runtime -v`

Expected: FAIL because eight-channel tuning and current dashboard lack four-channel preflight labels.

- [ ] **Step 3: Implement minimal integration**

Replace the follower table with:

```c
static const LineTableEntry line_table[4] = {
    {70, 55}, {105, 26}, {105, 26}, {70, 55}
};
```

Make negative position create left-wheel-greater correction and positive create right-wheel-greater correction; retain clamp, slew, IMU freshness, noise hold, reacquire, and low-speed seek. In `app_tasks.c`, use `FOUR_LINE_STALE_MS`; retain pipeline order scanner -> decode -> lap -> follower -> Drive. On completion send `(MotionRequest){0, 0, now_ms, false}`, set `lap_finished`, and only reset can arm again. Add `raw_x_bits` to diagnostics and render X1/X2 then X3/X4 before start while preserving elapsed run/stop time.

- [ ] **Step 4: Verify complete software build and commit**

Run: `python -m unittest tests.test_clean_line_runtime tests.test_line_estimator tests.test_line_scanner_timebase -v`

Run: `& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C MSPM0G3507_LineFollowing_Car rebuild`

Expected: all tests pass and firmware images are emitted under `dist/firmware/`.

```powershell
git add -- MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_follower.c MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_follower.h MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c MSPM0G3507_LineFollowing_Car/modules/display/dashboard.c MSPM0G3507_LineFollowing_Car/modules/display/dashboard.h tests/line_follower_clean_harness.c tests/test_clean_line_runtime.py
git commit -m "feat: run and stop H question 2 with four sensors"
```

### Task 4: Complete physical safety and track acceptance

**Files:**
- Modify: `docs/verification/sensor-platform-test-record.md`

- [ ] **Step 1: Record USB-only polarity test**

Add:

```markdown
| Surface | OLED X1/X2/X3/X4 | Expected Bxx | Result |
|---|---|---|---|
| white | | B00 | |
| left outer black | | B01 | |
| right outer black | | B08 | |
| transverse A line | | B0F | |
```

- [ ] **Step 2: Record raised-wheel safety pass/fail**

Verify reset zero, gradual K1 start, stop zero, sensor fault safe behavior, and timeout stop under the existing 200 ms watchdog.

- [ ] **Step 3: Record three clockwise track trials**

For each: start at A, record total time and ruler-measured stop offset. Accept only runs <=20.0 s and absolute offset <=2.0 cm. Tune only the Task-3 table and marker confirmation count; never relax Drive limits.

- [ ] **Step 4: Commit evidence**

```powershell
git add -- docs/verification/sensor-platform-test-record.md
git commit -m "docs: record H question 2 verification"
```

## Plan Self-Review

- Coverage: PA24--PA27 sampling is Task 2; polarity/mapping and full-black marker testing are Tasks 2 and 4; start, one-lap return, frozen time, <=20 s, and <=2 cm are Tasks 1, 3, and 4; safety is a global constraint and Task 4.
- Placeholder scan: every code task has a failing test, exact implementation details, verification command, and focused commit.
- Interface consistency: Tasks use existing `LineSensorSnapshot`, `LinePositionResult`, `MotionRequest`, and `LapTracker_*`; Task 2 defines scanner behavior before Task 3 relies on its unchanged facade.

