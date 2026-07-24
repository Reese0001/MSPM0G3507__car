# Corner Edge-Line Reacquisition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a sharp turn stop pivoting after three new reliable frames see black on any 1–3 channels, including the outer two channels, then let the existing 300 ms settle phase center the car.

**Architecture:** Keep the existing `CornerManeuver` state machine and public interfaces. Replace only the SEEK reacquisition predicate: reliable line presence owns SEEK exit, while the existing controller owns centering during SETTLE.

**Tech Stack:** C11, TI Arm Clang 4.0.4, host MSVC C harness, Python `unittest`.

## Global Constraints

- Work only in `D:\DevProject\MSPM0G3507__car\.worktrees\line-following-burn`.
- Keep motor commands within ±450 and never command both wheels negative.
- Preserve the global 120 ms actual-zero direction interlock, watchdog and fault latch.
- Do not change normal line PID, turn commands, timeouts or installed-hardware SysConfig.
- Duplicate sensor frames must not advance reacquisition evidence.
- Make one Git commit for this behavior change.

---

### Task 1: Accept Reliable Outer-Line Frames During Corner SEEK

**Files:**
- Modify: `tests/corner_maneuver_harness.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/corner_maneuver.c`

**Interfaces:**
- Consumes: `LineFeatures` passed to `CornerManeuver_Step(...)`.
- Produces: unchanged `CornerManeuver_Step(...)`; SEEK transitions to SETTLE after three unique frames with `active_count` 1–3 and `confidence >= 40`.

- [ ] **Step 1: Write the failing behavior test**

Extend the SEEK portion of `corner_maneuver_harness.c` so three unique right-edge frames use:

```c
set_features(&features, 230U, 3U, 2U);
features.centroid_error = 6.0f;
set_features(&features, 235U, 4U, 2U);
features.centroid_error = 6.0f;
set_features(&features, 240U, 5U, 2U);
features.centroid_error = 6.0f;
```

After the first two frames assert `CORNER_MANEUVER_SEEK`; after the third assert
`CORNER_MANEUVER_SETTLE`, `out.owns_motion`, `out.request.valid`, and
`check_not_reversing(&out)`. Repeat or mirror the sequence for `-6.0f` so both
turn directions are covered. Preserve the existing duplicate-sequence and timeout
assertions.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```powershell
python -m unittest tests.test_corner_maneuver -v
```

Expected: the host harness fails because `feature_is_centered()` rejects
`centroid_error` ±6 and SEEK does not enter SETTLE.

- [ ] **Step 3: Implement the minimal predicate change**

In `corner_maneuver.c`, rename the private predicate and remove only the centroid
condition:

```c
static bool feature_has_reliable_line(const LineFeatures *features)
{
    return features->active_count >= 1U &&
           features->active_count <= 3U &&
           features->confidence >= CORNER_MIN_CONFIDENCE;
}
```

Make `update_reacquisition()` call `feature_has_reliable_line(features)`. Do not
change counters, state timings, motor requests or public headers.

- [ ] **Step 4: Verify GREEN and regressions**

Run:

```powershell
python -m unittest tests.test_corner_maneuver -v
python -m unittest discover -s tests -p "test_*.py"
```

Expected: focused tests and the full suite pass with zero failures.

Compile the affected TI unit:

```powershell
$cc = "D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmclang.exe"
$project = (Resolve-Path "MSPM0G3507_LineFollowing_Car").Path
$build = Join-Path $project "Build_LineFollowing"
$sdk = "C:\ti\mspm0_sdk_2_10_00_04"
& $cc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O2 -Wall `
  -D__MSPM0G3507__ -D__USE_SYSCONFIG__ "-I$project" "-I$build" `
  "-I$project\application" "-I$project\modules\common" `
  "-I$project\modules\line_tracking" "-I$sdk\source" `
  "-I$sdk\source\third_party\CMSIS\Core\Include" `
  -c "$project\application\corner_maneuver.c" `
  -o "$build\obj_minimal\application_corner_maneuver.o"
```

Expected: exit code 0 with no diagnostics.

- [ ] **Step 5: Commit the behavior change**

```powershell
git add -- tests/corner_maneuver_harness.c `
  MSPM0G3507_LineFollowing_Car/application/corner_maneuver.c
git commit -m "fix: reacquire outer line after sharp turns"
```

- [ ] **Step 6: Regenerate burn artifacts**

Recompile all 23 active sources plus generated SysConfig and startup objects, link
the explicit 25-object manifest, then generate Intel HEX and TI-TXT. Parse both
files and require equal address sets and zero byte differences before copying them
to `firmware/`.
