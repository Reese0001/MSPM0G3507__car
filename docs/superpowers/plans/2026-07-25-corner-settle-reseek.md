# Corner Settle Reseek Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent a brief line loss after corner reacquisition from becoming a permanent D1 fault.

**Architecture:** Keep the corner state machine and public interfaces unchanged. When `SETTLE` loses a valid follow command, return to `SEEK`, clear only the reacquisition counter, and continue the original pivot under the existing 2-second total timeout.

**Tech Stack:** C11, Python `unittest`, Microsoft C host harness, TI Arm Clang 4.0.4 LTS.

## Global Constraints

- Work only in `D:\DevProject\MSPM0G3507__car\.worktrees\line-following-burn`.
- Do not modify speed, KP, steering direction, motor safety, SysConfig, or hardware configuration.
- Preserve `CORNER_TOTAL_TIMEOUT_MS (2000U)`.
- Do not add a new state, public function, or dependency.
- Commit this behavior change separately.

---

### Task 1: Return unstable corner settlement to the original seek direction

**Files:**
- Modify: `tests/corner_maneuver_harness.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/corner_maneuver.c`

**Interfaces:**
- Consumes: existing `CornerManeuver_Step()` inputs and `LineControlOutput.valid`.
- Produces: unchanged `CornerManeuver_Step()` and `CornerManeuverState`.

- [ ] **Step 1: Add the failing runtime sequence**

Add `run_settle_loss_reseek()` to the corner harness. Drive a left corner through
`BRAKE`, `COMMIT`, `SEEK`, and three fresh reliable frames into `SETTLE`.
Then pass a locked left-corner event with:

```c
LineControlOutput invalid_follow = {0, 0, false};
```

Assert:

```c
CHECK(CornerManeuver_Step(&features, &event, &invalid_follow,
                          false, 250U, &out));
CHECK(CornerManeuver_GetState() == CORNER_MANEUVER_SEEK);
CHECK(!out.fault && out.owns_motion && out.request.valid);
CHECK(out.request.left_speed == -80 && out.request.right_speed == 120);
```

Then provide three new reliable frames and assert the state re-enters
`CORNER_MANEUVER_SETTLE`. Call this function from `main()`.

- [ ] **Step 2: Verify RED**

Run:

```powershell
python -m unittest tests.test_corner_maneuver -v
```

Expected: FAIL because current `SETTLE` calls `enter_fault()` when
`follow.valid == false`.

- [ ] **Step 3: Implement the minimum state transition**

In the `CORNER_MANEUVER_SETTLE` branch, replace the existing explicit
`LINE_PATH_LOST` abort and the later invalid-follow fault with one early guard:

```c
if (path_event->type == LINE_PATH_LOST ||
    follow == 0 || !follow->valid) {
    reacquire_frames = 0U;
    enter_state(CORNER_MANEUVER_SEEK, now_ms);
    set_pivot(now_ms, &out->request);
    return true;
}
```

Keep the normal 300 ms settlement and completion path unchanged.

- [ ] **Step 4: Verify GREEN and regression coverage**

Run:

```powershell
python -m unittest tests.test_corner_maneuver -v
python -m unittest discover -s tests -p "test_*.py"
```

Expected: focused runtime harness passes; full suite reports zero failures.

- [ ] **Step 5: Compile, link, and generate UniFlash files**

Recompile all active sources and startup code with TI Arm Clang, link
`Build_LineFollowing/MSPM0G3507_LineFollowing_Car.out`, then regenerate:

```text
firmware/MSPM0G3507_LineFollowing_Car.hex
firmware/MSPM0G3507_LineFollowing_Car.txt
```

Expected: TI compilation and linking exit 0; Intel HEX and TI-TXT contain equal
address/data bytes.

- [ ] **Step 6: Commit**

```powershell
git add -- tests/corner_maneuver_harness.c `
  MSPM0G3507_LineFollowing_Car/application/corner_maneuver.c
git commit -m "fix: reseek line after unstable corner settle"
```
