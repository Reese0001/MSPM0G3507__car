# Official Line Baseline + YbImu Damping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to execute this plan task-by-task.

**Goal:** Make the eight-channel grayscale sensor the only line-following authority, and use only the YbImu Z-axis angular rate to damp steering jitter without ever stopping the car when IMU data is unavailable.

**Architecture:** Add one small table-driven baseline controller between the existing line decoder and recovery/motor safety layers. `line_motion.c` selects either the new baseline or the preserved assisted controller at compile time. The YbImu nonblocking driver supplies only `gyro_rad_s[2]`; stale data is converted to zero correction.

**Tech Stack:** MSPM0G3507 bare metal, TI DriverLib/TI Arm Clang, cooperative scheduler, Python `unittest` contract tests, host C harnesses.

## Global Constraints

- Keep `MotorAdapter_Apply` and `Motor_Safety_RequestSpeed` as the only motor-output path.
- Preserve K1 start gate, watchdog, 0→30% soft start, non-reversing lost-line search, and 0..140 output limits.
- Do not use magnetic heading, quaternion, absolute yaw, FOC, RTOS, blocking delays, or a new framework.
- Keep the existing assisted controller available as a compile-time fallback.
- Preserve unrelated working-tree changes.

---

### Task 1: Lock the active hardware and mode contracts

**Files:**
- Modify: `tests/test_ybimu_contract.py`
- Modify: `tests/test_sysconfig_contract.py`
- Create: `tests/test_official_baseline_profile.py`
- Modify: `MSPM0G3507_LineFollowing_Car/config/line_following_profile.h`
- Modify: `MSPM0G3507_LineFollowing_Car/empty.syscfg`
- Modify: `MSPM0G3507_LineFollowing_Car/bsp/bsp_i2c.c`
- Modify: `MSPM0G3507_LineFollowing_Car/.cproject`
- Modify: `MSPM0G3507_LineFollowing_Car/Makefile`

- [ ] Write contract tests requiring PA1=SCL, PA0=SDA, YbImu address/register constants, YbImu build sources, and official baseline as the default mode.
- [ ] Run the focused tests and confirm they fail because the active project still names/excludes the old MPU path.
- [ ] Make the smallest configuration/build edits needed to pass; retain the legacy MPU source path only for fallback mode.
- [ ] Re-run the focused tests and commit the hardware/profile checkpoint.

### Task 2: Implement the table-driven baseline controller test-first

**Files:**
- Create: `tests/line_official_control_harness.c`
- Create: `tests/test_line_official_control.py`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_official_control.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_official_control.c`

- [ ] Write host tests for centered line, left/right correction, Z-rate damping, deadband, ±24 correction clamp, stale-IMU bypass, 20 ms noise hold, locked lost direction, and three-frame reacquisition.
- [ ] Confirm RED because the controller does not exist.
- [ ] Implement a scalar-input API: decoded line position + Z angular rate + freshness → bounded forward/turn command and recovery event.
- [ ] Reuse the existing lookup table and recovery event types; do not depend on the YbImu driver type inside the controller.
- [ ] Confirm GREEN and commit the controller checkpoint.

### Task 3: Integrate the controller and nonblocking YbImu service

**Files:**
- Modify: `tests/test_app_scheduler.py`
- Modify: `MSPM0G3507_LineFollowing_Car/app/line/line_motion.c`
- Modify: `MSPM0G3507_LineFollowing_Car/app/line/line_motion.h`
- Modify: `MSPM0G3507_LineFollowing_Car/app/sensor/sensor_runtime.c`

- [ ] Add failing tests requiring explicit compile-time dispatch, per-sensor-cycle YbImu service, Z-only rate consumption, and stale-data bypass.
- [ ] Add two small private paths in `line_motion.c`: preserved assisted mode and official baseline mode.
- [ ] In baseline mode, initialize/service YbImu nonblockingly, convert `gyro_rad_s[2]` to deg/s, and pass freshness separately to the controller.
- [ ] Do not pass YbImu data into lost-line recovery and never suppress a grayscale motor request because IMU is stale.
- [ ] Run scheduler/controller tests and commit the runtime checkpoint.

### Task 4: Put useful smoothing diagnostics on OLED

**Files:**
- Modify: `tests/test_runtime_observer_contract.py`
- Modify: `MSPM0G3507_LineFollowing_Car/app/log/runtime_observer.c`
- Modify if needed: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_official_control.h`

- [ ] Add failing tests that reject absolute-yaw/magnetic diagnostics in baseline mode and require raw line bits, position/direction, Z rate, damping-used flag, and left/right command output.
- [ ] Expose one read-only diagnostics snapshot from the baseline controller.
- [ ] Render compact rolling OLED lines at the existing rate limit; display stale IMU as bypass rather than a fatal/stopped state.
- [ ] Run observer tests and commit the diagnostics checkpoint.

### Task 5: Update operator docs and produce a verified UniFlash image

**Files:**
- Modify: `README.md`
- Modify: `docs/setup/SETUP_GUIDE.md`
- Modify: `docs/hardware/ybimu-calibration-checklist.md`
- Generated: `dist/firmware/MSPM0G3507_LineFollowing_Car.txt`

- [ ] Document PA1/PA0 wiring, baseline/fallback switch, OLED fields, and the rule that IMU failure only disables damping.
- [ ] Run all Python/host tests.
- [ ] Clean old build products before rebuilding; verify the `.out`, `.map`, `.hex`, and `.txt` timestamps and symbols.
- [ ] Confirm the UniFlash artifact path and hash, then commit only task-related files.

