# MPU6050 Line Fusion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a nonblocking MPU6050 yaw-rate input that improves line-following stability and corner control while degrading safely to speed-limited line-only control.

**Architecture:** The existing software-I2C state machine owns PA12/PA13 and the new `modules/mpu6050` state machine performs bounded initialization, 100 Hz sampling, startup bias calibration, filtering, and snapshot publication. The eight-channel line pipeline remains authoritative; yaw rate only adds a bounded inner rate loop and reduces aggressive corner commands when rotation is already fast.

**Tech Stack:** MSPM0G3507 DriverLib, TI Arm Clang, bare-metal cooperative scheduler, C11-compatible host harnesses, Python `unittest`.

## Global Constraints

- Work directly on `main` as explicitly approved by the user.
- Complete one functional task per Git commit and push immediately after each commit.
- VCC is 3.3 V; SCL is PA12; SDA is PA13; AD0 is GND; address is `0x68`.
- Sensor `+Y` points forward and `+Z` points upward; positive Z yaw is a left turn.
- Hold motor output stopped during the two-second startup calibration.
- An IMU fault must degrade to speed-limited line-only control, not latch a control fault.
- Do not add DMP/eMPL, an RTOS, dynamic allocation, blocking delays, or new dependencies.
- Preserve motor soft-start, direction interlock, command ceilings, watchdog, and UART timeout.
- Keep physical tuning values in configuration headers.

---

### Task 1: Enable the nonblocking MPU6050 I2C bus

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/empty.syscfg`
- Modify: `MSPM0G3507_LineFollowing_Car/bsp/bsp_i2c.c`
- Modify: `MSPM0G3507_LineFollowing_Car/.cproject`
- Modify: `tests/test_ybimu_contract.py`
- Create: `tests/test_mpu6050_contract.py`

**Interfaces:**
- Consumes: existing `BSP_I2C_Init`, `BSP_I2C_BeginRead`, `BSP_I2C_BeginWrite`, `BSP_I2C_Service`, and `BSP_I2C_GetStatus`.
- Produces: the same BSP API bound to generated `MPU6050_I2C_*` GPIO macros on PA12/PA13.

- [ ] **Step 1: Write the failing SysConfig and BSP contract**

Add assertions to `tests/test_mpu6050_contract.py`:

```python
def test_sysconfig_assigns_mpu6050_bus(self):
    syscfg = (ROOT / "empty.syscfg").read_text(encoding="utf-8")
    self.assertIn('GPIO5.$name', syscfg)
    self.assertIn('"MPU6050_I2C"', syscfg)
    self.assertIn('"PA12"', syscfg)
    self.assertIn('"PA13"', syscfg)

def test_nonblocking_bus_uses_mpu6050_gpio(self):
    source = (ROOT / "bsp/bsp_i2c.c").read_text(encoding="utf-8")
    self.assertIn("MPU6050_I2C_SCL_PIN", source)
    self.assertIn("MPU6050_I2C_SDA_PIN", source)
    self.assertNotIn("YBIMU_I2C_", source)
    self.assertNotIn("delay_us", source)
    self.assertNotRegex(source, r"while\s*\(")
```

Update the old YBIMU contract so it validates the generic BSP API and
open-drain operations without requiring `YBIMU_I2C_*` names.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```powershell
python -m unittest tests.test_mpu6050_contract -v
```

Expected: FAIL because `GPIO5`, `MPU6050_I2C_*`, PA12, and PA13 are absent.

- [ ] **Step 3: Add the two SysConfig GPIO pins and enable the BSP source**

Add a fifth GPIO instance named `MPU6050_I2C`, with output-capable pins named
`SCL` on PA12 and `SDA` on PA13. Replace only the hard-coded YBIMU GPIO macro
prefix in `bsp_i2c.c`; retain the existing release-input/drive-low state
machine. Remove `bsp/bsp_i2c.c` from `.cproject` source exclusions.

- [ ] **Step 4: Run focused and existing contracts**

Run:

```powershell
python -m unittest tests.test_mpu6050_contract tests.test_ybimu_contract tests.test_sysconfig_contract -v
```

Expected: PASS.

- [ ] **Step 5: Commit and push**

```powershell
git add MSPM0G3507_LineFollowing_Car/empty.syscfg MSPM0G3507_LineFollowing_Car/bsp/bsp_i2c.c MSPM0G3507_LineFollowing_Car/.cproject tests/test_mpu6050_contract.py tests/test_ybimu_contract.py
git commit -m "feat: enable MPU6050 software I2C"
git push origin main
```

### Task 2: Add MPU6050 initialization, calibration, and filtered snapshots

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/modules/mpu6050/mpu6050.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/mpu6050/mpu6050.c`
- Create: `MSPM0G3507_LineFollowing_Car/modules/mpu6050/mpu6050_config.h`
- Modify: `MSPM0G3507_LineFollowing_Car/.cproject`
- Create: `tests/mpu6050_harness.c`
- Create: `tests/test_mpu6050.py`
- Modify: `tests/test_mpu6050_contract.py`

**Interfaces:**
- Consumes: nonblocking BSP I2C API and `ModuleStatus`.
- Produces:

```c
typedef enum {
    MPU6050_STATE_STARTUP = 0,
    MPU6050_STATE_CALIBRATING,
    MPU6050_STATE_READY,
    MPU6050_STATE_DEGRADED
} Mpu6050State;

typedef struct {
    ModuleStatus status;
    float yaw_rate_dps;
} Mpu6050Snapshot;

void Mpu6050_Init(uint32_t now_ms);
void Mpu6050_Service(uint32_t now_ms);
Mpu6050State Mpu6050_GetState(void);
bool Mpu6050_GetSnapshot(Mpu6050Snapshot *out);
```

- [ ] **Step 1: Write a fake-I2C host harness**

`tests/mpu6050_harness.c` supplies fake implementations of the BSP I2C API,
records requested registers, returns `WHO_AM_I=0x68`, feeds 200 stationary
gyro-Z samples, then feeds positive and negative motion samples. Assertions
must verify:

```c
assert(Mpu6050_GetState() == MPU6050_STATE_CALIBRATING);
assert(Mpu6050_GetState() == MPU6050_STATE_READY);
assert(snapshot.status.valid);
assert(snapshot.yaw_rate_dps > 0.0f);
assert(snapshot.yaw_rate_dps < previous_positive_rate);
assert(snapshot.yaw_rate_dps < 0.0f);
```

The harness also forces repeated I2C errors and verifies
`MPU6050_STATE_DEGRADED`.

- [ ] **Step 2: Run the harness wrapper and verify RED**

Run:

```powershell
python -m unittest tests.test_mpu6050 -v
```

Expected: FAIL because `modules/mpu6050/mpu6050.c` and its interface do not
exist.

- [ ] **Step 3: Implement the smallest nonblocking sensor state machine**

Use `WHO_AM_I`, `PWR_MGMT_1`, `CONFIG`, `GYRO_CONFIG`, `SMPLRT_DIV`, and
`GYRO_ZOUT_H` only. Service at 100 Hz, decode big-endian signed gyro Z,
calibrate for 2000 ms with at least 150 samples, apply configurable yaw sign,
range clamp, deadband, and first-order low-pass, then publish a timestamped
snapshot. Three consecutive transfer failures enter degraded mode; a later
valid sample may recover the module.

Configuration constants:

```c
#define MPU6050_I2C_ADDRESS (0x68U)
#define MPU6050_SAMPLE_PERIOD_MS (10U)
#define MPU6050_CALIBRATION_MS (2000U)
#define MPU6050_MIN_CALIBRATION_SAMPLES (150U)
#define MPU6050_GYRO_LSB_PER_DPS (65.5f)
#define MPU6050_YAW_SIGN (1.0f)
#define MPU6050_FILTER_ALPHA (0.25f)
#define MPU6050_DEADBAND_DPS (1.5f)
#define MPU6050_MAX_RATE_DPS (250.0f)
#define MPU6050_STALE_MS (50U)
#define MPU6050_MAX_CONSECUTIVE_ERRORS (3U)
```

- [ ] **Step 4: Run focused tests**

Run:

```powershell
python -m unittest tests.test_mpu6050 tests.test_mpu6050_contract -v
```

Expected: PASS with the host harness compiling and exiting zero.

- [ ] **Step 5: Commit and push**

```powershell
git add MSPM0G3507_LineFollowing_Car/modules/mpu6050 MSPM0G3507_LineFollowing_Car/.cproject tests/mpu6050_harness.c tests/test_mpu6050.py tests/test_mpu6050_contract.py
git commit -m "feat: add nonblocking MPU6050 yaw rate"
git push origin main
```

### Task 3: Add bounded yaw-rate feedback to normal line control

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/config/line_control_config.h`
- Modify: `tests/line_controller_harness.c`
- Modify: `tests/test_line_controller.py`

**Interfaces:**
- Consumes: the existing `yaw_rate_dps` and `yaw_fresh` arguments of
  `LineController_Step`.
- Produces: a turn command corrected by a bounded yaw-rate proportional loop.

- [ ] **Step 1: Add failing controller cases**

For the same left-turn line estimate, assert:

```c
assert(no_imu.turn == stale_imu.turn);
assert(slow_left_yaw.turn > matched_left_yaw.turn);
assert(fast_left_yaw.turn < matched_left_yaw.turn);
assert(abs(fast_left_yaw.turn - no_imu.turn) <= config.yaw_assist_limit);
```

Use fresh sequence numbers and reset the controller between cases so slew
history does not couple assertions.

- [ ] **Step 2: Run the controller test and verify RED**

Run:

```powershell
python -m unittest tests.test_line_controller -v
```

Expected: FAIL because yaw rate currently affects speed only, not turn.

- [ ] **Step 3: Implement one bounded helper**

Add `yaw_rate_per_command`, `yaw_rate_kp`, and `yaw_assist_limit` to
`LineControlConfig`. Before turn slew limiting, compute:

```c
target_rate = base_turn * control_config.yaw_rate_per_command;
assist = (target_rate - yaw_rate_dps) * control_config.yaw_rate_kp;
assist = clamp(assist,
               -control_config.yaw_assist_limit,
               control_config.yaw_assist_limit);
target_turn = clamp(base_turn + assist, -turn_limit, turn_limit);
```

When `yaw_fresh` is false, preserve the existing result exactly.

- [ ] **Step 4: Run controller and regression tests**

Run:

```powershell
python -m unittest tests.test_line_controller tests.test_line_following_contract tests.test_line_following_burn_profile -v
```

Expected: PASS.

- [ ] **Step 5: Commit and push**

```powershell
git add MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.c MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_controller.h MSPM0G3507_LineFollowing_Car/application/config/line_control_config.h tests/line_controller_harness.c tests/test_line_controller.py
git commit -m "feat: damp line steering with yaw rate"
git push origin main
```

### Task 4: Integrate startup hold, fail-soft limiting, and corner yaw braking

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/application/app_scheduler.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/config/line_following_profile.h`
- Modify: `MSPM0G3507_LineFollowing_Car/application/corner_maneuver.h`
- Modify: `MSPM0G3507_LineFollowing_Car/application/corner_maneuver.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/config/corner_maneuver_config.h`
- Modify: `tests/corner_maneuver_harness.c`
- Modify: `tests/test_corner_maneuver.py`
- Modify: `tests/test_app_scheduler.py`
- Modify: `tests/test_line_following_burn_profile.py`

**Interfaces:**
- Consumes: `Mpu6050Snapshot` and `Mpu6050State`.
- Produces: stopped calibration startup, fresh yaw input to line/corner control,
  and fail-soft speed-limited mission requests.

- [ ] **Step 1: Write failing scheduler and corner tests**

Contracts must verify:

```python
self.assertIn("Mpu6050_Init", scheduler)
self.assertIn("BSP_I2C_Service", scheduler)
self.assertIn("Mpu6050_Service", scheduler)
self.assertIn("Mpu6050_GetSnapshot", scheduler)
self.assertIn("MPU6050_STATE_CALIBRATING", scheduler)
self.assertIn("LINE_FOLLOWING_IMU_DEGRADED_LIMIT", profile)
```

The corner harness must show that a high yaw rate in the commanded turn
direction produces a smaller wheel-speed difference than a low yaw rate,
while stale yaw preserves the existing pivot request.

- [ ] **Step 2: Run focused tests and verify RED**

Run:

```powershell
python -m unittest tests.test_app_scheduler tests.test_corner_maneuver tests.test_line_following_burn_profile -v
```

Expected: FAIL because the scheduler passes `0.0F, false` and corner control
does not accept yaw rate.

- [ ] **Step 3: Wire the sensor and add fail-soft behavior**

Set:

```c
#define LINE_FOLLOWING_USE_IMU (1)
#define LINE_FOLLOWING_REQUIRE_IMU (0)
#define LINE_FOLLOWING_IMU_STARTUP_TIMEOUT_MS (2600U)
#define LINE_FOLLOWING_IMU_DEGRADED_LIMIT (180)
```

Call BSP I2C and MPU services from the fast scheduler path. During startup
calibration, invalidate the motion request until READY or timeout. Feed a
fresh snapshot to `LineController_Step` and `CornerManeuver_Step`. If the
optional IMU is stale/degraded after startup, cap each requested wheel to the
configured magnitude but keep the request valid.

Extend `CornerManeuver_Step` with yaw rate and freshness. In `COMMIT/SEEK`,
only when measured yaw has the commanded sign and exceeds
`CORNER_HIGH_YAW_RATE_DPS`, use configured reduced inner/outer commands.
Line reacquisition and all state timeouts remain unchanged.

- [ ] **Step 4: Run focused and full host tests**

Run:

```powershell
python -m unittest tests.test_app_scheduler tests.test_corner_maneuver tests.test_line_following_burn_profile -v
python -m unittest discover -s tests -p "test_*.py"
```

Expected: all tests PASS.

- [ ] **Step 5: Commit and push**

```powershell
git add MSPM0G3507_LineFollowing_Car/application/app_scheduler.c MSPM0G3507_LineFollowing_Car/application/config/line_following_profile.h MSPM0G3507_LineFollowing_Car/application/corner_maneuver.c MSPM0G3507_LineFollowing_Car/application/corner_maneuver.h MSPM0G3507_LineFollowing_Car/application/config/corner_maneuver_config.h tests/corner_maneuver_harness.c tests/test_corner_maneuver.py tests/test_app_scheduler.py tests/test_line_following_burn_profile.py
git commit -m "feat: fuse MPU6050 into line following"
git push origin main
```

### Task 5: Build and package the burn image

**Files:**
- Regenerate: `MSPM0G3507_LineFollowing_Car/Debug/ti_msp_dl_config.c`
- Regenerate: `MSPM0G3507_LineFollowing_Car/Debug/ti_msp_dl_config.h`
- Regenerate: `firmware/MSPM0G3507_LineFollowing_Car.hex`
- Regenerate: `firmware/MSPM0G3507_LineFollowing_Car.txt`

**Interfaces:**
- Consumes: all prior tasks.
- Produces: TI-linked image and UniFlash-compatible HEX/TXT files.

- [ ] **Step 1: Run the complete test suite**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py"
```

Expected: zero failures.

- [ ] **Step 2: Regenerate SysConfig and run a clean TI build**

Use the repository's pinned MSPM0 SDK 2.10.00.04 and TI Arm Clang 4.0.4.
Run the same SysConfig generation and Debug make commands recorded by the
current CCS project. Expected: every source compiles and the linker exits zero.

- [ ] **Step 3: Export and validate UniFlash files**

Generate Intel HEX and TI-TXT from the linked `.out`. Parse both outputs and
assert that their address-to-byte maps are identical and non-empty.

- [ ] **Step 4: Commit tracked build metadata only if generation changed it**

Do not add the user-owned untracked `firmware/` directory unless the user
explicitly requests firmware artifacts to be tracked. If tracked SysConfig
metadata changes, commit and push it separately:

```powershell
git add <changed-tracked-generated-files>
git commit -m "build: regenerate MPU6050 firmware metadata"
git push origin main
```

- [ ] **Step 5: Report first-power checks**

Before floor testing: raise the drive wheels, keep the car still for two
seconds, confirm no motion during calibration, rotate the nose left and right
by hand to verify yaw sign, then lower the car for reduced-speed line testing.
