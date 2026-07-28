# Single-Project Modular Cascade Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复当前上电卡死和循迹方向错误，在唯一的 CCS 工程内按功能整理源码，并加入“循迹位置外环 + MPU6050 角速度内环 + 电机板轮速闭环”的串级控制。

**Architecture:** `empty.c` 只负责硬件生成代码初始化、应用启动和 FreeRTOS 调度；`app/` 只负责任务编排与安全决策；每个硬件或算法功能集中在 `modules/<feature>/`；未启用的 K230、超声波、YB-IMU 和旧算法统一放在 `modules/optional/` 并从构建排除。循迹控制保留查表控制作为台架回退模式，新串级控制由外环把横向位置误差变成目标角速度，内环用 MPU6050 Z 轴角速度反馈产生左右轮差速。

**Tech Stack:** MSPM0G3507、TI DriverLib、TI Arm Clang 4.0.4、CCS Theia、FreeRTOS 静态任务、C11 主机测试、Python `unittest`。

## Global Constraints

- 只保留一个 CCS 工程：`MSPM0G3507_LineFollowing_Car/`；不得创建第二个 CCS 工程。
- `empty.syscfg` 是引脚与外设配置的唯一真实来源；不得手改 `Debug/ti_msp_dl_config.c`。
- 面向车头观察：X1 在右侧、X8 在左侧；软件位置定义保持“负值=线在车体左侧，正值=线在车体右侧”。
- 软件角速度约定为“正值=从车顶看顺时针右转”；实际安装方向只通过 `MPU6050_YAW_SIGN` 校准。
- 电机上电默认禁止输出；只有连续有效黑线帧通过启动门后才允许 `Motor_Safety_Arm()`。
- 电机安全层必须保留 0→30% soft-start、命令限幅、UART 超时和 1 ms 失控监控；禁止直接输出 100%。
- 安全判断采用独立 `if` + 立即 `return`；互斥业务分支采用 `if/else`；状态流采用 `switch`。
- 活跃控制路径不得使用阻塞式 `delay_ms()`；OLED、MPU6050 不得阻塞车辆安全任务。
- CCS 产物保留在 `Debug/`；CLI 产物写入 `build/cli/`；UniFlash 的 HEX 和 TI-TXT 写入 `dist/firmware/`。
- C/H 文件继续保持各自现有编码；新 C/H 文件使用 UTF-8，无 BOM。
- 不新增第三方依赖，不增加接口工厂、动态注册表或运行时模块加载器。

---

## Locked File Structure

```text
MSPM0G3507_LineFollowing_Car/
├── app/
│   ├── boot/app_boot.[ch]
│   ├── mailbox/app_mailbox.[ch]
│   ├── safety/safety_supervisor.[ch]
│   └── tasks/app_tasks.[ch]
├── config/
│   ├── line_cascade_config.h
│   ├── line_following_profile.h
│   ├── line_lookup_config.h
│   ├── line_recovery_config.h
│   ├── mpu6050_config.h
│   └── safety_config.h
├── modules/
│   ├── display/
│   │   ├── dashboard.[ch]
│   │   ├── i2c/oled_i2c.[ch]
│   │   └── ssd1306/ssd1306.[ch]
│   ├── line_tracking/
│   │   ├── controller/line_cascade_control.[ch]
│   │   ├── controller/line_lookup_control.[ch]
│   │   ├── controller/line_speed_profile.[ch]
│   │   ├── decoder/line_position.[ch]
│   │   ├── recovery/line_recovery.[ch]
│   │   └── scanner/line_mux.[ch]
│   │       scanner/line_scanner.[ch]
│   ├── motor/
│   │   ├── adapter/motor_adapter.[ch]
│   │   ├── configuration/motor_configuration.[ch]
│   │   ├── protocol/motor_protocol.[ch]
│   │   ├── safety/motor_safety.[ch]
│   │   └── uart/motor_uart.[ch]
│   ├── mpu6050/
│   │   ├── calibration/mpu6050_calibration.[ch]
│   │   ├── driver/mpu6050.[ch]
│   │   ├── filter/mpu6050_filter.[ch]
│   │   └── i2c/soft_i2c.[ch]
│   ├── buzzer/
│   ├── key/
│   ├── led/
│   ├── time/
│   └── optional/
│       ├── k230/
│       ├── ultrasonic/
│       ├── ybimu/
│       └── legacy/
├── shared/
│   ├── module_status.h
│   ├── line_control_types.h
│   ├── motion_request.h
│   └── safety_decision.h
├── empty.c
├── empty.syscfg
├── FreeRTOSConfig.h
└── Makefile
```

The split is deliberately limited to real responsibilities already present in the code. No generic HAL, dependency injection layer, or plugin registry is introduced.

---

### Task 1: Freeze the Baseline and Add Boot-Time Regression Coverage

**Files:**
- Create: `tests/test_boot_timebase_contract.py`
- Read only: `MSPM0G3507_LineFollowing_Car/application/app_main.c`
- Read only: `MSPM0G3507_LineFollowing_Car/bsp/time/timer.c`

**Interfaces:**
- Consumes: `Timer_Init(void)`, `BSP_Time_GetUs(void)`.
- Produces: a regression test proving the 1 MHz timebase starts before any delay-based motor configuration.

- [ ] **Step 1: Record the existing dirty worktree without modifying user files**

Run:

```powershell
git status --short
```

Expected: `.claude/settings.local.json` may be modified; preserve it and exclude it from every commit.

- [ ] **Step 2: Run the current host suite**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py"
```

Expected: record the exact pass/fail count. Existing failures are baseline evidence and are not hidden by later commits.

- [ ] **Step 3: Write the failing timebase contract**

Create `tests/test_boot_timebase_contract.py`:

```python
from pathlib import Path
import unittest


PROJECT = (
    Path(__file__).resolve().parents[1]
    / "MSPM0G3507_LineFollowing_Car"
)


class BootTimebaseContract(unittest.TestCase):
    def test_microsecond_counter_starts_before_motor_configuration(self):
        timer = (PROJECT / "bsp/time/timer.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        app = (PROJECT / "application/app_main.c").read_text(
            encoding="utf-8", errors="ignore"
        )

        self.assertIn(
            "DL_TimerG_startCounter(MICROSECOND_TIMEBASE_INST)", timer
        )
        self.assertLess(app.index("Timer_Init()"), app.index("Set_Motor(5)"))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 4: Run the test and verify the diagnosed failure**

Run:

```powershell
python -m unittest tests.test_boot_timebase_contract -v
```

Expected: FAIL because `Timer_Init()` currently starts only `TIMER_0_INST`.

- [ ] **Step 5: Commit only the failing regression test**

```powershell
git add tests/test_boot_timebase_contract.py
git commit -m "test: cover boot microsecond timebase"
```

---

### Task 2: Fix the Boot Deadlock at Its Root Cause

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/bsp/time/timer.c`
- Test: `tests/test_boot_timebase_contract.py`

**Interfaces:**
- Consumes: SysConfig-generated `MICROSECOND_TIMEBASE_INST`.
- Produces: a wrapping, live 1 MHz counter before `Set_Motor(5)` calls `delay_ms(100)`.

- [ ] **Step 1: Start the microsecond counter in `Timer_Init`**

Change `Timer_Init` to:

```c
void Timer_Init(void)
{
    DL_TimerG_startCounter(MICROSECOND_TIMEBASE_INST);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);
}
```

- [ ] **Step 2: Run the focused contract**

Run:

```powershell
python -m unittest tests.test_boot_timebase_contract -v
```

Expected: PASS.

- [ ] **Step 3: Run timebase-dependent host tests**

Run:

```powershell
python -m unittest tests.test_line_scanner_timebase tests.test_motor_uart_timeout tests.test_oled_contract -v
```

Expected: PASS.

- [ ] **Step 4: Build without powering the motor driver**

Run:

```powershell
gmake -C MSPM0G3507_LineFollowing_Car -j4 all
```

Expected: exit code 0 and a linked `.out`.

- [ ] **Step 5: Commit the root-cause fix**

```powershell
git add MSPM0G3507_LineFollowing_Car/bsp/time/timer.c
git commit -m "fix: start boot microsecond timebase"
```

---

### Task 3: Correct X1/X8 Physical Orientation

**Files:**
- Modify: `tests/line_position_harness.c`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_position.c`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_lookup_control.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_lookup_control.c`

**Interfaces:**
- Consumes: raw bitmap where bit 0 is X1 and bit 7 is X8.
- Produces: X1/right=`+7`, X8/left=`-7`; negative position still commands a left turn.

- [ ] **Step 1: Reverse the expected fifteen-position sequence in the harness**

Replace the first fifteen assertions with:

```c
LinePosition_Reset();
CHECK(LinePosition_Update(0x01U).stable_position == 7);
LinePosition_Reset();
CHECK(LinePosition_Update(0x03U).stable_position == 6);
LinePosition_Reset();
CHECK(LinePosition_Update(0x02U).stable_position == 5);
LinePosition_Reset();
CHECK(LinePosition_Update(0x06U).stable_position == 4);
LinePosition_Reset();
CHECK(LinePosition_Update(0x04U).stable_position == 3);
LinePosition_Reset();
CHECK(LinePosition_Update(0x0CU).stable_position == 2);
LinePosition_Reset();
CHECK(LinePosition_Update(0x08U).stable_position == 1);
LinePosition_Reset();
CHECK(LinePosition_Update(0x18U).stable_position == 0);
LinePosition_Reset();
CHECK(LinePosition_Update(0x10U).stable_position == -1);
LinePosition_Reset();
CHECK(LinePosition_Update(0x30U).stable_position == -2);
LinePosition_Reset();
CHECK(LinePosition_Update(0x20U).stable_position == -3);
LinePosition_Reset();
CHECK(LinePosition_Update(0x60U).stable_position == -4);
LinePosition_Reset();
CHECK(LinePosition_Update(0x40U).stable_position == -5);
LinePosition_Reset();
CHECK(LinePosition_Update(0xC0U).stable_position == -6);
LinePosition_Reset();
CHECK(LinePosition_Update(0x80U).stable_position == -7);
```

Update the later jump checks consistently: `0x01U` is `+7`, `0x03U` is `+6`, and `0x80U` is `-7`.

- [ ] **Step 2: Run the decoder test and verify it fails**

Run:

```powershell
python -m unittest tests.test_line_position -v
```

Expected: FAIL on the first X1 assertion.

- [ ] **Step 3: Reverse the decode table**

Use this table in `line_position.c`:

```c
static const int8_t position_by_bits[256] = {
    [0x01] = 7,  [0x03] = 6,  [0x02] = 5,  [0x06] = 4,
    [0x04] = 3,  [0x0C] = 2,  [0x08] = 1,  [0x18] = 0,
    [0x10] = -1, [0x30] = -2, [0x20] = -3, [0x60] = -4,
    [0x40] = -5, [0xC0] = -6, [0x80] = -7
};
```

Update comments in the lookup controller to state:

```c
/* Negative position means the line is left of the car.
 * Positive differential slows the left wheel and turns left. */
```

- [ ] **Step 4: Run decoder and steering symmetry tests**

Run:

```powershell
python -m unittest tests.test_line_position tests.test_line_lookup_control tests.test_tracking_decision_table -v
```

Expected: PASS.

- [ ] **Step 5: Commit the orientation correction**

```powershell
git add tests/line_position_harness.c MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_position.c MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_lookup_control.h MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_lookup_control.c
git commit -m "fix: match line positions to X1-right orientation"
```

---

### Task 4: Gate Automatic Start on a Valid Line

**Files:**
- Create: `tests/test_line_start_gate.py`
- Create: `tests/line_start_gate_harness.c`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_start_gate.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_start_gate.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/freertos/app_tasks.c`
- Modify: `MSPM0G3507_LineFollowing_Car/Makefile`

**Interfaces:**
- Produces: `void LineStartGate_Reset(void)` and `bool LineStartGate_Update(LinePatternType type)`.
- Consumes: two consecutive `LINE_PATTERN_POSITION` frames.
- Produces: a latched `true` start request; LOST, WIDE, and NOISE reset the pre-start counter.

- [ ] **Step 1: Write the harness**

Create `tests/line_start_gate_harness.c`:

```c
#include <assert.h>
#include "modules/line_tracking/line_start_gate.h"

int main(void)
{
    LineStartGate_Reset();
    assert(!LineStartGate_Update(LINE_PATTERN_LOST));
    assert(!LineStartGate_Update(LINE_PATTERN_POSITION));
    assert(LineStartGate_Update(LINE_PATTERN_POSITION));
    assert(LineStartGate_Update(LINE_PATTERN_NOISE));

    LineStartGate_Reset();
    assert(!LineStartGate_Update(LINE_PATTERN_POSITION));
    assert(!LineStartGate_Update(LINE_PATTERN_WIDE));
    assert(!LineStartGate_Update(LINE_PATTERN_POSITION));
    assert(LineStartGate_Update(LINE_PATTERN_POSITION));
    return 0;
}
```

Create `tests/test_line_start_gate.py` using the same MSVC compile pattern as `tests/test_line_position.py`, compiling the harness with `modules/line_tracking/line_start_gate.c`.

- [ ] **Step 2: Run the new test and verify it fails**

Run:

```powershell
python -m unittest tests.test_line_start_gate -v
```

Expected: FAIL because the module does not exist.

- [ ] **Step 3: Implement the minimal start gate**

`line_start_gate.h`:

```c
#ifndef LINE_START_GATE_H
#define LINE_START_GATE_H

#include <stdbool.h>
#include "line_position.h"

void LineStartGate_Reset(void);
bool LineStartGate_Update(LinePatternType type);

#endif
```

`line_start_gate.c`:

```c
#include "line_start_gate.h"

static unsigned char valid_frames;
static bool started;

void LineStartGate_Reset(void)
{
    valid_frames = 0U;
    started = false;
}

bool LineStartGate_Update(LinePatternType type)
{
    if (started) {
        return true;
    }
    if (type != LINE_PATTERN_POSITION) {
        valid_frames = 0U;
        return false;
    }
    if (valid_frames < 2U) {
        valid_frames++;
    }
    started = valid_frames >= 2U;
    return started;
}
```

- [ ] **Step 4: Integrate start ownership into `SafetyTask`**

In `app_tasks.c`:

- call `LineStartGate_Reset()` in `AppTasks_Create`;
- remove `Motor_Safety_Arm()` from `ControlTask`;
- keep a `static volatile bool line_start_ready`;
- update it only after a new line sample:

```c
line_start_ready = LineStartGate_Update(sample.position.type);
```

- in `SafetyTask`, use:

```c
inputs.start_pressed = line_start_ready;
```

- after `SafetySupervisor_Step`, arm once when the supervisor first reaches `SAFETY_RUNNING`:

```c
if (!motor_armed &&
    SafetySupervisor_GetState() == SAFETY_RUNNING) {
    Motor_Safety_Arm();
    motor_armed = true;
}
```

All rejected decisions still go through `MotorAdapter_Apply` as zero commands.

- [ ] **Step 5: Run start and safety tests**

Run:

```powershell
python -m unittest tests.test_line_start_gate tests.test_freertos_contract tests.test_safety_supervisor tests.test_motor_safety_contract -v
```

Expected: PASS after updating the FreeRTOS contract to require arm ownership in `SafetyTask`, not `ControlTask`.

- [ ] **Step 6: Commit the automatic line start**

```powershell
git add tests/test_line_start_gate.py tests/line_start_gate_harness.c tests/test_freertos_contract.py MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_start_gate.c MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_start_gate.h MSPM0G3507_LineFollowing_Car/application/freertos/app_tasks.c MSPM0G3507_LineFollowing_Car/Makefile
git commit -m "feat: start only after valid line acquisition"
```

---

### Task 5: Move Active Code into Feature-Owned Directories

**Files:**
- Move: active files according to `Locked File Structure`
- Modify: moved `#include` directives
- Modify: `tests/test_*.py`
- Modify: `tests/*_harness.c`
- Modify: `MSPM0G3507_LineFollowing_Car/.cproject`
- Modify: `MSPM0G3507_LineFollowing_Car/Makefile`

**Interfaces:**
- All public function signatures remain unchanged in this task.
- Only paths and include ownership change; behavior must remain byte-for-byte equivalent at the host-test level.

- [ ] **Step 1: Move the application files**

Use `git mv` for:

```text
application/app_main.[ch]                 -> app/boot/app_boot.[ch]
application/freertos/app_tasks.[ch]       -> app/tasks/app_tasks.[ch]
application/freertos/app_mailbox.[ch]     -> app/mailbox/app_mailbox.[ch]
application/safety_supervisor.[ch]        -> app/safety/safety_supervisor.[ch]
application/config/*.h                    -> config/*.h
application/diagnostics/dashboard.[ch]    -> modules/display/dashboard.[ch]
application/line_recovery.[ch]            -> modules/line_tracking/recovery/line_recovery.[ch]
```

Rename `App_Main_Init` to `AppBoot_Init` and update `empty.c`. Do not add an application facade.

- [ ] **Step 2: Move line-tracking files by responsibility**

```text
bsp/bsp_line_mux.[ch]                     -> modules/line_tracking/scanner/line_mux.[ch]
modules/line_tracking/line_scanner.[ch]   -> modules/line_tracking/scanner/line_scanner.[ch]
modules/line_tracking/line_position.[ch]  -> modules/line_tracking/decoder/line_position.[ch]
modules/line_tracking/line_start_gate.[ch]-> modules/line_tracking/decoder/line_start_gate.[ch]
modules/line_tracking/line_lookup_control.[ch]
                                           -> modules/line_tracking/controller/line_lookup_control.[ch]
```

- [ ] **Step 3: Move motor, display, time, and shared files**

```text
modules/common/*.h                         -> shared/*.h
bsp/bsp_oled_i2c.[ch]                     -> modules/display/i2c/oled_i2c.[ch]
modules/display/ssd1306.[ch]               -> modules/display/ssd1306/ssd1306.[ch]
bsp/time/timer.[ch]                        -> modules/time/timer.[ch]
bsp/delay.[ch]                             -> modules/time/delay.[ch]
modules/motor/app_motor.[ch]               -> modules/motor/configuration/motor_configuration.[ch]
modules/motor/app_motor_usart.[ch]         -> modules/motor/protocol/motor_protocol.[ch]
modules/motor/bsp_motor_usart.[ch]         -> modules/motor/uart/motor_uart.[ch]
modules/motor/motor_safety.[ch]            -> modules/motor/safety/motor_safety.[ch]
modules/motor/motor_adapter.[ch]           -> modules/motor/adapter/motor_adapter.[ch]
```

Keep `Set_Motor(5)` and all public motor APIs unchanged during this move.
Task 7 replaces only the blocking configuration API after the path-only
refactor has passed.

- [ ] **Step 4: Update include paths and build manifests**

Set the active source list in `Makefile` to only:

```text
empty.c
app/boot/app_boot.c
app/mailbox/app_mailbox.c
app/safety/safety_supervisor.c
app/tasks/app_tasks.c
modules/buzzer/buzzer.c
modules/display/dashboard.c
modules/display/i2c/oled_i2c.c
modules/display/ssd1306/ssd1306.c
modules/key/key.c
modules/led/led.c
modules/line_tracking/controller/line_lookup_control.c
modules/line_tracking/decoder/line_position.c
modules/line_tracking/decoder/line_start_gate.c
modules/line_tracking/recovery/line_recovery.c
modules/line_tracking/scanner/line_mux.c
modules/line_tracking/scanner/line_scanner.c
modules/motor/adapter/motor_adapter.c
modules/motor/configuration/motor_configuration.c
modules/motor/protocol/motor_protocol.c
modules/motor/safety/motor_safety.c
modules/motor/uart/motor_uart.c
modules/mpu6050/driver/mpu6050.c
modules/time/delay.c
modules/time/timer.c
```

Update `.cproject` include paths to the exact active directories. Remove stale include paths for `application`, legacy tasks, and legacy MPU6050.

- [ ] **Step 5: Update path-sensitive tests**

Replace old path literals in tests with their new exact locations. Strengthen `test_modular_architecture_contract.py` so it asserts the required roots are:

```python
for name in ("app", "config", "modules", "shared"):
    self.assertTrue((ROOT / name).is_dir(), name)
```

Also assert that `application` and `bsp` no longer exist after Task 6.

- [ ] **Step 6: Run the complete host suite**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py"
```

Expected: the same passing behavior as the Task 1 baseline plus the new boot, orientation, and start-gate tests.

- [ ] **Step 7: Commit the active-code move**

```powershell
git add MSPM0G3507_LineFollowing_Car tests
git commit -m "refactor: group active code by feature"
```

---

### Task 6: Quarantine Optional and Legacy Modules and Normalize Outputs

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/shared/line_control_types.h`
- Move: optional and legacy code
- Remove: `.claude/skills/`
- Modify: `.gitignore`
- Modify: `MSPM0G3507_LineFollowing_Car/.cproject`
- Modify: `MSPM0G3507_LineFollowing_Car/Makefile`
- Modify: `tests/test_modular_architecture_contract.py`

**Interfaces:**
- Produces: one excluded root, `modules/optional/`, for everything not called by the current firmware.
- Produces: CLI outputs in `build/cli/` and flash images in `dist/firmware/`.

- [ ] **Step 1: Move optional hardware modules**

```text
modules/k230_link/* and bsp/bsp_k230_uart.[ch]
    -> modules/optional/k230/
modules/ultrasonic/* and bsp/bsp_ultrasonic.[ch]
    -> modules/optional/ultrasonic/
modules/ybimu/*
    -> modules/optional/ybimu/
```

- [ ] **Step 2: Move old control implementations**

```text
application/app_scheduler.[ch]
application/corner_maneuver.[ch]
application/motion_primitives.[ch]
application/legacy_questions/
application/legacy_task/
modules/legacy_mpu6050/
modules/line_tracking/app_irtracking.[ch]
modules/line_tracking/line_controller.[ch]
modules/line_tracking/line_estimator.[ch]
modules/line_tracking/line_event_classifier.[ch]
modules/line_tracking/line_features.[ch]
modules/line_tracking/line_trend_detector.[ch]
bsp/debug_uart.[ch]
    -> modules/optional/legacy/<matching-feature>/
```

Before moving estimator/trend implementations, extract only the three data
contracts still consumed by `app_tasks` and active recovery code into
`shared/line_control_types.h`:

```c
#ifndef LINE_CONTROL_TYPES_H
#define LINE_CONTROL_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "module_status.h"

typedef enum {
    LINE_EVENT_NONE = 0,
    LINE_EVENT_HARD_LEFT,
    LINE_EVENT_HARD_RIGHT,
    LINE_EVENT_WIDE_BLACK,
    LINE_EVENT_LOST
} LineEvent;

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
    float error;
    float derivative;
    float predicted_error;
    uint8_t confidence;
    LineEvent event;
} LineEstimate;

typedef struct {
    ModuleStatus status;
    LineTrendType type;
    int8_t direction;
} LineTrendResult;

typedef struct {
    int16_t forward;
    int16_t turn;
    bool valid;
} LineControlOutput;

#endif
```

Update `line_recovery.h` and `app_tasks.c` to include this header. Keep
`LineRecovery_Step` unchanged so this task is structural only; the optional
legacy headers may also include the shared type header instead of redefining
the types.

- [ ] **Step 3: Exclude the single optional root in CCS**

Set `.cproject` source exclusion to include:

```xml
excluding="Debug|modules/optional"
```

Do not maintain per-file exclusions for optional modules.

- [ ] **Step 4: Normalize CLI and firmware output paths**

In `Makefile` use:

```make
BUILD_DIR   := ../build/cli/$(NAME)
OBJ_DIR     := $(BUILD_DIR)/obj
FIRMWARE_DIR := ../dist/firmware
```

Update `.gitignore`:

```gitignore
**/Debug/
build/
dist/firmware/
tmp/
```

- [ ] **Step 5: Remove the duplicate Claude skill copy**

Delete only `.claude/skills/`. Preserve `.claude/commands/`, `.claude/plans/`, and the user-modified `.claude/settings.local.json`. `.agents/skills/` remains the sole in-repository skill source.

- [ ] **Step 6: Verify optional code is not compiled**

Add assertions to `test_modular_architecture_contract.py`:

```python
makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
cproject = (ROOT / ".cproject").read_text(encoding="utf-8")
self.assertIn("modules/optional", cproject)
self.assertNotIn("modules/optional/", makefile)
self.assertIn("../build/cli/$(NAME)", makefile)
self.assertIn("../dist/firmware", makefile)
```

- [ ] **Step 7: Run contracts and CLI build**

Run:

```powershell
python -m unittest tests.test_modular_architecture_contract tests.test_freertos_contract tests.test_text_encoding -v
gmake -C MSPM0G3507_LineFollowing_Car -j4 clean all images
```

Expected:

```text
build/cli/MSPM0G3507_LineFollowing_Car/MSPM0G3507_LineFollowing_Car.out
dist/firmware/MSPM0G3507_LineFollowing_Car.hex
dist/firmware/MSPM0G3507_LineFollowing_Car.txt
```

- [ ] **Step 8: Commit the quarantine and output layout**

```powershell
git add .claude .gitignore MSPM0G3507_LineFollowing_Car tests
git commit -m "refactor: quarantine optional modules and build outputs"
```

---

### Task 7: Make Motor Configuration Nonblocking

**Files:**
- Create: `tests/test_motor_configuration.py`
- Create: `tests/motor_configuration_harness.c`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/configuration/motor_configuration.[ch]`
- Modify: `MSPM0G3507_LineFollowing_Car/app/boot/app_boot.c`
- Modify: `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c`

**Interfaces:**
- Produces: `MotorConfiguration_Start`, `MotorConfiguration_Service`, `MotorConfiguration_IsComplete`.
- Consumes: existing protocol functions `send_motor_type`,
  `send_pulse_phase`, `send_pulse_line`, `send_wheel_diameter`, and
  `send_motor_deadzone`.
- Guarantees: no blocking delay and no speed command before configuration completion.

- [ ] **Step 1: Write a host harness with a fake clock and protocol recorder**

The harness calls `MotorConfiguration_Start(0)`, services at `0, 99, 100, 200, 300, 400 ms`, and asserts exactly one configuration frame is emitted at each 100 ms boundary in this order:

```text
send_motor_type(1)
-> send_pulse_phase(40)
-> send_pulse_line(11)
-> send_wheel_diameter(67.0)
-> send_motor_deadzone(1900)
```

It also asserts `MotorConfiguration_IsComplete()` is false through 399 ms and true after the final frame.

- [ ] **Step 2: Run the harness and verify it fails**

Run:

```powershell
python -m unittest tests.test_motor_configuration -v
```

Expected: FAIL because only blocking `Set_Motor(5)` exists.

- [ ] **Step 3: Implement the configuration state machine**

Use:

```c
typedef enum {
    MOTOR_CONFIG_TYPE = 0,
    MOTOR_CONFIG_PHASE,
    MOTOR_CONFIG_LINE,
    MOTOR_CONFIG_DIAMETER,
    MOTOR_CONFIG_DEADZONE,
    MOTOR_CONFIG_COMPLETE
} MotorConfigurationState;
```

`MotorConfiguration_Service(now_ms)` emits at most one frame per call, advances only when 100 ms elapsed, and never calls `delay_ms`.

- [ ] **Step 4: Integrate configuration into boot and safety**

- `AppBoot_Init` starts the timebase and calls `MotorConfiguration_Start(Get_Time())`.
- `SafetyTask` calls `MotorConfiguration_Service(now_ms)` every 1 ms.
- `inputs.start_pressed` is true only when both the line gate and configuration are ready:

```c
inputs.start_pressed =
    line_start_ready && MotorConfiguration_IsComplete();
```

- [ ] **Step 5: Run motor and boot tests**

Run:

```powershell
python -m unittest tests.test_motor_configuration tests.test_boot_timebase_contract tests.test_motor_safety_contract tests.test_motor_uart_timeout -v
```

Expected: PASS and no `delay_ms` in the active motor configuration source.

- [ ] **Step 6: Commit the nonblocking configuration**

```powershell
git add MSPM0G3507_LineFollowing_Car tests
git commit -m "refactor: configure motor without blocking boot"
```

---

### Task 8: Split MPU6050 Calibration and Filtering without Changing Behavior

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/modules/mpu6050/calibration/mpu6050_calibration.[ch]`
- Create: `MSPM0G3507_LineFollowing_Car/modules/mpu6050/filter/mpu6050_filter.[ch]`
- Move/Modify: `MSPM0G3507_LineFollowing_Car/modules/mpu6050/driver/mpu6050.[ch]`
- Move/Modify: `MSPM0G3507_LineFollowing_Car/modules/mpu6050/i2c/soft_i2c.[ch]`
- Modify: `tests/mpu6050_harness.c`
- Modify: `tests/test_mpu6050.py`

**Interfaces:**
- Produces: `Mpu6050Calibration_Reset`, `Mpu6050Calibration_Add`, `Mpu6050Calibration_IsReady`, `Mpu6050Calibration_GetBias`.
- Produces: `Mpu6050Filter_Reset`, `Mpu6050Filter_Apply`.
- Keeps: `Mpu6050_Init`, `Mpu6050_Service`, `Mpu6050_GetState`, `Mpu6050_GetSnapshot`.

- [ ] **Step 1: Extend the existing MPU harness**

Add assertions that:

- calibration becomes ready only after both 2000 ms and 150 samples;
- bias equals the arithmetic mean of the calibration samples;
- filter deadband converts `±1.0 dps` to zero;
- filter clamps beyond `±250 dps`;
- driver state transitions remain STARTUP → CALIBRATING → READY and DEGRADED recovery remains unchanged.

- [ ] **Step 2: Run the MPU test before splitting**

Run:

```powershell
python -m unittest tests.test_mpu6050 -v
```

Expected: existing driver behavior passes; new standalone API assertions fail to compile.

- [ ] **Step 3: Extract calibration state**

Move only sample count, sum, start time, and bias into `mpu6050_calibration.c`. The driver remains the sole owner of device state and I2C sequencing.

- [ ] **Step 4: Extract filtering**

Move sign correction, deadband, low-pass state, and ±250 dps clamp into `mpu6050_filter.c`. Keep configuration values in `config/mpu6050_config.h`.

- [ ] **Step 5: Keep the active driver API unchanged**

The driver uses the two focused modules internally, so mailbox and task code require no new abstraction.

- [ ] **Step 6: Run MPU and architecture tests**

Run:

```powershell
python -m unittest tests.test_mpu6050 tests.test_mpu6050_contract tests.test_modular_architecture_contract -v
```

Expected: PASS.

- [ ] **Step 7: Commit the focused MPU split**

```powershell
git add MSPM0G3507_LineFollowing_Car/modules/mpu6050 tests
git commit -m "refactor: separate MPU calibration and filtering"
```

---

### Task 9: Add the Cascade Controller as a Tested, Bounded Module

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/config/line_cascade_config.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_speed_profile.[ch]`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_cascade_control.[ch]`
- Create: `tests/line_cascade_control_harness.c`
- Create: `tests/test_line_cascade_control.py`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_lookup_control.c`

**Interfaces:**
- Produces: `LineSpeedProfile_GetBase(int8_t position)`.
- Produces: `LineCascadeControl_Reset()` and `LineCascadeControl_Step(position, yaw_rate_dps, yaw_fresh, now_ms)`.
- Produces: bounded left/right commands, base speed, `diff` in the existing
  `LineControlOutput.turn` convention (positive=left turn), target yaw rate,
  and validity.

`line_cascade_control.h` exposes exactly:

```c
#ifndef LINE_CASCADE_CONTROL_H
#define LINE_CASCADE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t left;
    int16_t right;
    int16_t base;
    int16_t diff;
    float target_yaw_rate_dps;
    bool valid;
} LineCascadeCommand;

void LineCascadeControl_Reset(void);
LineCascadeCommand LineCascadeControl_Step(int8_t position,
                                           float yaw_rate_dps,
                                           bool yaw_fresh,
                                           uint32_t now_ms);

#endif
```

- [ ] **Step 1: Write cascade behavior tests**

The C harness must assert:

```c
LineCascadeControl_Reset();
center = LineCascadeControl_Step(0, 0.0f, true, 10U);
CHECK(center.valid);
CHECK(center.left == center.right);

left = LineCascadeControl_Step(-7, 0.0f, true, 20U);
CHECK(left.target_yaw_rate_dps < 0.0f);
CHECK(left.left < left.right);

right = LineCascadeControl_Step(7, 0.0f, true, 30U);
CHECK(right.target_yaw_rate_dps > 0.0f);
CHECK(right.right < right.left);

stale = LineCascadeControl_Step(7, 0.0f, false, 40U);
CHECK(stale.valid);
CHECK(abs16(stale.left) <= 220);
CHECK(abs16(stale.right) <= 220);

LineCascadeControl_Reset();
restarted = LineCascadeControl_Step(0, 0.0f, true, 1000U);
CHECK(restarted.diff == 0);
```

Loop over positions `-7..7`, yaw rates `-250..250`, and timestamps across `uint32_t` wrap; assert every wheel command remains within `±450` and no invalid float reaches integer conversion.

- [ ] **Step 2: Run the new test and verify it fails**

Run:

```powershell
python -m unittest tests.test_line_cascade_control -v
```

Expected: FAIL because the controller does not exist.

- [ ] **Step 3: Add conservative, explicit calibration knobs**

Create `config/line_cascade_config.h`:

```c
#ifndef LINE_CASCADE_CONFIG_H
#define LINE_CASCADE_CONFIG_H

#define LINE_CASCADE_OUTER_PERIOD_MS       (10U)
#define LINE_CASCADE_OUTER_KP              (9.0f)
#define LINE_CASCADE_OUTER_KI              (0.15f)
#define LINE_CASCADE_OUTER_KD              (0.08f)
#define LINE_CASCADE_TARGET_YAW_LIMIT_DPS  (90.0f)
#define LINE_CASCADE_INNER_KP              (1.10f)
#define LINE_CASCADE_INNER_KI              (0.15f)
#define LINE_CASCADE_INNER_KD              (0.00f)
#define LINE_CASCADE_INTEGRAL_LIMIT        (80.0f)
#define LINE_CASCADE_DIFF_LIMIT            (180)
#define LINE_CASCADE_COMMAND_LIMIT         (450)
#define LINE_CASCADE_IMU_STALE_LIMIT       (220)

#endif
```

These are safe first-run values, not claimed final race tuning: target yaw and differential are deliberately lower than the current lookup edge command.

- [ ] **Step 4: Extract the shared base-speed profile**

`LineSpeedProfile_GetBase` returns:

```c
static const int16_t base_by_magnitude[8] = {
    220, 215, 205, 195, 180, 165, 150, 135
};
```

Both lookup and cascade controllers call this function. Do not duplicate the table.

- [ ] **Step 5: Implement outer position PID**

Every 10 ms:

```c
error = (float)position;
outer_integral = clamp(
    outer_integral + error * dt_s,
    -LINE_CASCADE_INTEGRAL_LIMIT,
    LINE_CASCADE_INTEGRAL_LIMIT);
derivative = (error - previous_error) / dt_s;
target_yaw_rate_dps = clamp(
    kp * error + ki * outer_integral + kd * derivative,
    -LINE_CASCADE_TARGET_YAW_LIMIT_DPS,
    LINE_CASCADE_TARGET_YAW_LIMIT_DPS);
```

Use the held target between outer-loop updates.

- [ ] **Step 6: Implement inner yaw-rate PID on every fresh control step**

```c
yaw_error = target_yaw_rate_dps - yaw_rate_dps;
inner_integral = clamp(
    inner_integral + yaw_error * dt_s,
    -LINE_CASCADE_INTEGRAL_LIMIT,
    LINE_CASCADE_INTEGRAL_LIMIT);
yaw_command = clamp_float(
    inner_kp * yaw_error +
    inner_ki * inner_integral +
    inner_kd * yaw_derivative,
    -(float)LINE_CASCADE_DIFF_LIMIT,
    (float)LINE_CASCADE_DIFF_LIMIT);
/* Positive yaw is a right turn, but positive follow.turn is a left turn. */
diff = (int16_t)(-yaw_command);
left = clamp(base - diff, -LINE_CASCADE_COMMAND_LIMIT,
             LINE_CASCADE_COMMAND_LIMIT);
right = clamp(base + diff, -LINE_CASCADE_COMMAND_LIMIT,
              LINE_CASCADE_COMMAND_LIMIT);
```

When IMU is stale, do not integrate; use proportional outer steering clamped to `LINE_CASCADE_IMU_STALE_LIMIT`. Reset both integrals on explicit reset, lost line, safety rejection, or a timestamp gap above 50 ms.

- [ ] **Step 7: Run focused controller tests**

Run:

```powershell
python -m unittest tests.test_line_cascade_control tests.test_line_lookup_control tests.test_line_position -v
```

Expected: PASS with symmetric commands, correct turn direction, bounded output, and reset behavior.

- [ ] **Step 8: Commit the standalone controller**

```powershell
git add MSPM0G3507_LineFollowing_Car/config/line_cascade_config.h MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller tests/line_cascade_control_harness.c tests/test_line_cascade_control.py
git commit -m "feat: add bounded line yaw cascade controller"
```

---

### Task 10: Integrate Cascade Scheduling with Lookup Fallback

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/config/line_following_profile.h`
- Modify: `MSPM0G3507_LineFollowing_Car/config/mpu6050_config.h`
- Modify: `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/recovery/line_recovery.c`
- Modify: `MSPM0G3507_LineFollowing_Car/Makefile`
- Create: `tests/test_cascade_integration_contract.py`

**Interfaces:**
- Produces two compile-time modes: `LINE_CONTROL_MODE_LOOKUP` and `LINE_CONTROL_MODE_CASCADE`.
- Default burn profile uses cascade only after host tests; one define switches back to lookup for track comparison.
- Sensor task remains 2 ms; MPU service is due every 5 ms; outer loop updates every 10 ms.

- [ ] **Step 1: Add the integration contract**

Assert:

```python
self.assertIn("#define LINE_CONTROL_MODE_CASCADE", profile)
self.assertIn("LINE_FOLLOWING_CONTROL_MODE", profile)
self.assertIn("LineCascadeControl_Step", tasks)
self.assertIn("LineCascadeControl_Reset", tasks)
self.assertIn("MPU6050_SAMPLE_PERIOD_MS (5U)", mpu_config)
self.assertNotIn("delay_ms(", tasks)
```

Also assert lookup control remains compiled and selectable.

- [ ] **Step 2: Run the contract and verify it fails**

Run:

```powershell
python -m unittest tests.test_cascade_integration_contract -v
```

Expected: FAIL before integration.

- [ ] **Step 3: Add the profile selector**

In `line_following_profile.h`:

```c
#define LINE_CONTROL_MODE_LOOKUP  (0U)
#define LINE_CONTROL_MODE_CASCADE (1U)
#define LINE_FOLLOWING_CONTROL_MODE LINE_CONTROL_MODE_CASCADE
```

- [ ] **Step 4: Service MPU without blocking startup**

Set:

```c
#define MPU6050_SAMPLE_PERIOD_MS (5U)
```

Call `Mpu6050_Service` from the existing 2 ms sensor task every cycle; the driver starts an I2C transaction only when its 5 ms deadline is due. Remove `ImuStartupHold`: while MPU calibrates, lookup fallback supplies a limited command; when a fresh yaw sample becomes available, cascade control takes over.

- [ ] **Step 5: Select the controller with mutually exclusive branches**

Inside `BuildMotionRequest`:

```c
if (LINE_FOLLOWING_CONTROL_MODE == LINE_CONTROL_MODE_CASCADE &&
    yaw_fresh) {
    cascade = LineCascadeControl_Step(
        sample->position.stable_position,
        yaw_rate_dps, true, now_ms);
    follow.forward = cascade.base;
    follow.turn = cascade.diff;
    follow.valid = cascade.valid;
} else {
    lookup = LineLookupControl_Step(
        sample->position.stable_position,
        yaw_rate_dps, yaw_fresh);
    follow.forward = lookup.base;
    follow.turn = lookup.diff;
    follow.valid = lookup.valid;
}
```

Before this branch, use independent safety guards with immediate return for invalid input, latched fault, and lost line. Do not allow a later branch to overwrite a stop decision.

- [ ] **Step 6: Reset controller state on every non-follow state**

Call `LineCascadeControl_Reset()` when:

- line pattern is LOST or NOISE;
- recovery state is not FOLLOW;
- safety decision is rejected;
- motor is disarmed or a task fault is latched.

- [ ] **Step 7: Run all host tests**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py"
```

Expected: all tests PASS.

- [ ] **Step 8: Build both control modes**

Build once with cascade selected and once with the profile define changed to lookup. For both:

```powershell
gmake -C MSPM0G3507_LineFollowing_Car -j4 clean all images
```

Expected: exit code 0; `.out`, `.hex`, and `.txt` are generated in the normalized output directories.

- [ ] **Step 9: Commit cascade integration**

```powershell
git add MSPM0G3507_LineFollowing_Car tests
git commit -m "feat: integrate cascade control with lookup fallback"
```

---

### Task 11: Update Documentation and Perform Staged Hardware Acceptance

**Files:**
- Modify: `README.md`
- Modify: `MSPM0G3507_LineFollowing_Car/README.md`
- Modify: `docs/setup/SETUP_GUIDE.md`
- Create: `docs/verification/cascade-control-test-record.md`
- Modify only after user approval: `PROJECT_SKILLS.md`

**Interfaces:**
- Produces: exact CCS, CLI, UniFlash, module-call, PID-tuning, and rollback instructions.

- [ ] **Step 1: Document the one-project layout**

Document:

- `app/` is orchestration only;
- each active feature is under `modules/<feature>/`;
- unused code is under `modules/optional/` and excluded;
- enabling an optional module requires adding its source to `Makefile` and removing its CCS exclusion deliberately;
- there is still exactly one CCS project.

- [ ] **Step 2: Document build and flash paths**

Use these exact paths:

```text
CCS/XDS110: MSPM0G3507_LineFollowing_Car/Debug/*.out
CLI: build/cli/MSPM0G3507_LineFollowing_Car/*.out
UniFlash: dist/firmware/MSPM0G3507_LineFollowing_Car.txt
HEX: dist/firmware/MSPM0G3507_LineFollowing_Car.hex
```

- [ ] **Step 3: Document controller tuning order**

Record this fixed order:

1. Lift wheels; verify X1 produces positive position and X8 negative.
2. Rotate the lifted car by hand: clockwise right turn must report positive yaw; otherwise change only `MPU6050_YAW_SIGN`.
3. Set base speed to 145 and differential limit to 90.
4. Tune inner yaw-rate `Kp`; increase until following commanded yaw is quick but not oscillatory.
5. Add only enough inner `Ki` to remove steady yaw error; keep `Kd=0` unless logged gyro data proves it helps.
6. Tune outer position `Kp`; then add small `Kd`; add outer `Ki` last.
7. Increase base speed by 20 per run while preserving the 30% soft-start and ±450 command ceiling.
8. Compare cascade and lookup modes on the same track and retain the faster stable mode.

- [ ] **Step 4: Stop for the mandatory powered-motor checklist**

Before connecting 12.6 V motor power, confirm with the user:

```text
[ ] 车轮悬空
[ ] MCU、传感器、电机驱动共地
[ ] 电机安全层 soft-start 保持启用
[ ] 命令限幅不超过 ±450
[ ] UART 超时与 1 ms watchdog 测试已通过
[ ] X1=右、X8=左的 OLED/串口读数已确认
[ ] 顺时针右转时 MPU6050 角速度为正
[ ] D2 已恢复心跳
[ ] OLED 已恢复显示
[ ] 手边可以立即断开 12.6 V 电源
```

Do not proceed to motor rotation without explicit confirmation.

- [ ] **Step 5: Perform unpowered acceptance**

With the motor driver power disconnected:

- flash `dist/firmware/MSPM0G3507_LineFollowing_Car.txt`;
- verify D2 heartbeat;
- verify OLED diagnostics;
- move a black strip from X8 to X1 and record positions `-7 → 0 → +7`;
- verify no motor command is approved before two valid position frames.

- [ ] **Step 6: Perform wheels-up powered acceptance after confirmation**

Record:

- startup delay to first valid command;
- 0→30% soft-start duration;
- X8/left line makes left wheel slower;
- X1/right line makes right wheel slower;
- lost line commands zero or enters the bounded recovery state;
- stale MPU falls back to limited lookup control;
- watchdog stops commands when the control task heartbeat is withheld.

- [ ] **Step 7: Run final software verification**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py"
gmake -C MSPM0G3507_LineFollowing_Car -j4 clean all images
git status --short
```

Expected:

- all host tests PASS;
- firmware build and image generation exit 0;
- only intentional files are modified;
- `.claude/settings.local.json` remains untouched by the implementation commits.

- [ ] **Step 8: Commit documentation and verification record**

```powershell
git add README.md MSPM0G3507_LineFollowing_Car/README.md docs/setup/SETUP_GUIDE.md docs/verification/cascade-control-test-record.md
git commit -m "docs: explain modular cascade firmware workflow"
```

---

## Completion Criteria

- D2 heartbeat and OLED operate after boot; the microsecond counter cannot remain stopped.
- X1/right decodes positive and X8/left decodes negative.
- No motor arming occurs before two consecutive valid line-position frames and completed motor configuration.
- Active startup and task paths contain no blocking `delay_ms`.
- Exactly one CCS project remains, and all optional code is under one excluded root.
- CLI and UniFlash artifacts are generated in `build/cli/` and `dist/firmware/`.
- Cascade output is symmetric, direction-correct, resettable, stale-IMU tolerant, and bounded by the existing safety layer.
- Lookup mode remains available through one compile-time profile define.
- Full host suite, both control-mode builds, unpowered checks, and confirmed wheels-up checks are recorded.
