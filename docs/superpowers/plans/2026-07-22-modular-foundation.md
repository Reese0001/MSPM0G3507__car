# Modular Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不启动新传感器和不改变现有电机安全行为的前提下，建立 `application/ → modules/ → bsp/` 目录、公共快照合同和非阻塞调度入口。

**Architecture:** 先用合同测试锁定目录和依赖，再通过 Git 感知的移动把现有 BSP 拆到模块层与板级层。`empty.c` 仅执行生成初始化和 `App_Main()`；迁移阶段默认不 Arm 电机，先证明结构和构建稳定。

**Tech Stack:** C11、TI DriverLib、CCS/SysConfig、Python unittest、Git worktree。

## Global Constraints

- 分支/工作树：`codex/modular-foundation`，基于 `8cfd480`。
- 不修改 `empty.syscfg` 的引脚或外设实例。
- 不改变 M2/M4 映射、soft-start 或 200ms watchdog。
- Windows 大小写不敏感；`BSP` 改名必须经过临时目录。
- 新代码 UTF-8；被移动的旧 C/H 文件保持原字节编码。

---

## File Structure

- Create: `MSPM0G3507_LineFollowing_Car/application/app_main.c/.h` — 顶层初始化与循环步进。
- Create: `MSPM0G3507_LineFollowing_Car/application/app_scheduler.c/.h` — 无 RTOS 周期任务调度。
- Create: `MSPM0G3507_LineFollowing_Car/modules/common/module_status.h` — 统一快照状态。
- Create: `MSPM0G3507_LineFollowing_Car/modules/common/motion_request.h` — 跨控制与安全层的运动请求。
- Move: `BSP/Motor` → `modules/motor`。
- Move: `BSP/Eight_Tracking` → `modules/line_tracking`。
- Move: `BSP/Key|LED|Buzzer` → `modules/key|led|buzzer`。
- Move: `BSP/MPU6050|eMPL` → `modules/legacy_mpu6050`。
- Move: `BSP/Timer` → `bsp/time`；`BSP/usart.*` → `bsp/debug_uart.*`；`BSP/delay.*` → `bsp/delay.*`。
- Move: `BSP/Questions` → `application/legacy_questions`。
- Remove after migration: `BSP/Task`，由 `application/app_scheduler.*` 替代。
- Modify: `empty.c`, `.cproject`, `README.md`, `docs/setup/SETUP_GUIDE.md`。
- Test: `tests/test_modular_architecture_contract.py`, existing `tests/test_*.py` path updates。

### Task 1: 锁定模块目录与依赖合同

**Interfaces:**
- Consumes: 当前工程文件清单
- Produces: 路径、include 和禁止反向依赖的自动检查

- [ ] **Step 1: 写失败测试**

Create `tests/test_modular_architecture_contract.py`:

```python
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"

class ModularArchitectureContract(unittest.TestCase):
    def test_required_roots_exist(self):
        for name in ("application", "modules", "bsp"):
            self.assertTrue((ROOT / name).is_dir(), name)

    def test_legacy_bsp_is_removed(self):
        self.assertFalse((ROOT / "BSP").exists())

    def test_lower_layers_do_not_include_application(self):
        for base in (ROOT / "modules", ROOT / "bsp"):
            for path in base.rglob("*.[ch]"):
                text = path.read_text(encoding="utf-8", errors="ignore")
                self.assertNotIn('#include "application/', text, str(path))
```

- [ ] **Step 2: 验证测试失败**

Run: `python -m unittest tests.test_modular_architecture_contract -v`

Expected: FAIL，因为三个新目录不存在且旧 `BSP/` 仍存在。

- [ ] **Step 3: 用 Git 移动目录**

```powershell
git mv MSPM0G3507_LineFollowing_Car/BSP MSPM0G3507_LineFollowing_Car/_bsp_migration
New-Item -ItemType Directory MSPM0G3507_LineFollowing_Car/application,MSPM0G3507_LineFollowing_Car/modules,MSPM0G3507_LineFollowing_Car/bsp
git mv MSPM0G3507_LineFollowing_Car/_bsp_migration/Motor MSPM0G3507_LineFollowing_Car/modules/motor
git mv MSPM0G3507_LineFollowing_Car/_bsp_migration/Eight_Tracking MSPM0G3507_LineFollowing_Car/modules/line_tracking
```

继续用 `git mv` 按 File Structure 完成剩余移动；最后仅在确认 `_bsp_migration` 为空后移除空目录。

- [ ] **Step 4: 更新 include 与工程路径**

在 `.cproject` 中把 `${PROJECT_ROOT}/BSP/...` 替换成明确的新目录 include；源文件 include 使用模块公共头名，不使用 `../..` 相对穿越。

Run: `python -m unittest tests.test_modular_architecture_contract -v`

Expected: PASS。

- [ ] **Step 5: 提交目录迁移**

```bash
git add MSPM0G3507_LineFollowing_Car tests/test_modular_architecture_contract.py
git commit -m "refactor: establish modular project layout"
```

### Task 2: 增加公共模块状态合同

**Interfaces:**
- Consumes: `uint32_t now_ms`
- Produces: `ModuleStatus`, `ModuleStatus_IsFresh(const ModuleStatus *, uint32_t, uint32_t)`

- [ ] **Step 1: 扩展失败测试**

```python
    def test_module_status_contract_exists(self):
        text = (ROOT / "modules/common/module_status.h").read_text(encoding="utf-8")
        for token in ("timestamp_ms", "sequence", "valid", "health", "ModuleStatus_IsFresh"):
            self.assertIn(token, text)
```

Run: `python -m unittest tests.test_modular_architecture_contract -v`

Expected: FAIL，`module_status.h` 不存在。

- [ ] **Step 2: 实现公共头**

Create `modules/common/module_status.h`:

```c
#ifndef MODULE_STATUS_H
#define MODULE_STATUS_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MODULE_HEALTH_UNKNOWN = 0,
    MODULE_HEALTH_OK,
    MODULE_HEALTH_DEGRADED,
    MODULE_HEALTH_FAULT
} ModuleHealth;

typedef struct {
    uint32_t timestamp_ms;
    uint16_t sequence;
    bool valid;
    ModuleHealth health;
} ModuleStatus;

static inline bool ModuleStatus_IsFresh(const ModuleStatus *status,
                                        uint32_t now_ms,
                                        uint32_t max_age_ms)
{
    return status != 0 && status->valid &&
           status->health != MODULE_HEALTH_FAULT &&
           (uint32_t)(now_ms - status->timestamp_ms) <= max_age_ms;
}
#endif
```

- [ ] **Step 3: 运行测试并提交**

Run: `python -m unittest tests.test_modular_architecture_contract -v`

Expected: PASS。

```bash
git add MSPM0G3507_LineFollowing_Car/modules/common/module_status.h tests/test_modular_architecture_contract.py
git commit -m "feat: add shared module status contract"
```

### Task 3: 定义跨模块运动请求

**Interfaces:**
- Consumes: left/right target speed and `now_ms`
- Produces: `MotionRequest`

- [ ] **Step 1: 增加失败合同**

```python
    def test_motion_request_contract_exists(self):
        text = (ROOT / "modules/common/motion_request.h").read_text(encoding="utf-8")
        for token in ("MotionRequest", "left_speed", "right_speed", "timestamp_ms", "valid"):
            self.assertIn(token, text)
```

Run: `python -m unittest tests.test_modular_architecture_contract -v`

Expected: FAIL，运动请求头不存在。

- [ ] **Step 2: 实现公共请求类型**

```c
#ifndef MOTION_REQUEST_H
#define MOTION_REQUEST_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t left_speed;
    int16_t right_speed;
    uint32_t timestamp_ms;
    bool valid;
} MotionRequest;
#endif
```

- [ ] **Step 3: 验证并提交**

Run: `python -m unittest tests.test_modular_architecture_contract -v`

Expected: PASS。

```bash
git add MSPM0G3507_LineFollowing_Car/modules/common/motion_request.h tests/test_modular_architecture_contract.py
git commit -m "feat: define shared motion request"
```

### Task 4: 替换旧调度器并保持安全零速

**Interfaces:**
- Consumes: `BSP_Time_GetMs()`、静态任务表
- Produces: `AppScheduler_Init(uint32_t)`, `AppScheduler_Run(uint32_t)`, `App_Main_Init()`, `App_Main_RunOnce()`

- [ ] **Step 1: 写调度器失败合同**

```python
    def test_main_delegates_to_application(self):
        main = (ROOT / "empty.c").read_text(encoding="utf-8")
        self.assertIn("App_Main_Init();", main)
        self.assertIn("App_Main_RunOnce();", main)
        self.assertNotIn("LineWalking();", main)
```

Run: `python -m unittest tests.test_modular_architecture_contract -v`

Expected: FAIL，主循环仍直接调用循迹。

- [ ] **Step 2: 定义调度接口**

Create `application/app_scheduler.h`:

```c
#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H
#include <stdint.h>
typedef void (*AppTaskFn)(uint32_t now_ms);
typedef struct { uint16_t period_ms; uint32_t last_ms; AppTaskFn run; } AppTask;
void AppScheduler_Init(uint32_t now_ms);
void AppScheduler_Run(uint32_t now_ms);
#endif
```

`AppScheduler_Run` 使用无符号时间差；任务回调不得包含 `delay_ms()` 或外设等待循环。

- [ ] **Step 3: 建立安全主入口**

Create `application/app_main.c` with this call order:

```c
void App_Main_Init(void)
{
    Timer_Init();
    Motor_Safety_Init();
    Set_Motor(5);
    AppScheduler_Init(Get_Time());
}

void App_Main_RunOnce(void)
{
    AppScheduler_Run(Get_Time());
    Motor_Safety_Service();
}
```

本任务不调用 `Motor_Safety_Arm()`，保证模块化迁移版本上电零速。

- [ ] **Step 4: 精简 empty.c**

```c
int main(void)
{
    SYSCFG_DL_init();
    App_Main_Init();
    while (1) {
        App_Main_RunOnce();
    }
}
```

- [ ] **Step 5: 全量离线验证**

Run: `python -m unittest discover -s tests -v`

Expected: 所有测试 PASS；旧路径测试已同步到新目录；不存在直接 `Contrl_Speed()` 的应用调用。

Run: CCS Theia → Build Project。

Expected: TI Arm Clang 编译和链接成功；不手改 `Debug/ti_msp_dl_config.*`。

- [ ] **Step 6: 提交调度基线**

```bash
git add MSPM0G3507_LineFollowing_Car tests docs/setup/SETUP_GUIDE.md
git commit -m "refactor: add safe application scheduler"
```
