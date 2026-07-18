# Latest CCS Workspace Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用 `workspace_ccstheia` 中的最新版替换当前 CCS 工程，统一维护文本为UTF-8，并为L型520循迹程序加入软启动和200 ms失控锁定保护。

**Architecture:** 临时目录只作为只读来源，先建立双侧清单并备份现有工程，再复制最新版非生成文件。编码治理仅转换经严格检测确认的3个GBK头文件；电机安全逻辑封装在独立模块中，主循环负责正常服务，1 ms定时器负责超时锁存和有界紧急零速。

**Tech Stack:** TI MSPM0G3507、TI DriverLib、TI Arm Clang/CCS Theia、C11子集、Python 3 `unittest`、PowerShell、Git/GitHub CLI。

## Global Constraints

- 最新源码唯一来源：`workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang/`。
- 最终工程位置：`empty_LP_MSPM0G3507_nortos_ticlang/`。
- 不导入临时目录中的 `docs_backup/`、`.env`、IDE配置、下载记录或生成物。
- L型520使用 `Set_Motor(5)`；UART1保持 PB6 TX / PB7 RX。
- 上电先零速；1000 ms、10级软启动到目标30%；随后每100 ms最多再提高目标的10%，直到目标值。
- 200 ms无控制更新后锁存故障并持续零速；普通控制请求不能解锁。
- ISR紧急路径禁止 `sprintf`、动态内存和无界等待。
- 所有维护文本必须严格UTF-8可解码、无 `U+FFFD`、无已知中文乱码特征。
- 不进行12.6V带载测试；烧录前断开电机电源或悬空车轮。
- 临时来源只有在整合提交已推送、验证完成且用户确认后才能永久删除。

---

### Task 1: Freeze Both Source Trees and Create Recovery Data

**Files:**
- Create: `_integration_staging/source-manifest.csv`
- Create: `_integration_staging/current-manifest.csv`
- Create: `_integration_staging/current-backup/`

**Interfaces:**
- Produces: 可追溯来源清单和失败恢复副本。

- [ ] **Step 1: Verify repository state and source paths**

Run:

```powershell
git status --short --branch
$source = (Resolve-Path -LiteralPath 'workspace_ccstheia\empty_LP_MSPM0G3507_nortos_ticlang').Path
$target = (Resolve-Path -LiteralPath 'empty_LP_MSPM0G3507_nortos_ticlang').Path
if ($source -eq $target) { throw 'Source and target resolve to the same path' }
```

Expected: both paths exist and differ. Existing unrelated curated `docs/` changes are recorded but not discarded.

- [ ] **Step 2: Create SHA-256 manifests excluding Debug**

For each tree, enumerate non-`Debug/` files and export `RelativePath,Length,SHA256` to the corresponding CSV.

Expected: source and target manifests contain the same relative-path set; 19 hashes differ.

- [ ] **Step 3: Create a recovery copy of the current target**

```powershell
New-Item -ItemType Directory -Force -Path '_integration_staging\current-backup' | Out-Null
Get-ChildItem -LiteralPath 'empty_LP_MSPM0G3507_nortos_ticlang' -Force | Copy-Item -Destination '_integration_staging\current-backup' -Recurse -Force
```

Expected: recovery copy contains `.project`, `.cproject`, `empty.c`, `empty.syscfg` and `BSP/`.

---

### Task 2: Replace the Current CCS Project with the Latest Files

**Files:**
- Replace: `empty_LP_MSPM0G3507_nortos_ticlang/**` except `Debug/`.

**Interfaces:**
- Consumes: source manifest from Task 1.
- Produces: target tree matching the latest source before safety changes.

- [ ] **Step 1: Copy latest non-generated files by relative path**

For every source-manifest row, create the target parent directory and use `Copy-Item -LiteralPath` to overwrite the same relative path.

Expected: every target file hash equals its source-manifest hash.

- [ ] **Step 2: Verify CCS metadata immediately**

```powershell
[xml](Get-Content -Raw -LiteralPath 'empty_LP_MSPM0G3507_nortos_ticlang\.project') | Out-Null
[xml](Get-Content -Raw -LiteralPath 'empty_LP_MSPM0G3507_nortos_ticlang\.cproject') | Out-Null
```

Expected: both XML files parse; `empty.syscfg` and all `${PROJECT_ROOT}/BSP/...` include directories exist.

- [ ] **Step 3: Confirm latest entry behavior is present before patching**

```powershell
rg -n 'LineWalking|Set_Motor\(1\)|motor_init_count' 'empty_LP_MSPM0G3507_nortos_ticlang\empty.c'
```

Expected: all three patterns are found. This proves the latest entry was copied and also establishes the failing safety condition.

---

### Task 3: Convert Maintained Text to UTF-8 Without Double-Encoding

**Files:**
- Convert: `BSP/eMPL/inv_mpu.h`
- Convert: `BSP/Motor/app_motor_usart.h`
- Convert: `BSP/MPU6050/app_mpu6050.h`
- Verify: every maintained text file in the CCS project.

**Interfaces:**
- Produces: strict UTF-8 source tree for CCS display.

- [ ] **Step 1: Write a strict encoding audit script**

Create `_integration_staging/audit-encoding.ps1` that:

- scans `.c,.h,.md,.txt,.syscfg,.project,.cproject,.ccsproject,.json,.xml,.prefs` outside `Debug/`;
- decodes using `new UTF8Encoding(false, true)`;
- reports non-UTF8 paths;
- reports `U+FFFD` and known mojibake markers;
- exits nonzero on any failure.

- [ ] **Step 2: Run audit and verify the known failure set**

Expected: exactly the three approved GBK headers fail strictUTF-8; no other file is silently converted.

- [ ] **Step 3: Convert only the three approved files**

Read each as code page936 and write using `new UTF8Encoding(false)`. Before writing, verify the decoded text contains expected identifiers (`motor_type_t`, `mpu_init`/header guard, or InvenSense declarations) and no replacement character.

- [ ] **Step 4: Run the audit again**

Expected: `0` non-UTF8 files, `0` replacement characters, `0` mojibake markers.

---

### Task 4: Add Failing Safety Contract Tests

**Files:**
- Create: `tests/test_motor_safety_contract.py`

**Interfaces:**
- Produces: executable contract for source structure and safety constants.

- [ ] **Step 1: Write tests before implementation**

Tests must assert:

- `motor_safety.c/.h` exist;
- constants are `1000`, `10`, `30`, `200`, and post-ramp `10` percent per100 ms;
- `empty.c` contains `Set_Motor(5)`, `Motor_Safety_Init`, `Motor_Safety_Arm`, `Motor_Safety_Service`;
- `empty.c` contains no `Set_Motor(1)` or `motor_init_count`;
- `timer.c` calls `Motor_Safety_Tick1ms`;
- `timer.c/.h` define `Timer_Init`, and `empty.c` calls it before `Motor_Safety_Arm`;
- `Motion_Car_Control` and `Motion_Yaw_Calc` route through `Motor_Safety_RequestSpeed`;
- emergency ISR routine contains the fixed `$spd:0,0,0,0#` frame and no `sprintf`.

- [ ] **Step 2: Run tests and confirm failure**

```powershell
python -m unittest tests.test_motor_safety_contract -v
```

Expected: FAIL because the safety module and patched calls do not yet exist.

---

### Task 5: Implement the Motor Safety Module

**Files:**
- Create: `BSP/Motor/motor_safety.h`
- Create: `BSP/Motor/motor_safety.c`
- Modify: `BSP/Motor/app_motor.c`
- Modify: `BSP/Motor/app_motor.h`

**Interfaces:**
- Produces: `Motor_Safety_Init`, `Motor_Safety_Arm`, `Motor_Safety_RequestSpeed`, `Motor_Safety_Service`, `Motor_Safety_Tick1ms`, `Motor_Safety_IsFaultLatched`.

- [ ] **Step 1: Define constants and state enum**

Use:

```c
#define MOTOR_SAFETY_RAMP_MS                 (1000U)
#define MOTOR_SAFETY_RAMP_STEPS              (10U)
#define MOTOR_SAFETY_INITIAL_PERCENT         (30U)
#define MOTOR_SAFETY_POST_RAMP_STEP_PERCENT  (10U)
#define MOTOR_SAFETY_WATCHDOG_MS              (200U)
```

States: `DISARMED`, `RAMPING`, `RUNNING`, `FAULT_LATCHED`.

- [ ] **Step 2: Implement target capture and heartbeat**

`Motor_Safety_RequestSpeed` clamps all channels to `[-1000,1000]`, stores targets and resets heartbeat age only when not fault-latched. A zero target may lower output immediately.

- [ ] **Step 3: Implement staged ramp and continued slew limiting**

During each100 ms ramp step, maximum output magnitude rises by3% of target, reaching30% at1000 ms. Afterward, maximum permitted magnitude rises by10% of target per100 ms until100%. Sign changes must pass through zero.

- [ ] **Step 4: Route motion functions through safety layer**

Replace direct non-emergency `Contrl_Speed` calls in `Motion_Car_Control` and `Motion_Yaw_Calc` with `Motor_Safety_RequestSpeed`.

---

### Task 6: Add the Bounded Emergency Stop and Timer Watchdog

**Files:**
- Modify: `BSP/Motor/bsp_motor_usart.c`
- Modify: `BSP/Motor/bsp_motor_usart.h`
- Modify: `BSP/Timer/timer.c`
- Modify: `BSP/Timer/timer.h`

**Interfaces:**
- Produces: `Motor_EmergencyStop_FromISR(void)` and timer-driven watchdog.

- [ ] **Step 1: Implement fixed-frame ISR send**

Use a `static const uint8_t` frame containing `$spd:0,0,0,0#`. Send each byte directly through DriverLib with a fixed busy-loop iteration cap; do not call `sprintf`, `strlen`, heap functions or the normal array sender.

- [ ] **Step 2: Latch timeout in `Motor_Safety_Tick1ms`**

Increment saturated heartbeat age. At200 ms, set `FAULT_LATCHED` before calling the emergency routine. Further ticks keep the fault latched.

- [ ] **Step 3: Call watchdog from 1 ms ISR**

Add `Motor_Safety_Tick1ms()` in the `DL_TIMER_IIDX_ZERO` case alongside the existing counter and buzzer service.

- [ ] **Step 4: Explicitly enable and start the timer**

Add `Timer_Init(void)` that clears the pending IRQ, enables `TIMER_0_INST_INT_IRQN`, and calls `DL_TimerG_startCounter(TIMER_0_INST)`. Declare it in `timer.h`. Do not assume `SYSCFG_DL_init()` starts the counter.

---

### Task 7: Patch the Latest Main Program

**Files:**
- Modify: `empty.c`

**Interfaces:**
- Consumes: safety module APIs.
- Produces: safe latest line-following entry point.

- [ ] **Step 1: Replace unsafe initialization**

Required order:

```c
SYSCFG_DL_init();
USART_Init();
Timer_Init();
Motor_Safety_Init();
Set_Motor(5);
Motor_Safety_Arm();
```

- [ ] **Step 2: Replace repeated motor reconfiguration loop**

Main loop must call only:

```c
LineWalking();
Motor_Safety_Service();
delay_ms(10);
```

Remove three retries, `motor_init_count`, and periodic `Set_Motor(1)`.

- [ ] **Step 3: Run safety tests**

```powershell
python -m unittest tests.test_motor_safety_contract -v
```

Expected: all tests PASS.

---

### Task 8: Verify Encoding, CCS Structure and Buildability

**Files:**
- Verify: final CCS project.

- [ ] **Step 1: Run strict encoding audit**

Expected: every maintained text file passes UTF-8 and mojibake checks.

- [ ] **Step 2: Run static safety searches**

```powershell
rg -n 'Set_Motor\(1\)|motor_init_count|Contrl_Pwm\([^0]' empty_LP_MSPM0G3507_nortos_ticlang
```

Expected: no unsafe main-path matches. Historical enum/config definitions are reviewed separately.

- [ ] **Step 3: Parse CCS XML and verify include/source directories**

Expected: `.project/.cproject` parse; `BSP/Motor/motor_safety.c` is discoverable by managed build; all include directories exist.

- [ ] **Step 4: Build in CCS Theia**

Run Clean Project then Build Project. Expected: no compiler/linker errors and a new ignored `Debug/*.out`. If the local TI toolchain path is unavailable, record the exact missing path and do not claim build success.

---

### Task 9: Commit, Push and Remove the Temporary Source

**Files:**
- Commit: latest project replacement, encoding conversion, tests and safety layer.
- Delete after confirmation: `workspace_ccstheia/` and `_integration_staging/`.

- [ ] **Step 1: Review Git scope**

Expected tracked changes contain the CCS project, tests and approved docs only; no `workspace_ccstheia/docs_backup`, `.env`, `Debug`, object files or tokens.

- [ ] **Step 2: Commit and push integration**

```powershell
git add tests empty_LP_MSPM0G3507_nortos_ticlang docs
git commit -m 'feat: integrate latest CCS project with motor safety'
git push origin main
```

Expected: private remote `main` contains the integration commit.

- [ ] **Step 3: Ask final deletion confirmation**

Report source/target hashes, encoding audit, safety tests and CCS build result. Obtain explicit confirmation before permanent deletion.

- [ ] **Step 4: Delete exact temporary directories**

Resolve and verify exact paths:

```text
D:\DevProject\MSPM0G3507__car\workspace_ccstheia
D:\DevProject\MSPM0G3507__car\_integration_staging
```

Delete only those paths, without globs. Report that `workspace_ccstheia` contained untracked raw archives and cannot be restored from Git.

- [ ] **Step 5: Final verification**

```powershell
git status --short --branch
gh repo view Reese0001/MSPM0G3507__car --json visibility,defaultBranchRef,url
```

Expected: clean `main...origin/main`, visibility `PRIVATE`, default branch `main`.
