# 循迹小车可运行性验证计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 验证当前 MSPM0G3507 工程能否完成“八路灰度读取 → 方向误差计算 → 安全速度请求 → UART1 电机驱动”的完整循迹闭环，并在确认硬件安全后完成架空轮测试。

**Architecture:** 先建立完全离线的构建与静态数据流证据，再检查 SysConfig 与代码的外设配置是否一致。只有离线门槛全部通过，且用户确认接线与架空轮安全条件后，才进入板端零速、低速和循迹测试；任何修复都先添加能复现问题的失败测试。

**Tech Stack:** TI MSPM0G3507、TI Arm Clang 4.0.4、MSPM0 SDK 2.10.00.04、CCS Theia、SysConfig、Python `unittest`、裸机 1 ms 定时器。

## Global Constraints

- 主工程固定为 `empty_LP_MSPM0G3507_nortos_ticlang/`，历史参考工程不得参与构建。
- `empty.syscfg` 是引脚和外设配置的唯一真实来源；不得手改 `Debug/ti_msp_dl_config.c`。
- 电机型号固定为 L 型 520，即 `Set_Motor(5)`；M2/M4 为驱动轮，M1/M3 必须保持零速。
- 电机启动必须经 `Motor_Safety_RequestSpeed()`，1000 ms 内由 0 逐级提升到 30%，200 ms 无新请求必须锁存故障并发送固定零速帧。
- 未完成接线确认前不得烧录；未断开 12.6 V 电机电源或架空驱动轮前不得发送非零速度。
- 所有受维护文本必须是 UTF-8；不得重新引入 GBK/ANSI 文件。

---

### Task 1: 冻结检查基线与安全边界

**Files:**
- Read: `AGENTS.md`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/empty.c`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/empty.syscfg`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/BSP/Motor/motor_safety.c`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/BSP/Eight_Tracking/app_irtracking.c`
- Create: `_integration_staging/line-following-audit/baseline.txt`

**Interfaces:**
- Consumes: 当前 `main` 初始化顺序、SysConfig 配置和 Git 提交状态。
- Produces: 可复核的提交号、工具链版本、待验证文件哈希和“本轮不烧录/不带载”的安全声明。

- [ ] **Step 1: 记录 Git 与工具链基线**

Run:

```powershell
git status --short --branch
git rev-parse HEAD
& 'D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmclang.exe' --version
```

Expected: `main` 无未说明改动；编译器报告 TI Arm Clang 4.0.4。

- [ ] **Step 2: 记录核心文件哈希**

Run:

```powershell
Get-FileHash empty_LP_MSPM0G3507_nortos_ticlang\empty.c
Get-FileHash empty_LP_MSPM0G3507_nortos_ticlang\empty.syscfg
Get-FileHash empty_LP_MSPM0G3507_nortos_ticlang\BSP\Motor\motor_safety.c
Get-FileHash empty_LP_MSPM0G3507_nortos_ticlang\BSP\Eight_Tracking\app_irtracking.c
```

Expected: 四个文件均存在并产生 SHA-256。

- [ ] **Step 3: 明确本阶段安全限制**

记录：只执行读取、测试和离线编译；不连接 12.6 V 电机电源，不烧录，不发送非零电机帧。

### Task 2: CCS 全工程编译与链接验证

**Files:**
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/.cproject`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/Debug/makefile`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/Debug/subdir_vars.mk`
- Verify: `empty_LP_MSPM0G3507_nortos_ticlang/BSP/Motor/motor_safety.c`

**Interfaces:**
- Consumes: TI Arm Clang 4.0.4、SDK 2.10.00.04 和 SysConfig 生成物。
- Produces: 完整 `.out` 链接结果，且必须包含 `motor_safety.o`。

- [ ] **Step 1: 先运行现有自动测试**

Run:

```powershell
python -m unittest discover -s tests -v
```

Expected: 现有 6 个测试全部通过。

- [ ] **Step 2: 检查 CCS 构建清单是否包含安全模块**

Run:

```powershell
Select-String -Path empty_LP_MSPM0G3507_nortos_ticlang\Debug\BSP\Motor\subdir_vars.mk -Pattern 'motor_safety.c'
```

Expected: 找到 `motor_safety.c`。若未找到，说明 `Debug/` 是旧生成物，必须在 CCS Theia 中执行 **Project → Clean** 后 **Build Project**，让 managed build 重新生成 makefile；不得手改生成文件。

- [ ] **Step 3: 在 CCS Theia 执行 Clean + Build**

操作：导入 `empty_LP_MSPM0G3507_nortos_ticlang`，选择 **Project → Clean**，再右键项目 **Build Project**。

Expected: 0 errors；构建日志出现 `motor_safety.c`、`app_irtracking.c`、`app_motor.c`、`timer.c` 的编译命令，并生成 `Debug/empty_LP_MSPM0G3507_nortos_ticlang.out`。

- [ ] **Step 4: 若构建失败，只记录首个根因**

记录首个 error 的完整命令、文件和行号；不同时修改多个问题。为该错误添加最小失败测试后，再编写单独修复计划。

### Task 3: 循迹闭环静态合同检查

**Files:**
- Create: `tests/test_line_following_contract.py`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/empty.c`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/BSP/Eight_Tracking/app_irtracking.c`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/BSP/Motor/app_motor.c`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/BSP/Motor/motor_safety.c`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/BSP/Timer/timer.c`

**Interfaces:**
- Consumes: `LineWalking()`、`Motion_Car_Control()`、`Motor_Safety_RequestSpeed()`、`Motor_Safety_Service()`、`Motor_Safety_Tick1ms()`。
- Produces: 自动化合同，证明主循环和安全层没有被绕过。

- [ ] **Step 1: 编写失败测试**

测试必须断言：

```python
self.assertIn("LineWalking();", main)
self.assertIn("Motor_Safety_Service();", main)
self.assertIn("Motion_Car_Control", tracking)
self.assertIn("Motor_Safety_RequestSpeed", motor)
self.assertNotIn("Contrl_Speed(", tracking)
self.assertIn("Motor_Safety_Tick1ms();", timer)
self.assertIn("Set_Motor(5);", main)
```

并解析 `Motion_Car_Control()`，确认 M1/M3 固定为 0，M2/M4 承担左右轮速度。

- [ ] **Step 2: 运行测试并观察结果**

Run:

```powershell
python -m unittest tests.test_line_following_contract -v
```

Expected: 若当前闭环完整则 PASS；若 FAIL，失败信息必须直接指出断开的调用边界。

- [ ] **Step 3: 对失败项建立单一修复任务**

每次只修一个调用边界；先保留失败测试，再修改对应 C 文件，直至测试转绿。涉及电机 UART、引脚或速度上限的修复必须先向用户提交安全 Checklist。

### Task 4: SysConfig 与硬件分配一致性检查

**Files:**
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/empty.syscfg`
- Read: `docs/setup/SETUP_GUIDE.md`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/Debug/ti_msp_dl_config.h`
- Create: `tests/test_sysconfig_contract.py`

**Interfaces:**
- Consumes: UART1 PB6/PB7、I2C1 PA15/PA16、Timer 1 ms、UART0 PA10/PA11 的约定。
- Produces: 代码配置与已确认接线之间的差异表。

- [ ] **Step 1: 添加 SysConfig 合同测试**

测试必须核对：

```text
UART1 motor: PB6 TX / PB7 RX
I2C1 tracking: PA15 SCL / PA16 SDA
tracking address: 0x12
timer period: 1 ms
debug UART0: 115200 baud
```

- [ ] **Step 2: 运行合同测试**

Run:

```powershell
python -m unittest tests.test_sysconfig_contract -v
```

Expected: 所有已确认配置 PASS。任何引脚差异先报告，不直接修改 `empty.syscfg`。

- [ ] **Step 3: 用户确认实物接线**

用户需逐项确认：扩展板型号、UART1 TX/RX 是否交叉、MCU 与电机板是否共地、灰度模块地址是否为 0x12、PA15/PA16 是否带正确上拉。未确认不得进入板端测试。

### Task 5: 循迹算法输入输出离线验证

**Files:**
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/BSP/Eight_Tracking/app_irtracking.c`
- Read: `empty_LP_MSPM0G3507_nortos_ticlang/BSP/Eight_Tracking/app_irtracking.h`
- Create: `tests/test_tracking_decision_table.py`

**Interfaces:**
- Consumes: 八路二值输入 `x1..x8`、循迹误差 `err`、PID 输出和基础速度。
- Produces: 中线、左偏、右偏、全白/全黑、直角五类输入的期望方向表。

- [ ] **Step 1: 从当前 C 代码提取实际判断顺序**

建立表格：输入位型、命中分支、`err` 符号、左右轮期望关系。至少覆盖：

```text
中线：左右轮近似相等
左偏：右轮速度高于左轮或产生正确左转角速度
右偏：左轮速度高于右轮或产生正确右转角速度
全黑：按交叉线/特殊状态处理
全白：不得无限沿用危险高速命令
```

- [ ] **Step 2: 写入决策表测试并先运行**

Run:

```powershell
python -m unittest tests.test_tracking_decision_table -v
```

Expected: 测试反映当前实际逻辑；方向符号冲突或全白无安全处理时 FAIL。

- [ ] **Step 3: 仅在失败证据明确后规划算法修复**

若需要重构，将纯决策逻辑提取为不访问硬件的函数，硬件读取仍保留在 `LineWalking()`；先测试后实现，不在本验证计划中猜测 PID 数值。

### Task 6: 板端零速与架空轮分阶段测试

**Files:**
- Verify: `empty_LP_MSPM0G3507_nortos_ticlang/Debug/empty_LP_MSPM0G3507_nortos_ticlang.out`
- Record: `docs/notes/line-following-bringup-log.md`

**Interfaces:**
- Consumes: 已成功链接的固件和用户确认的接线。
- Produces: 传感器、左右轮方向、看门狗、软启动的实测记录。

- [ ] **Step 1: 烧录前 Checklist**

必须全部确认：

```text
[ ] 首次烧录时断开 12.6 V 电机电源
[ ] MCU、灰度模块、电机驱动板共地
[ ] UART1 PB6/PB7 接线与电机板 TX/RX 对应正确
[ ] 八路灰度 PA15/PA16 与地址 0x12 已确认
[ ] 驱动轮架空，车辆不会冲出桌面
[ ] 可立即断电，周围无人手持车轮
```

- [ ] **Step 2: 仅 MCU/传感器上电**

观察八路灰度原始值，依次用白底、黑线覆盖各探头；记录每一路极性和顺序。Expected: X1 到 X8 与实物从左到右顺序一致，黑/白极性与代码判断一致。

- [ ] **Step 3: 架空轮低速方向测试**

接通电机电源，保持驱动轮架空；只允许经安全层从 0 缓升，验证 M2 左轮、M4 右轮的正方向。Expected: M1/M3 不动，左右驱动轮前进方向一致，无瞬间满速。

- [ ] **Step 4: 看门狗停机测试**

人为停止刷新速度请求超过 200 ms。Expected: 电机收到固定零速帧并保持故障锁存，恢复请求不能自行重新启动，必须重新初始化。

- [ ] **Step 5: 低速短赛道测试**

在封闭短赛道以不高于 30% 初始限制测试中线、缓弯、直角和丢线；记录传感器位型、误差、左右轮命令。PID 调整必须依据日志，每次只改一个参数。

### Task 7: 形成结论与提交证据

**Files:**
- Create: `docs/notes/line-following-verification-report.md`
- Modify only if relevant: `PROJECT_SKILLS.md`

**Interfaces:**
- Consumes: 构建日志、合同测试、SysConfig 差异表、板端测试记录。
- Produces: `可编译`、`可烧录`、`可架空运行`、`可落地循迹` 四级结论，禁止把前一级结果冒充后一级。

- [ ] **Step 1: 运行最终离线验证**

Run:

```powershell
python -m unittest discover -s tests -v
git status --short --branch
```

Expected: 全部测试 PASS，改动范围与报告一致。

- [ ] **Step 2: 写验证报告**

报告必须逐项列出：编译/链接结果、调用链、引脚配置、未验证硬件项、实际测试结果、剩余风险，不使用“应该能跑”替代证据。

- [ ] **Step 3: 用户批准后提交并推送**

```powershell
git add tests docs/notes PROJECT_SKILLS.md
git commit -m "verify line-following car control path"
git push origin main
```

Expected: 本地与远端 `main` 提交哈希一致。

## Self-Review

- 覆盖了 CCS 编译与链接、主循环调用链、SysConfig/接线、循迹决策、软启动、200 ms 看门狗和分阶段上车测试。
- 所有关键硬件变更都设置了用户确认门槛，没有假设未确认的扩展板细节。
- 未把离线语法检查等同于完整链接，也未把架空轮成功等同于赛道循迹成功。
- 计划中无占位项；算法参数只允许在取得实测日志后单项调整。
