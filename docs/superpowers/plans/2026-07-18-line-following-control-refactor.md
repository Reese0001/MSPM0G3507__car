# 循迹控制修复与 CCS 工程重命名实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 CCS 工程重命名为 `MSPM0G3507_LineFollowing_Car`，修复八路灰度循迹的确定性逻辑缺陷，并补齐可直接导入、构建和安全调试的项目说明。

**Architecture:** 保留 PA15/PA16/PA17 选通、PA18 读取和现有 UART 电机安全层。传感器位型先转换为对称加权误差，再由有界 PD 产生左右轮差速；丢线和特殊弯道使用非阻塞状态处理，不让普通循迹修正反转车轮。

**Tech Stack:** TI MSPM0G3507、TI Arm Clang 4.0.4、MSPM0 SDK 2.10.00.04、CCS Theia、SysConfig、C11、Python `unittest`。

## Global Constraints

- 新工程目录和 CCS 工程名统一为 `MSPM0G3507_LineFollowing_Car`。
- `empty.syscfg` 仍是外设配置唯一真实来源；本轮不改变 PA15/PA16/PA17/PA18 分配。
- 电机命令必须继续经过 `Motor_Safety_RequestSpeed()`；保留 0→30% soft-start 和 200 ms 看门狗。
- 普通循迹状态中 `abs(correction) < base_speed`，禁止由 PD 输出意外反转单侧车轮。
- 连续丢线先以不高于 30% 的速度短时恢复，超过限定周期必须请求零速。
- 所有维护文本使用 UTF-8；本计划只做离线测试、生成、编译和链接，不烧录、不接通 12.6 V 电机电源。

---

### Task 1: 提交修改前证据基线

**Files:**
- Create: `docs/notes/line-following-offline-audit.md`
- Create: `docs/notes/line-following-strategy-research.md`
- Create: `tests/test_line_following_contract.py`
- Create: `tests/test_sysconfig_contract.py`
- Create: `tests/test_tracking_decision_table.py`
- Create: `docs/superpowers/plans/2026-07-18-line-following-control-refactor.md`

**Interfaces:**
- Consumes: 已完成的离线审计和资料调研。
- Produces: 修改生产代码前可回退、可复核的独立提交。

- [ ] **Step 1: 验证文档编码和 Git 差异**

Run:

```powershell
python -m unittest tests.test_text_encoding -v
git diff --check
git status --short
```

Expected: 编码测试通过，`git diff --check` 无错误，待提交文件仅为计划、审计、调研和修改前失败合同。

- [ ] **Step 2: 记录修改前失败测试**

Run:

```powershell
python -m unittest tests.test_line_following_contract tests.test_sysconfig_contract tests.test_tracking_decision_table -v
```

Expected: SysConfig 和调用链合同通过；PID 历史更新、丢线停车和阻塞延时三项按预期失败。

- [ ] **Step 3: 创建基线提交**

```powershell
git add docs/notes docs/superpowers/plans tests
git commit -m "test: capture line-following audit baseline"
```

### Task 2: 重命名 CCS 工程并更新活动引用

**Files:**
- Move: `empty_LP_MSPM0G3507_nortos_ticlang/` → `MSPM0G3507_LineFollowing_Car/`
- Modify: `MSPM0G3507_LineFollowing_Car/.project`
- Modify: `MSPM0G3507_LineFollowing_Car/.cproject`
- Modify: `.theia/launch.json`
- Modify: `tests/test_motor_safety_contract.py`
- Modify: `tests/test_line_following_contract.py`
- Modify: `tests/test_sysconfig_contract.py`
- Modify: `tests/test_tracking_decision_table.py`
- Modify: `tests/test_text_encoding.py`
- Modify: `AGENTS.md`
- Modify: `CLAUDE.md`
- Modify: `docs/README.md`
- Modify: `docs/setup/SETUP_GUIDE.md`

**Interfaces:**
- Consumes: Eclipse/CCS `.project` 名称和 Theia launch `resourceId`。
- Produces: 可从新目录导入且内部活动工程名一致的 CCS 工程。

- [ ] **Step 1: 先修改路径合同并验证 RED**

在测试中把 `PROJECT` 改为：

```python
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"
```

Run: `python -m unittest discover -s tests -v`

Expected: 因新目录尚不存在而失败。

- [ ] **Step 2: 使用 Git 移动目录并更新工程元数据**

Run:

```powershell
git mv empty_LP_MSPM0G3507_nortos_ticlang MSPM0G3507_LineFollowing_Car
```

将 `.project` 的 `<name>`、`.cproject` 的活动 project id 前缀和 `.theia/launch.json` 的名称/`resourceId` 更新为 `MSPM0G3507_LineFollowing_Car`；保留 `.ccsproject` 的 SDK 示例来源信息作为 provenance，不伪造 TI 模板路径。

- [ ] **Step 3: 更新当前使用说明，不改写历史归档**

更新 AGENTS、CLAUDE、docs README、setup guide 和当前计划中的活动路径。`docs/superpowers/specs`、旧计划、handoff 和 build-history 中的旧名称属于历史证据，保留原文并注明已重命名。

- [ ] **Step 4: 验证重命名**

Run:

```powershell
python -m unittest tests.test_motor_safety_contract tests.test_sysconfig_contract tests.test_text_encoding -v
git diff --check
```

Expected: 路径、SysConfig、安全合同和编码测试通过。

- [ ] **Step 5: 提交工程重命名**

```powershell
git add -A
git commit -m "refactor: rename CCS line-following project"
```

### Task 3: 用对称加权误差替换脆弱位型逻辑

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.h`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.c`
- Modify: `tests/test_tracking_decision_table.py`

**Interfaces:**
- Produces: `Tracking_ComputeWeightedError(uint8_t sensor_bits, float *error)`，返回是否检测到有效黑线。

- [ ] **Step 1: 添加单路、相邻双路、中线和全白失败测试**

测试权重固定为 `{-7,-5,-3,-1,1,3,5,7}`；中心双路平均为 0，全白返回无有效线。

- [ ] **Step 2: 运行测试确认 RED**

Run: `python -m unittest tests.test_tracking_decision_table -v`

Expected: 因纯决策函数尚不存在而失败。

- [ ] **Step 3: 实现最小加权函数并接入采样结果**

遍历八位输入，对黑线通道求权重平均；不访问 GPIO、不发送电机命令。黑白极性在 `Gray_ReadChannel()` 边界统一转换。

- [ ] **Step 4: 运行测试确认 GREEN 并提交**

```powershell
python -m unittest tests.test_tracking_decision_table -v
git add MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking tests/test_tracking_decision_table.py
git commit -m "feat: compute weighted tracking error"
```

### Task 4: 修复 PD、限幅和丢线非阻塞状态

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.c`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.h`
- Modify: `tests/test_line_following_contract.py`
- Modify: `tests/test_tracking_decision_table.py`

**Interfaces:**
- Produces: 每周期更新 `last_error` 的 PD；`TRACKING/SEARCHING/LINE_LOST` 状态；有界左右轮目标速度。

- [ ] **Step 1: 保持现有三项测试为 RED**

Run: `python -m unittest tests.test_line_following_contract tests.test_tracking_decision_table -v`

Expected: 缺少 `error_last = error`、全白零速和存在 `delay_ms(100)` 三项失败。

- [ ] **Step 2: 最小修复 PD 状态更新**

在计算本周期输出后执行 `error_last = error`；第一阶段令 `Ki=0`，积分状态在丢线/停车时清零。

- [ ] **Step 3: 实现有界修正和连续丢线处理**

正常循迹对修正量限幅到基础速度的 80%；连续丢线前两周期以不高于安全层 30% 的速度沿最后有效方向恢复，第三周期请求 `Motion_Car_Control(0, 0, 0)` 并进入 `LINE_LOST`。

- [ ] **Step 4: 移除循迹路径的 100 ms 阻塞延时**

以周期计数表达直角/恢复持续时间；每次 `LineWalking()` 只执行一次采样和一次决策，立即返回主循环。

- [ ] **Step 5: 验证 GREEN 并提交**

```powershell
python -m unittest tests.test_line_following_contract tests.test_tracking_decision_table -v
git add MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking tests
git commit -m "fix: make line-following control bounded and nonblocking"
```

### Task 5: 编写项目说明并完成离线构建验证

**Files:**
- Create: `README.md`
- Modify: `MSPM0G3507_LineFollowing_Car/README.md`
- Modify: `docs/notes/line-following-offline-audit.md`
- Modify if reusable guidance changed: `PROJECT_SKILLS.md`

**Interfaces:**
- Produces: 仓库入口说明、CCS 导入/构建步骤、接线表、安全上电顺序、算法说明和验证边界。

- [ ] **Step 1: 编写仓库和工程两级说明**

说明必须包含新工程名、PA15～PA18 灰度接口、PB6/PB7 电机 UART、115200 调试串口、SysConfig 唯一来源、soft-start、看门狗、离线构建和实车测试 Checklist。

- [ ] **Step 2: 运行完整测试和编码检查**

Run:

```powershell
python -m unittest discover -s tests -v
git diff --check
```

Expected: 全部测试通过，文本无编码或空白错误。

- [ ] **Step 3: SysConfig 生成和 TI Arm Clang 全量编译链接**

按离线审计中已验证的命令重新生成配置、编译全部生产 C 源并链接；输出文件名使用 `MSPM0G3507_LineFollowing_Car.out`。

Expected: SysConfig、编译和链接均返回 0；符号表含 `main`、`LineWalking`、`Motor_Safety_Service`。

- [ ] **Step 4: 更新审计报告并提交**

```powershell
git add README.md MSPM0G3507_LineFollowing_Car/README.md docs/notes PROJECT_SKILLS.md
git commit -m "docs: explain line-following car project"
```

## Self-Review

- 覆盖了用户要求的先计划、先提交基线、再修改代码、工程重命名和项目说明。
- 生产代码修改均安排了先失败后通过的测试步骤。
- 重命名只更新活动引用，历史资料继续保留旧名称以维持证据真实性。
- 电机安全层、软启动、看门狗、零速丢线和不上电边界均有明确验证门槛。
