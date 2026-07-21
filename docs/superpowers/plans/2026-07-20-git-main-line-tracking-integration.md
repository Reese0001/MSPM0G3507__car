# Git Main 与循迹修复集成计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 审核并提交当前八路循迹修复，在不带入无关文件的前提下合并回本地 `main`，完成离线验证后再同步 GitHub。

**Architecture:** 当前 `fix/repair-line-tracking-contract` 与本地 `main` 都位于 `1924421`，循迹修复仍存在于工作区。先把修复和对应验证固化为独立提交，再以 fast-forward 方式更新 `main`；编码历史问题另开提交处理，避免功能修复与大范围文本变更耦合。

**Tech Stack:** MSPM0G3507、TI DriverLib、TI Arm Clang 4.0.4、CCS Theia、Python `unittest`、Git/GitHub。

## Global Constraints

- `empty.syscfg` 是引脚配置的唯一真实来源；PA15/PA16/PA17 选择通道，PA18 读取灰度 OUT。
- 所有电机命令必须经过 `Motor_Safety_RequestSpeed()`，保留 0→30% soft-start 和 200 ms watchdog。
- 首次实车验证必须架空驱动轮、使用低速并准备断开 12.6V 电机电源。
- 不跟踪、不提交当前无关的 `my-project/`。
- 不把生成的 `Debug/` 构建产物加入源码提交。

---

### Task 1: 固化 Git 审核基线

**Files:**
- Inspect: `.git/` branch and commit metadata
- Inspect: `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.c`
- Inspect: `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.h`

**Interfaces:**
- Consumes: `main`、`origin/main`、`fix/repair-line-tracking-contract`
- Produces: 可审计的分支拓扑和精确暂存范围

- [ ] **Step 1: 确认分支拓扑**

Run: `git log --graph --decorate --oneline --all -n 30`

Expected: `fix/repair-line-tracking-contract` 与 `main` 起点均为 `1924421`，`origin/main` 为其祖先 `be2f771`。

- [ ] **Step 2: 确认工作区范围**

Run: `git status --short --branch`

Expected: 仅两个循迹源文件、本文档发生修改；`my-project/` 保持未跟踪。

- [ ] **Step 3: 检查补丁格式**

Run: `git diff --check`

Expected: exit code 0，无空白错误；CRLF 提示不视为失败。

### Task 2: 验证循迹控制契约

**Files:**
- Modify only if a test exposes a defect: `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.c`
- Modify only if API changes: `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.h`
- Test: `tests/test_tracking_decision_table.py`
- Test: `tests/test_line_following_contract.py`
- Test: `tests/test_motor_safety_contract.py`

**Interfaces:**
- Consumes: `Tracking_ComputeWeightedError(uint8_t, float *)`、`Motion_Car_Control(int16_t, int16_t, int16_t)`
- Produces: 对称加权误差、实车转向极性、丢线停止、非阻塞控制循环

- [ ] **Step 1: 执行全部离线契约测试**

Run: `python -m unittest discover -s tests -v`

Expected: 22 tests，全部 `OK`。

- [ ] **Step 2: 检查安全调用路径**

Run: `rg -n "Motion_Car_Control|Motor_Safety_RequestSpeed|Contrl_Speed" MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking MSPM0G3507_LineFollowing_Car/BSP/Motor`

Expected: 循迹层只调用 `Motion_Car_Control()`；最终速度由 `Motor_Safety_RequestSpeed()` 接管；循迹层不得直接调用 `Contrl_Speed()`。

- [ ] **Step 3: 编译 CCS Debug 工程**

Run from `MSPM0G3507_LineFollowing_Car/Debug`: `D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe -j4 all`

Expected: exit code 0，生成或确认 `MSPM0G3507_LineFollowing_Car.out` 为最新。

### Task 3: 提交修复分支

**Files:**
- Stage: `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.c`
- Stage: `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.h`
- Stage: `docs/superpowers/plans/2026-07-20-git-main-line-tracking-integration.md`

**Interfaces:**
- Consumes: Task 1 和 Task 2 的通过结果
- Produces: 单一、可回滚的循迹修复提交

- [ ] **Step 1: 精确暂存文件**

Run: `git add MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.c MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.h docs/superpowers/plans/2026-07-20-git-main-line-tracking-integration.md`

Expected: `my-project/` 不在暂存区。

- [ ] **Step 2: 审核暂存内容**

Run: `git diff --cached --stat`

Expected: 仅上述三个文件。

- [ ] **Step 3: 创建提交**

Run: `git commit -m "fix: restore safe weighted line tracking"`

Expected: 当前分支比 `main` 领先 1 个提交。

### Task 4: 合并到本地 main 并回归验证

**Files:**
- Update: Git branch `main`

**Interfaces:**
- Consumes: Task 3 的修复提交
- Produces: 与修复分支同一提交的本地 `main`

- [ ] **Step 1: 切换 main**

Run: `git switch main`

Expected: 工作区干净，`my-project/` 仍为未跟踪且未修改。

- [ ] **Step 2: 仅允许 fast-forward 合并**

Run: `git merge --ff-only fix/repair-line-tracking-contract`

Expected: Fast-forward 成功，不产生额外 merge commit。

- [ ] **Step 3: 再跑完整测试与编译**

Run: `python -m unittest discover -s tests -v`

Run from `MSPM0G3507_LineFollowing_Car/Debug`: `D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe -j4 all`

Expected: 22 tests 全部通过，构建 exit code 0。

### Task 5: 推送 GitHub 与实车验证门槛

**Files:**
- Update: remote branch `origin/main`

**Interfaces:**
- Consumes: 已验证的本地 `main`
- Produces: GitHub 上可查看的完整提交历史

- [ ] **Step 1: 推送 main**

Run: `git push origin main`

Expected: `origin/main` 快进到本地 `main`，包含原先领先的 5 个提交和本次修复提交。

- [ ] **Step 2: 检查远端同步状态**

Run: `git rev-list --left-right --count origin/main...main`

Expected: `0 0`。

- [ ] **Step 3: 执行低风险实车验证**

将驱动轮架空后烧录最新固件；确认 X1→X8 的物理左右顺序、黑线电平为 0、左侧检测触发实际左转、右侧检测触发实际右转、连续丢线后停车。任何方向不符时只记录串口传感器序列并断开电机电源，不继续高速测试。

### Task 6: 单独处理中文乱码债务

**Files:**
- Audit: `MSPM0G3507_LineFollowing_Car/**/*.c`
- Audit: `MSPM0G3507_LineFollowing_Car/**/*.h`
- Audit: `openspec/specs/**/*.md`
- Modify: `tests/test_text_encoding.py`

**Interfaces:**
- Consumes: 当前合法 UTF-8 但语义损坏的中文文本
- Produces: 独立编码修复提交，不改变固件行为

- [ ] **Step 1: 建立乱码特征检查**

在 `tests/test_text_encoding.py` 中增加对常见 mojibake 片段（例如 `鍨`、`鐢`、`寰`、`锛`）的扫描，并为明确允许的历史归档建立最小白名单。

- [ ] **Step 2: 先运行测试确认能暴露现状**

Run: `python -m unittest tests.test_text_encoding -v`

Expected: FAIL，并列出维护目录中语义损坏的文本文件。

- [ ] **Step 3: 按原始语义逐文件修复并复测**

Run: `python -m unittest discover -s tests -v`

Expected: 全部通过；随后单独提交为 `fix: repair maintained UTF-8 text`。

