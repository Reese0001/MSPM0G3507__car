# Sensor Platform Worktree Roadmap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 按独立 Git 工作树交付模块化基础、三个新传感器、自适应循迹和最终安全集成。

**Architecture:** 先建立无行为变化的公共接口和目录基线，再从同一提交派生超声波、YbImu、K230 和循迹控制工作树。最后在唯一集成工作树合并模块并修改 `empty.syscfg`，所有运动请求通过 SafetySupervisor 和 MotorSafety。

**Tech Stack:** MSPM0G3507、TI DriverLib、TI Arm Clang 4.0.4、CCS Theia、SysConfig、C11、Python unittest、CanMV/K230 MicroPython。

## Global Constraints

- 开始执行前必须使用 `using-git-worktrees` 技能创建隔离工作树。
- 电机首次启动必须 0→30% soft-start，保留 200ms watchdog，禁止直接 100% 输出。
- `empty.syscfg` 是引脚和外设配置的唯一真实来源，只有最终集成分支永久修改它。
- C 文件保持原编码；新增 Markdown、Python 和新建 C 文件统一 UTF-8。
- YbImu 使用 PA12=SCL、PA13=SDA、地址 `0x23`、100kHz。
- HC-SR04 使用 PA26=Trig、PA27=Echo，Echo 进入 MCU 前必须降到 3.3V。
- K230 使用 PA21 TX→IO10/RX、PA22 RX←IO9/TX、115200。
- 12.6V 满电电池不得在未通过输入范围与温升验收时直接长期供给标称 5～12V 的扩展板。

---

### Task 1: 建立共同基线

**Files:**
- Follow: `docs/superpowers/plans/2026-07-22-modular-foundation.md`

**Interfaces:**
- Consumes: commit `8cfd480`
- Produces: `codex/modular-foundation` 的审核提交，包含 `ModuleStatus`、`AppScheduler`、新目录和保持零速的主循环

- [ ] **Step 1: 创建并执行基础工作树计划**

Run: 按 modular-foundation 计划逐项执行。

Expected: `python -m unittest discover -s tests -v` 全部通过；CCS Build Project 成功；电机保持零速。

- [ ] **Step 2: 记录基线提交**

```powershell
git rev-parse codex/modular-foundation^{commit}
```

Expected: 输出一个 40 位提交 ID；把该值原样记录到执行日志，后续四个模块工作树都从这个提交创建。

### Task 2: 并行开发可独立模块

**Files:**
- Follow: `docs/superpowers/plans/2026-07-22-ultrasonic-module.md`
- Follow: `docs/superpowers/plans/2026-07-22-ybimu-module.md`
- Follow: `docs/superpowers/plans/2026-07-22-k230-link.md`
- Follow: `docs/superpowers/plans/2026-07-22-adaptive-line-control.md`

**Interfaces:**
- Consumes: `FOUNDATION_COMMIT`
- Produces: 四个互不依赖的审核分支；每个分支有离线测试和模块级验收记录

- [ ] **Step 1: 从共同基线创建四个工作树**

```text
codex/ultrasonic
codex/ybimu
codex/k230-link
codex/adaptive-line-control
```

Run: 分别执行 `git merge-base codex/ultrasonic codex/modular-foundation`、`git merge-base codex/ybimu codex/modular-foundation`、`git merge-base codex/k230-link codex/modular-foundation`、`git merge-base codex/adaptive-line-control codex/modular-foundation`。

Expected: 四条命令都输出 Task 1 记录的同一个 40 位提交 ID。

- [ ] **Step 2: 分别执行模块计划**

Run: 每个工作树只执行自己的计划，不跨工作树复制未审核提交。

Expected: 每个分支都能单独通过 Python 合同测试；模块禁用时主工程仍保持安全零速。

### Task 3: 最终安全集成

**Files:**
- Follow: `docs/superpowers/plans/2026-07-22-sensor-safety-integration.md`

**Interfaces:**
- Consumes: 基础分支与四个模块分支的审核提交
- Produces: `codex/sensor-safety-integration`，唯一的整车 SysConfig、接线表和集成验证证据

- [ ] **Step 1: 按固定顺序合并**

```text
modular-foundation
→ ultrasonic
→ ybimu
→ k230-link
→ adaptive-line-control
→ safety supervisor / syscfg / board validation
```

Expected: 每次合并后运行全套离线测试；若失败，在引入失败的合并点修复，不把多个失败叠加。

- [ ] **Step 2: 执行硬件分级验收**

Run: 断开电机动力 → 单模块通电 → 全逻辑联调 → 架空轮 → 30% 以下落地测试。

Expected: 每一级都有日志；未通过前一级不得进入下一级。
