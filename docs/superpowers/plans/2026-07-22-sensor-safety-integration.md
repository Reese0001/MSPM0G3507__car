# Sensor Safety Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在唯一集成工作树中合并所有模块、配置最终 SysConfig、实现 SafetySupervisor，并完成从断电检查到低速整车的分级验收。

**Architecture:** 集成分支按固定顺序引入已审核模块，任何冲突先保持电机零速。SafetySupervisor 消费模块快照和任务 MotionRequest，按 watchdog、障碍、关键传感器、任务请求的优先级生成唯一批准请求，再由 motor adapter 转给现有 MotorSafety。

**Tech Stack:** MSPM0G3507、TI DriverLib/SysConfig、TI Arm Clang、CCS Theia、Python unittest、UART telemetry、万用表/示波器。

## Global Constraints

- 分支：`codex/sensor-safety-integration`，基于 modular-foundation 审核提交。
- 合并顺序：ultrasonic → ybimu → k230-link → adaptive-line-control。
- 只有本分支永久修改 `empty.syscfg`；禁止手改 `Debug/ti_msp_dl_config.*`。
- SafetySupervisor 是唯一批准非零运动请求的应用组件；最终仍进入 MotorSafety。
- 电源门槛和电机 Checklist 未通过时，固件保持 `BOOT_SAFE`。
- 所有硬件验收必须分级，不能把编译成功写成实车成功。

---

## File Structure

- Create: `application/safety_supervisor.c/.h` — 状态、优先级、锁存和批准请求。
- Create: `application/mission_manager.c/.h` — 路线基元编排。
- Create: `application/config/safety_config.h`, `vehicle_config.h`。
- Create: `modules/motor/motor_adapter.c/.h` — 两轮请求到 M2/M4 的唯一适配。
- Create: `application/telemetry.c/.h` — 限频遥测和停机原因。
- Modify: `application/app_main.c`, `app_scheduler.c`, `empty.syscfg`, `.cproject`。
- Create: `tests/test_safety_supervisor.py`, `tests/test_final_pin_contract.py`, `tests/test_motor_authority.py`。
- Create: `docs/hardware/final-wiring.md`, `docs/hardware/power-acceptance.md`, `docs/verification/sensor-platform-test-record.md`。

### Task 1: 合并已审核模块并保持零速

**Interfaces:**
- Consumes: 四个模块分支的审核提交
- Produces: 可构建但未 Arm 电机的集成树

- [ ] **Step 1: 逐个合并并测试**

```text
git merge --no-ff codex/ultrasonic
python -m unittest discover -s tests -v
git merge --no-ff codex/ybimu
python -m unittest discover -s tests -v
git merge --no-ff codex/k230-link
python -m unittest discover -s tests -v
git merge --no-ff codex/adaptive-line-control
python -m unittest discover -s tests -v
```

Expected: 每次测试 PASS；`App_Main_Init()` 仍不调用 `Motor_Safety_Arm()`。

- [ ] **Step 2: 编译共同代码**

Run: CCS Theia → Build Project。

Expected: TI Arm Clang 编译、链接成功；无重复 ISR、重复符号或旧 BSP include。

- [ ] **Step 3: 提交冲突修复**

```bash
git add MSPM0G3507_LineFollowing_Car tests
git commit -m "chore: integrate reviewed sensor modules"
```

若无冲突和无新增修改，不创建空提交。

### Task 2: 实现 SafetySupervisor 状态与优先级

**Interfaces:**
- Consumes: `SafetyInputs`, `MotionRequest`, `now_ms`
- Produces: `SafetyDecision`, `SafetySupervisorState`

- [ ] **Step 1: 写失败状态测试**

```python
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"

class SafetySupervisorContract(unittest.TestCase):
    def test_states_and_priority_exist(self):
        text = (ROOT / "application/safety_supervisor.h").read_text(encoding="utf-8")
        for token in ("SAFETY_BOOT_SAFE", "SAFETY_READY", "SAFETY_RUNNING",
                      "SAFETY_LIMITED", "SAFETY_STOP_LATCHED", "SAFETY_FAULT"):
            self.assertIn(token, text)

    def test_only_adapter_calls_motor_safety_speed(self):
        matches = []
        for path in ROOT.rglob("*.c"):
            if "Motor_Safety_RequestSpeed" in path.read_text(encoding="utf-8", errors="ignore"):
                matches.append(path.as_posix())
        self.assertEqual(1, len(matches), matches)
        self.assertTrue(matches[0].endswith("modules/motor/motor_adapter.c"))
```

Run: `python -m unittest tests.test_safety_supervisor -v`

Expected: FAIL，监督器和唯一 motor adapter 尚不存在。

- [ ] **Step 2: 定义安全输入与决策**

```c
typedef enum {
    SAFETY_BOOT_SAFE, SAFETY_READY, SAFETY_RUNNING,
    SAFETY_LIMITED, SAFETY_STOP_LATCHED, SAFETY_FAULT
} SafetySupervisorState;

typedef struct {
    UltrasonicSnapshot ultrasonic;
    YbImuSnapshot imu;
    K230VisionSnapshot vision;
    bool vision_required;
    bool imu_required;
    bool start_pressed;
    bool reset_pressed;
    bool power_qualified;
} SafetyInputs;

typedef struct {
    bool approved;
    int16_t left_speed;
    int16_t right_speed;
    uint16_t reason;
    SafetySupervisorState state;
} SafetyDecision;
```

- [ ] **Step 3: 实现严格优先级**

```text
MotorSafety fault
→ obstacle <= 200mm
→ required IMU/vision stale
→ obstacle 200..350mm speed limit
→ validated mission request
```

恢复条件为超声波 `>400mm` 连续 5 个有效样本并收到 K1；FAULT 只允许重新初始化解除。

在 `application/config/safety_config.h` 定义 `MOTION_REQUEST_MAX_AGE_MS (50U)`；请求无效、时间戳超过 50ms 或左右速度超过当前档位限制时，一律生成零速拒绝决策。

- [ ] **Step 4: 建立唯一 motor adapter**

```c
void MotorAdapter_Apply(const SafetyDecision *decision)
{
    if (decision == 0 || !decision->approved) {
        Motor_Safety_RequestSpeed(0, 0, 0, 0);
        return;
    }
    Motor_Safety_RequestSpeed(0, decision->left_speed,
                              0, decision->right_speed);
}
```

- [ ] **Step 5: 测试并提交**

Run: `python -m unittest tests.test_safety_supervisor tests.test_motor_authority -v`

Expected: PASS；超声波优先于任务，旧数据不能继续驱动。

```bash
git add MSPM0G3507_LineFollowing_Car/application MSPM0G3507_LineFollowing_Car/modules/motor tests
git commit -m "feat: arbitrate all motion through safety supervisor"
```

### Task 3: 配置唯一最终 SysConfig

**Interfaces:**
- Consumes: approved pin contract
- Produces: SysConfig-generated GPIO/UART/I2C/timer symbols

- [ ] **Step 1: 写最终引脚失败测试**

```python
    def test_final_pin_contract(self):
        syscfg = (ROOT / "empty.syscfg").read_text(encoding="utf-8")
        for pin in ("PA10", "PA11", "PB6", "PB7", "PA12", "PA13",
                    "PA15", "PA16", "PA17", "PA18",
                    "PA21", "PA22", "PA26", "PA27"):
            self.assertIn(pin, syscfg)
```

Run: `python -m unittest tests.test_final_pin_contract -v`

Expected: FAIL，因为 K230 和超声波引脚尚未加入。

- [ ] **Step 2: 在 SysConfig GUI 配置**

保持现有 UART0、UART1、灰度、蜂鸣器和 1ms timer；新增：

```text
YbImu software/hardware I2C: PA12 SCL, PA13 SDA, 100kHz
K230 UART: PA21 TX, PA22 RX, 115200, RX interrupt
HC-SR04: PA26 output Trig, PA27 capture/input Echo
```

若 PA27 不能映射到所选捕获通道，先记录 SysConfig 实际可选映射并停止集成，不擅自换引脚。

- [ ] **Step 3: 重新生成并验证**

Run: SysConfig Generate → CCS Build Project。

Expected: 生成成功、无 pinmux 冲突、编译链接成功。

Run: `python -m unittest discover -s tests -v`

Expected: PASS。

- [ ] **Step 4: 提交 SysConfig**

```bash
git add MSPM0G3507_LineFollowing_Car/empty.syscfg MSPM0G3507_LineFollowing_Car/.cproject tests/test_final_pin_contract.py
git commit -m "feat: configure final sensor pin map"
```

不提交 `Debug/` 生成产物。

### Task 4: 集成任务调度、遥测和安全启动

**Interfaces:**
- Consumes: all snapshots and MissionManager MotionRequest
- Produces: 1ms-safe main loop and bounded telemetry

- [ ] **Step 1: 注册周期任务**

```text
1ms: SafetySupervisor step, MotorSafety service
microsecond service: line scanner and ultrasonic trigger pulse
5ms: line estimator/control
10ms: YbImu group start, mission step
60ms: ultrasonic measurement start
low priority: K230 parse and UART0 telemetry
```

每个任务记录最大执行时间和超期次数；遥测缓冲满时丢弃遥测，不阻塞安全任务。

`MissionManager` 每个 10ms 周期从最新快照构造 `MotionContext`：编码器填充 `distance_mm/odometry_fresh`，YbImu 填充 `yaw_deg/yaw_fresh`，K230 填充 `vision_event/vision_fresh`，然后调用 `MotionPrimitive_StepWithContext` 生成唯一任务请求。

- [ ] **Step 2: 建立安全启动门**

只有 `power_qualified`、超声波有效、当前任务关键模块健康且 K1 启动后，才调用 `Motor_Safety_Arm()` 并进入 RUNNING。复位后默认 BOOT_SAFE。

- [ ] **Step 3: 运行离线验证**

Run: `python -m unittest discover -s tests -v`

Expected: 全部 PASS。

Run: `rg -n "delay_ms|while\s*\(" MSPM0G3507_LineFollowing_Car/application MSPM0G3507_LineFollowing_Car/modules`

Expected: 控制、传感器服务和协议路径无阻塞等待；允许的初始化短延时逐项有注释且不在 RUNNING 路径。

- [ ] **Step 4: 提交**

```bash
git add MSPM0G3507_LineFollowing_Car/application tests
git commit -m "feat: schedule sensor safety application"
```

### Task 5: 电源和最终接线文档

**Interfaces:**
- Consumes: 用户实物照片、扩展板丝印、万用表结果
- Produces: 可逐线勾选的接线表与电源验收记录

- [ ] **Step 1: 写电源验收表**

`docs/hardware/power-acceptance.md` 记录：电池实测电压、扩展板允许输入依据、5V 空载、K230 启动最低电压、逻辑全载电压、电机启停压降、10 分钟温升、结论与测试人。

若输入仍为满电 12.6V 且没有制造商允许或合规降压方案，结论必须为 FAIL，软件保持 BOOT_SAFE。

- [ ] **Step 2: 写最终逐线接线表**

`docs/hardware/final-wiring.md` 按断电顺序列出：

```text
YbImu: 3.3V, GND, PA12/SCL, PA13/SDA
HC-SR04: 5V, GND, PA26/Trig, Echo→level shifter→PA27
K230: 5V, GND, PA21/TX→IO10/RX, PA22/RX←IO9/TX
Gray: verified supply, GND, PA15/AD0, PA16/AD1, PA17/AD2, PA18/OUT
Motor UART: PB6/TX→driver RX, PB7/RX←driver TX
Debug UART: PA10/TX, PA11/RX
```

每条写明两端、供电电压、交叉方向和电平转换；附用户照片后再次核对接口朝向。

- [ ] **Step 3: 提交文档**

```bash
git add docs/hardware/power-acceptance.md docs/hardware/final-wiring.md
git commit -m "docs: add final power and wiring acceptance"
```

### Task 6: 分级上板和实车验收

**Interfaces:**
- Consumes: 已构建固件、通过的电源/接线表
- Produces: `docs/verification/sensor-platform-test-record.md`

- [ ] **Step 1: 断开电机动力做逻辑测试**

Expected: 灰度完整帧目标 ≤2ms；YbImu 100Hz；超声波 60ms/30ms；K230 事件和 300ms stale；1ms 安全任务无超期。

- [ ] **Step 2: 执行电机前 Checklist**

确认轮子架空、M2/M4 左右、正负方向、UART 交叉与共地、零速帧、0→30% soft-start、200ms watchdog、可立即断开动力电源。

- [ ] **Step 3: 架空轮验证**

依次验证零速、10%、20%、30%、左右转、单轮反转状态、超声波中断和通信断线。任何异常立即断电并记录，不继续落地。

- [ ] **Step 4: 低速落地路线**

按安全档测试直线、缓弯、连续弯、直角、短时丢线、45°/800ms 失败停车、障碍限速/急停/人工恢复、定距、定角、视觉掉线。

- [ ] **Step 5: 长时间与稳健档回归**

所有安全档项目通过后，运行全模块 30 分钟压力测试，再评估稳健档；冲刺档只在稳健档完整回归后启用。

- [ ] **Step 6: 记录最终结论并提交**

```text
可编译 / 可烧录 / 可架空运行 / 可低速落地 / 可稳健档运行
```

逐级标记 PASS/FAIL 和证据，禁止用前一级代替后一级。

```bash
git add docs/verification/sensor-platform-test-record.md
git commit -m "test: record integrated car validation"
```
