# Adaptive Line Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把现有阻塞式八路读取和单一 PD 升级为非阻塞扫描、线置信度、预测式 PD、弯道速度规划和 IMU 限角找线能力。

**Architecture:** `line_scanner` 只发布同一时间戳的 8-bit 快照，`line_estimator` 把位型历史转换为位置、趋势、置信度和事件，`line_controller` 生成目标车体速度，`line_recovery` 管理明确的单轮反转状态。纯决策层不调用 DriverLib 或 motor API。

**Tech Stack:** C11、GPIO MUX、Python unittest、MSPM0 cooperative scheduler。

## Global Constraints

- 分支：`codex/adaptive-line-control`，基于 modular-foundation 审核提交。
- PA15/PA16/PA17 为选通，PA18 为 OUT；黑线极性实车复核。
- 普通 FOLLOW 状态禁止单轮反转；反转仅限 HARD_CURVE/PIVOT_SEARCH。
- 所有速度输出是 MotionRequest，不直接调用 `Contrl_Speed` 或 `Motion_Car_Control`。
- 初次落地速度不高于 30%；控制周期目标 5～10ms。

---

## File Structure

- Create: `modules/line_tracking/line_scanner.c/.h` — 非阻塞八路采集。
- Create: `modules/line_tracking/line_estimator.c/.h` — 位型、趋势、置信度、事件。
- Create: `modules/line_tracking/line_controller.c/.h` — 预测 PD 和速度规划。
- Create: `application/line_recovery.c/.h` — 恢复状态机。
- Create: `application/motion_primitives.c/.h` — Follow/Distance/Turn/Search/Stop 接口。
- Create: `application/config/line_control_config.h`, `line_recovery_config.h`。
- Create: `tests/test_line_estimator.py`, `tests/test_line_recovery.py`。
- Retire after parity: old `modules/line_tracking/app_irtracking.c/.h`。

### Task 1: 非阻塞灰度扫描

**Interfaces:**
- Consumes: `now_us`, PA15～PA18 BSP
- Produces: `LineSensorSnapshot { ModuleStatus status; uint8_t black_bits; }`

- [ ] **Step 1: 写失败合同**

```python
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"

class LineScannerContract(unittest.TestCase):
    def test_scanner_is_nonblocking(self):
        source = (ROOT / "modules/line_tracking/line_scanner.c").read_text(encoding="utf-8")
        self.assertNotIn("delay_ms", source)
        self.assertNotRegex(source, r"while\s*\(")
        self.assertIn("black_bits", source)
```

Run: `python -m unittest tests.test_line_estimator -v`

Expected: FAIL，新扫描器不存在。

- [ ] **Step 2: 实现选通状态机**

```c
typedef enum { LINE_SCAN_SELECT, LINE_SCAN_SETTLE, LINE_SCAN_SAMPLE } LineScanState;
void LineScanner_Service(uint32_t now_us);
bool LineScanner_GetSnapshot(LineSensorSnapshot *out);
```

每次 Service 最多完成一次 GPIO 操作；通道 0～7 全部采完后原子发布 `black_bits`。`LINE_MUX_SETTLE_US` 放入配置，初值 10μs，最终按示波器结果调整。

- [ ] **Step 3: 验证并提交**

Run: `python -m unittest tests.test_line_estimator -v`

Expected: PASS；旧每路 `delay_ms(1)` 不在活动扫描路径。

```bash
git add MSPM0G3507_LineFollowing_Car/modules/line_tracking tests/test_line_estimator.py
git commit -m "feat: scan line sensors without blocking"
```

### Task 2: 估计位置、趋势和事件

**Interfaces:**
- Consumes: `LineSensorSnapshot`
- Produces: `LineEstimate`

- [ ] **Step 1: 写 256 位型测试模型**

```python
WEIGHTS = (-7, -5, -3, -1, 1, 3, 5, 7)

def weighted_error(bits: int):
    active = [WEIGHTS[i] for i in range(8) if bits & (1 << i)]
    return None if not active else sum(active) / len(active)

def test_all_patterns_are_bounded_and_symmetric():
    for bits in range(1, 256):
        value = weighted_error(bits)
        mirror = weighted_error(int(f"{bits:08b}"[::-1], 2))
        assert -7 <= value <= 7
        assert abs(value + mirror) < 1e-6
```

- [ ] **Step 2: 定义估计结果**

```c
typedef enum { LINE_EVENT_NONE, LINE_EVENT_HARD_LEFT, LINE_EVENT_HARD_RIGHT,
               LINE_EVENT_WIDE_BLACK, LINE_EVENT_LOST } LineEvent;
typedef struct {
    ModuleStatus status;
    float error;
    float derivative;
    float predicted_error;
    uint8_t confidence;
    LineEvent event;
} LineEstimate;
```

置信度由有效通道连续性、是否多簇、历史跳变和数据年龄计算，范围 0～100。宽黑区只输出候选事件，不直接停车。

- [ ] **Step 3: 运行测试并提交**

Run: `python -m unittest tests.test_line_estimator -v`

Expected: 256 位型、镜像对称、全白、宽黑和历史趋势测试全部 PASS。

```bash
git add MSPM0G3507_LineFollowing_Car/modules/line_tracking tests/test_line_estimator.py
git commit -m "feat: estimate line confidence and trend"
```

### Task 3: 预测 PD 与速度规划

**Interfaces:**
- Consumes: `LineEstimate`, speed profile
- Produces: `LineControlOutput { int16_t forward; int16_t turn; bool valid; }`

- [ ] **Step 1: 写控制边界测试**

测试直道加速、预测入弯减速、低置信度减速、FOLLOW 不反转、目标速度每周期变化受限。

```python
def test_follow_never_reverses_inner_wheel():
    for base in range(1, 301):
        correction_limit = base * 80 // 100
        assert base - correction_limit >= 0
```

- [ ] **Step 2: 实现控制公式**

```c
predicted = error + prediction_horizon_s * derivative;
turn = kp_for_speed * error + kd_for_speed * derivative;
turn = clamp(turn, -base_speed * 0.8f, base_speed * 0.8f);
target_speed = speed_from_curve_confidence(predicted, yaw_rate, confidence);
target_speed = slew_limit(previous_speed, target_speed, accel_step, decel_step);
```

参数全部放在 `line_control_config.h`，积分默认关闭。

- [ ] **Step 3: 验证并提交**

Run: `python -m unittest tests.test_line_estimator -v`

Expected: PASS。

```bash
git add MSPM0G3507_LineFollowing_Car/modules/line_tracking MSPM0G3507_LineFollowing_Car/application/config tests
git commit -m "feat: add predictive line speed control"
```

### Task 4: IMU 限角丢线恢复

**Interfaces:**
- Consumes: line estimate, `yaw_deg`, `yaw_fresh`, `now_ms`
- Produces: `MotionRequest` and `LineRecoveryState`

- [ ] **Step 1: 写状态转换测试**

覆盖 FOLLOW→LOSS_CONFIRM、左右趋势选择 PIVOT、连续 3 帧重获、45° 超角、800ms 超时、IMU 过期、超声波中断。

- [ ] **Step 2: 定义配置**

```c
#define LINE_LOSS_CONFIRM_COUNT       (3U)
#define LINE_REACQUIRE_COUNT          (3U)
#define LINE_PIVOT_FORWARD_PERCENT    (18)
#define LINE_PIVOT_REVERSE_PERCENT    (12)
#define LINE_RECOVERY_MAX_YAW_DEG     (45.0f)
#define LINE_RECOVERY_TIMEOUT_MS      (800U)
#define LINE_ALIGN_DURATION_MS        (300U)
```

- [ ] **Step 3: 实现状态机**

左转 PIVOT 输出左轮 `-12%`、右轮 `+18%`，右转相反；整车不得两轮同时为负。IMU 不新鲜或超过转角/超时时输出 invalid MotionRequest 并进入 RECOVERY_FAULT。

- [ ] **Step 4: 验证并提交**

Run: `python -m unittest tests.test_line_recovery -v`

Expected: PASS。

```bash
git add MSPM0G3507_LineFollowing_Car/application tests/test_line_recovery.py
git commit -m "feat: recover line with bounded pivot search"
```

### Task 5: 建立运动基元接口

**Interfaces:**
- Consumes: module-independent `MotionContext`
- Produces: `MotionPrimitive_Start`, `MotionPrimitive_StepWithContext`, `MotionPrimitive_Cancel`

- [ ] **Step 1: 定义非阻塞接口**

```c
typedef enum { MOTION_RUNNING, MOTION_COMPLETE, MOTION_FAILED } MotionResult;
typedef struct {
    LineEstimate line;
    int32_t distance_mm;
    float yaw_deg;
    uint8_t vision_event;
    bool odometry_fresh;
    bool yaw_fresh;
    bool vision_fresh;
} MotionContext;
MotionResult MotionPrimitive_StepWithContext(uint32_t now_ms,
                                             const MotionContext *context,
                                             MotionRequest *request);
void MotionPrimitive_Cancel(void);
```

单模块工作树的测试调用 `MotionPrimitive_StepWithContext` 注入数据；集成分支由 MissionManager 从已合并模块构造 `MotionContext` 后转调。提供 FollowLine、DriveDistance、TurnRelative、SearchLine、StopAtMarker、WaitVision 六种参数结构；每次 Step 只推进一次状态，不调用延时。

- [ ] **Step 2: 更新旧入口合同**

旧 `LineWalking()` 从活动主循环移除；`Questions` 只允许调用 motion primitive，不允许直接调用 `Motion_Car_Control`。

- [ ] **Step 3: 全量测试和提交**

Run: `python -m unittest discover -s tests -v`

Expected: PASS；`rg "Motion_Car_Control|Contrl_Speed" application modules/line_tracking` 只匹配 motor 适配边界允许项。

```bash
git add MSPM0G3507_LineFollowing_Car/application MSPM0G3507_LineFollowing_Car/modules/line_tracking tests
git commit -m "feat: add composable motion primitives"
```
