# Ultrasonic Emergency Guard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 交付固定朝前、完全非阻塞的 HC-SR04 距离快照模块，为安全层提供限速和锁存急停输入。

**Architecture:** 模块层实现测量状态机、脉宽换算、有效性和快照；BSP 层只负责 Trig 和边沿捕获。模块工作树不永久修改 SysConfig，通过可替换 BSP 接口进行离线测试，最终引脚配置在集成分支完成。

**Tech Stack:** MSPM0G3507、C11、Timer capture/GPIO、Python unittest、TI DriverLib。

## Global Constraints

- 分支：`codex/ultrasonic`，基于 modular-foundation 审核提交。
- PA26=Trig、PA27=Echo；Echo 必须先降压到 3.3V。
- 触发初值 60ms；回波超时 30ms；ISR 不做浮点运算和打印。
- 超声波不能直接调用 motor；只发布 `UltrasonicSnapshot`。
- 本分支不永久修改 `empty.syscfg`。

---

## File Structure

- Create: `modules/ultrasonic/ultrasonic.c/.h` — 状态机、脉宽换算、快照。
- Create: `modules/ultrasonic/ultrasonic_config.h` — 周期、超时、物理范围。
- Create: `bsp/bsp_ultrasonic.h/.c` — Trig 与捕获适配接口；集成前提供零硬件 stub。
- Create: `tests/test_ultrasonic_contract.py` — 非阻塞、安全边界和接口合同。
- Create: `docs/hardware/ultrasonic-bench-checklist.md` — 电平与固定距离验收。

### Task 1: 定义快照与换算边界

**Interfaces:**
- Consumes: Echo high pulse in microseconds
- Produces: `UltrasonicSnapshot`, `Ultrasonic_PulseUsToMm(uint32_t, uint16_t *)`

- [ ] **Step 1: 写失败测试**

```python
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"

class UltrasonicContract(unittest.TestCase):
    def test_public_contract_and_limits(self):
        header = (ROOT / "modules/ultrasonic/ultrasonic.h").read_text(encoding="utf-8")
        config = (ROOT / "modules/ultrasonic/ultrasonic_config.h").read_text(encoding="utf-8")
        for token in ("UltrasonicSnapshot", "distance_mm", "pulse_us",
                      "Ultrasonic_PulseUsToMm", "Ultrasonic_GetSnapshot"):
            self.assertIn(token, header)
        self.assertIn("60000U", config)
        self.assertIn("30000U", config)
```

Run: `python -m unittest tests.test_ultrasonic_contract -v`

Expected: FAIL，模块文件不存在。

- [ ] **Step 2: 创建配置与接口**

```c
#define ULTRASONIC_TRIGGER_PERIOD_US  (60000U)
#define ULTRASONIC_ECHO_TIMEOUT_US    (30000U)
#define ULTRASONIC_MIN_PULSE_US       (100U)
#define ULTRASONIC_MAX_PULSE_US       (25000U)

typedef struct {
    ModuleStatus status;
    uint32_t pulse_us;
    uint16_t distance_mm;
} UltrasonicSnapshot;

bool Ultrasonic_PulseUsToMm(uint32_t pulse_us, uint16_t *distance_mm);
bool Ultrasonic_GetSnapshot(UltrasonicSnapshot *out);
```

换算使用整数舍入：`distance_mm = (pulse_us * 343 + 1000) / 2000`；超出最小/最大脉宽返回 false。

- [ ] **Step 3: 运行测试并提交**

Run: `python -m unittest tests.test_ultrasonic_contract -v`

Expected: PASS。

```bash
git add MSPM0G3507_LineFollowing_Car/modules/ultrasonic tests/test_ultrasonic_contract.py
git commit -m "feat: define ultrasonic snapshot contract"
```

### Task 2: 实现非阻塞测量状态机

**Interfaces:**
- Consumes: `now_us`, BSP Trig, captured rise/fall timestamps
- Produces: `Ultrasonic_Init`, `Ultrasonic_Service`, `Ultrasonic_OnEchoEdge`

- [ ] **Step 1: 增加状态机合同测试**

```python
    def test_state_machine_is_non_blocking(self):
        source = (ROOT / "modules/ultrasonic/ultrasonic.c").read_text(encoding="utf-8")
        for state in ("ULTRA_IDLE", "ULTRA_WAIT_RISE", "ULTRA_WAIT_FALL"):
            self.assertIn(state, source)
        self.assertNotIn("delay_ms", source)
        self.assertNotRegex(source, r"while\s*\(")
```

Run: `python -m unittest tests.test_ultrasonic_contract -v`

Expected: FAIL，状态机未实现。

- [ ] **Step 2: 定义 BSP 边界**

```c
void BSP_Ultrasonic_Init(void);
uint32_t BSP_Ultrasonic_NowUs(void);
void BSP_Ultrasonic_SetTrig(bool high);
```

`BSP_Ultrasonic_SetTrig(true)` 后由状态机在 10μs 到期时拉低；不得用延时函数维持脉冲。

- [ ] **Step 3: 实现状态迁移**

```c
void Ultrasonic_Service(uint32_t now_us)
{
    if (state == ULTRA_IDLE && elapsed(now_us, last_trigger_us) >= 60000U) {
        BSP_Ultrasonic_SetTrig(true);
        trigger_high_us = now_us;
        state = ULTRA_TRIGGER_HIGH;
    } else if (state == ULTRA_TRIGGER_HIGH && elapsed(now_us, trigger_high_us) >= 10U) {
        BSP_Ultrasonic_SetTrig(false);
        measure_start_us = now_us;
        state = ULTRA_WAIT_RISE;
    } else if ((state == ULTRA_WAIT_RISE || state == ULTRA_WAIT_FALL) &&
               elapsed(now_us, measure_start_us) >= ULTRASONIC_ECHO_TIMEOUT_US) {
        publish_invalid(now_us, MODULE_HEALTH_DEGRADED);
        state = ULTRA_IDLE;
    }
}
```

边沿回调只保存时间戳和推进状态；下降沿后在非 ISR 路径换算并发布快照。

- [ ] **Step 4: 验证与提交**

Run: `python -m unittest tests.test_ultrasonic_contract -v`

Expected: PASS；源码无 `delay_ms` 和 Echo 等待循环。

```bash
git add MSPM0G3507_LineFollowing_Car/modules/ultrasonic MSPM0G3507_LineFollowing_Car/bsp tests/test_ultrasonic_contract.py
git commit -m "feat: add nonblocking ultrasonic state machine"
```

### Task 3: 编写模块台架验收

**Interfaces:**
- Consumes: 5V、GND、Trig、已降压 Echo
- Produces: 距离误差、无回波超时和电平验收记录

- [ ] **Step 1: 写硬件 Checklist**

`docs/hardware/ultrasonic-bench-checklist.md` 必须列出：断电接线、Echo 分压输出最大值、10/20/40/80cm 固定点、移除障碍后的 30ms 超时、60ms 触发周期、传感器无响应时主循环仍运行。

- [ ] **Step 2: 运行全套离线测试**

Run: `python -m unittest discover -s tests -v`

Expected: PASS，现有 motor safety 合同不变。

- [ ] **Step 3: 提交文档**

```bash
git add docs/hardware/ultrasonic-bench-checklist.md
git commit -m "docs: add ultrasonic bench acceptance"
```
