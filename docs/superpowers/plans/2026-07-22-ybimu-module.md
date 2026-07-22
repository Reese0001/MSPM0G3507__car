# YbImu Nine-Axis Module Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用 YbImu 九轴 I2C 快照替代 MPU6050/eMPL 的活动控制路径，提供 100Hz 姿态、角速度和健康状态。

**Architecture:** 协议层负责厂家寄存器与字节解码，模块层负责分步读取、坐标映射、时效和校准状态，BSP 层负责 I2C 事务。校准使用可取消的状态机，不复制厂家例程中的阻塞等待循环。

**Tech Stack:** MSPM0G3507、I2C 100kHz、C11、Python unittest、Yahboom YbImu vendor protocol。

## Global Constraints

- 分支：`codex/ybimu`，基于 modular-foundation 审核提交。
- PA12=SCL、PA13=SDA、7-bit 地址 `0x23`。
- 厂家寄存器：gyro `0x0A`、mag `0x10`、quat `0x16`、Euler `0x26`。
- Euler 厂家数据为 little-endian IEEE754 float，输出统一为 degree；gyro 输出 rad/s。
- 本分支不永久修改 `empty.syscfg`，不同时运行 MPU6050/eMPL。

---

## File Structure

- Create: `modules/ybimu/ybimu_protocol.c/.h` — 寄存器常量与无硬件字节解码。
- Create: `modules/ybimu/ybimu.c/.h` — 100Hz 快照、健康度、坐标映射。
- Create: `modules/ybimu/ybimu_config.h` — 地址、周期、过期和安装方向。
- Create: `bsp/bsp_i2c.c/.h` — 受限长度的非永久适配接口。
- Create: `tests/test_ybimu_contract.py` — 协议、单位、非阻塞和旧模块隔离合同。
- Create: `docs/hardware/ybimu-calibration-checklist.md`。

### Task 1: 锁定厂家协议与字节解码

**Interfaces:**
- Consumes: little-endian byte arrays
- Produces: `YbImuProtocol_DecodeI16LE`, `YbImuProtocol_DecodeFloatLE`

- [ ] **Step 1: 写失败测试**

```python
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"

class YbImuContract(unittest.TestCase):
    def test_vendor_registers_and_address(self):
        text = (ROOT / "modules/ybimu/ybimu_protocol.h").read_text(encoding="utf-8")
        for token in ("0x23U", "0x0AU", "0x10U", "0x16U", "0x26U"):
            self.assertIn(token, text)
```

Run: `python -m unittest tests.test_ybimu_contract -v`

Expected: FAIL，协议头不存在。

- [ ] **Step 2: 定义协议常量和解码**

```c
#define YBIMU_I2C_ADDRESS   (0x23U)
#define YBIMU_REG_GYRO      (0x0AU)
#define YBIMU_REG_MAG       (0x10U)
#define YBIMU_REG_QUAT      (0x16U)
#define YBIMU_REG_EULER     (0x26U)

int16_t YbImuProtocol_DecodeI16LE(const uint8_t bytes[2]);
float YbImuProtocol_DecodeFloatLE(const uint8_t bytes[4]);
```

Float 解码使用 `uint32_t` 逐字节组装后 `memcpy` 到 float；禁止未对齐指针强转。

- [ ] **Step 3: 验证并提交**

Run: `python -m unittest tests.test_ybimu_contract -v`

Expected: PASS。

```bash
git add MSPM0G3507_LineFollowing_Car/modules/ybimu tests/test_ybimu_contract.py
git commit -m "feat: add YbImu protocol decoder"
```

### Task 2: 实现 100Hz 姿态快照

**Interfaces:**
- Consumes: `BSP_I2C_Read(0x23, reg, buffer, length)`
- Produces: `YbImu_Init`, `YbImu_Service`, `YbImu_GetSnapshot`

- [ ] **Step 1: 增加快照合同测试**

```python
    def test_snapshot_is_timestamped_and_nonblocking(self):
        header = (ROOT / "modules/ybimu/ybimu.h").read_text(encoding="utf-8")
        source = (ROOT / "modules/ybimu/ybimu.c").read_text(encoding="utf-8")
        for token in ("YbImuSnapshot", "gyro_rad_s", "euler_deg", "quat", "mag_uT"):
            self.assertIn(token, header)
        self.assertNotIn("delay_ms", source)
        self.assertNotRegex(source, r"while\s*\(")
```

Run: `python -m unittest tests.test_ybimu_contract -v`

Expected: FAIL，快照模块不存在。

- [ ] **Step 2: 定义快照**

```c
typedef struct {
    ModuleStatus status;
    float gyro_rad_s[3];
    float mag_uT[3];
    float quat[4];
    float euler_deg[3];
    bool magnetic_heading_healthy;
} YbImuSnapshot;

void YbImu_Init(uint32_t now_ms);
void YbImu_Service(uint32_t now_ms);
bool YbImu_GetSnapshot(YbImuSnapshot *out);
```

- [ ] **Step 3: 实现分步读取**

`YbImu_Service` 每次只推进一项：gyro 6B → mag 6B → quat 16B → Euler 12B → 原子发布。任一事务失败则增加错误计数，完整组未成功时不得更新时间戳。

```c
static const uint8_t registers[] = {
    YBIMU_REG_GYRO, YBIMU_REG_MAG, YBIMU_REG_QUAT, YBIMU_REG_EULER
};
```

模块每 10ms 开始一个新组；前一组未完成时不覆盖公开快照。

- [ ] **Step 4: 验证并提交**

Run: `python -m unittest tests.test_ybimu_contract -v`

Expected: PASS；模块源码无校准等待循环。

```bash
git add MSPM0G3507_LineFollowing_Car/modules/ybimu MSPM0G3507_LineFollowing_Car/bsp tests/test_ybimu_contract.py
git commit -m "feat: publish YbImu attitude snapshots"
```

### Task 3: 增加校准与磁健康门控

**Interfaces:**
- Consumes: K1 请求、静止样本、磁场模长与变化率
- Produces: `YbImu_RequestCalibration`, `YbImuCalibrationState`, `magnetic_heading_healthy`

- [ ] **Step 1: 写校准状态合同**

```python
    def test_calibration_has_timeout_states(self):
        text = (ROOT / "modules/ybimu/ybimu.h").read_text(encoding="utf-8")
        for token in ("YBIMU_CAL_IDLE", "YBIMU_CAL_RUNNING",
                      "YBIMU_CAL_SUCCESS", "YBIMU_CAL_FAILED"):
            self.assertIn(token, text)
```

- [ ] **Step 2: 实现非阻塞校准状态机**

IMU 校准写寄存器 `0x70=0x01`，磁校准写 `0x71=0x01`；每 100ms 读取同一寄存器状态，IMU 7s 超时，磁校准由用户明确结束或 60s 上限终止。校准期间电机必须保持零速。

- [ ] **Step 3: 写台架 Checklist 并全量验证**

文档列出：版本读取、静止 60s yaw 漂移、X/Y/Z 正方向、左右转 yaw 符号、8 字磁校准、电机断电/运行时磁场差异、100Hz 数据年龄。

Run: `python -m unittest discover -s tests -v`

Expected: PASS。

- [ ] **Step 4: 提交**

```bash
git add MSPM0G3507_LineFollowing_Car/modules/ybimu tests/test_ybimu_contract.py docs/hardware/ybimu-calibration-checklist.md
git commit -m "feat: add YbImu calibration health gating"
```
