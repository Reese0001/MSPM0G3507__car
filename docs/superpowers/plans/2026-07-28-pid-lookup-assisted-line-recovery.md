# PID + 开环查表辅助循迹实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在保留位置环 + MPU6050 角度环串级 PID 的同时，接入 2 ms 八路灰度 15 位置查表前馈、三帧方向预测和丢线后单向持续找线，并生成新的 UniFlash TI-TXT。

**Architecture:** `line_lookup_control` 根据位置提供基础速度与前馈差速；`line_cascade_control` 将前馈差速与位置 PID 合并后送入角度环，并统一限制最终轮速。独立预测器只保存三帧可信位置；恢复状态机只在丢线时接管，锁定方向后用一个轮停、另一个轮正转的方式持续找线。

**Tech Stack:** C11、TI Arm Clang 4.0.4、裸机 1 ms 基准时间片、2 ms 灰度任务、Python `unittest`、MSVC `/W4 /WX` 主机 harness。

## Global Constraints

- `bit0=X1=车头右侧`，`bit7=X8=车头左侧`；合法位置 `-7..+7`，负数为线路在左。
- 灰度任务固定 2 ms；跨越大于 1 个位置的跳变连续 2 帧才接受。
- 串级 PID 保持活动：位置 `KP=14.0/KD=0.010`，角度 `KP=1.5/KD=0.55` 作为起点。
- 查表只做辅助前馈，不能替代或绕开 PID。
- `-1/0/+1` 查表差速为 0，位置 PID 有效误差为 0，微分噪声不得穿过中心死区。
- 正常、ALIGN 和 SEEK 输出均满足 `0 <= left/right <= 140`；底层 450 安全限幅仍保留。
- 丢线方向只锁定一次；不得倒车、反向、往返扫描或因 yaw/时间到阈值停车。
- K1 门控、软启动、电机 UART、M2/M4 映射、看门狗和故障停机保持不变。

---

### Task 1: 固定传感器方向、15 位置和 2 ms 防抖合同

**Files:**
- Modify: `tests/line_position_harness.c`
- Modify: `tests/test_line_scanner_timebase.py`
- Modify: `tests/test_line_following_contract.py`

**Interfaces:**
- Consumes: `LinePosition_Update(uint8_t)`、`app_task_slots[]`。
- Produces: X1/X8 方向、全部 15 种图案、2 ms 周期和两帧跳变防抖合同。

- [ ] **Step 1: 增加明确合同**

```c
LinePosition_Reset();
CHECK(LinePosition_Update(0x01U).stable_position == 7);   /* X1 right */
LinePosition_Reset();
CHECK(LinePosition_Update(0x80U).stable_position == -7); /* X8 left */
```

```python
self.assertIn("{APP_TASK_SENSOR, sensor_task, 2U * APP_TASK_BASE_TICK_MS", tasks)
self.assertIn("LINE_POSITION_JUMP_ACCEPT_FRAMES (2U)", lookup_config)
```

- [ ] **Step 2: 运行基线合同**

Run: `python -m unittest tests.test_line_position tests.test_line_scanner_timebase tests.test_line_following_contract -v`

Expected: PASS；若失败，只修复传感器映射/周期，不修改控制器。

- [ ] **Step 3: 提交合同**

```powershell
git add tests/line_position_harness.c tests/test_line_scanner_timebase.py tests/test_line_following_contract.py
git commit -m "test: lock line sensor orientation and cadence"
```

---

### Task 2: 把查表控制收敛为纯前馈

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/config/line_lookup_config.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_lookup_control.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_lookup_control.c`
- Modify: `tests/line_lookup_control_harness.c`
- Modify: `tests/test_line_lookup_control.py`

**Interfaces:**
- Produces: `LineLookupControl_Step(int8_t position) -> LineLookupCommand`，其中 `base/diff` 是串级 PID 的前馈输入。

- [ ] **Step 1: 先写失败的低速前馈测试**

```c
center = LineLookupControl_Step(0);
CHECK(center.base == 140 && center.diff == 0);
CHECK(LineLookupControl_Step(-1).diff == 0);
CHECK(LineLookupControl_Step(1).diff == 0);
for (position = -7; position <= 7; position++) {
    LineLookupCommand c = LineLookupControl_Step(position);
    CHECK(c.valid);
    CHECK(c.base >= 0 && c.base <= 140);
    CHECK(c.left >= 0 && c.left <= 140);
    CHECK(c.right >= 0 && c.right <= 140);
}
```

- [ ] **Step 2: 运行并确认旧高速表失败**

Run: `python -m unittest tests.test_line_lookup_control -v`

Expected: FAIL；当前中心值为 420，且函数仍带 IMU 参数。

- [ ] **Step 3: 实现纯查表前馈**

```c
#define LINE_LOOKUP_COMMAND_LIMIT (140)
#define LINE_LOOKUP_TABLE_ENTRIES \
    {140, 0}, {140, 0}, {130, 12}, {115, 28}, \
    {95, 45}, {80, 60}, {70, 65}, {60, 60}
```

接口改为：

```c
LineLookupCommand LineLookupControl_Step(int8_t position);
```

删除查表器内部的 yaw 分支和 IMU stale 降级；该信息由串级 PID/恢复层处理。合成 `left=base-diff/right=base+diff` 后夹紧为 `0..140`。

- [ ] **Step 4: 验证查表方向与限幅**

Run: `python -m unittest tests.test_line_lookup_control tests.test_line_position -v`

Expected: PASS，负位置左轮更慢、正位置右轮更慢、中心三位置零差速。

- [ ] **Step 5: 提交查表前馈**

```powershell
git add MSPM0G3507_LineFollowing_Car/config/line_lookup_config.h MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_lookup_control.c MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_lookup_control.h tests/line_lookup_control_harness.c tests/test_line_lookup_control.py
git commit -m "refactor: provide bounded line lookup feedforward"
```

---

### Task 3: 将查表前馈融合进串级 PID

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_cascade_control.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_cascade_control.c`
- Modify: `MSPM0G3507_LineFollowing_Car/config/line_cascade_config.h`
- Modify: `tests/line_cascade_control_harness.c`
- Modify: `tests/test_line_cascade_control.py`

**Interfaces:**
- Consumes: `const LineLookupCommand *feedforward`、`LineEstimate`、`Mpu6050Snapshot`。
- Produces: 保持 `LineControlOutput`，但 `forward` 来自查表 `base`，`turn` 为查表差速 + 位置环 + 角度环结果。

- [ ] **Step 1: 写融合失败测试**

```c
LineLookupCommand feed = {.base = 100, .diff = 30, .valid = true};
set_center_estimate(&estimate, 10U); /* error 0 */
CHECK(LineCascadeControl_Step(&estimate, &feed, 0, false, 10U, &out));
CHECK(out.forward == 100);
CHECK(out.turn > 0); /* 前馈仍进入最终转向 */
```

再加入：相同前馈下非零位置误差会改变 `turn`；新鲜 IMU 会改变角度反馈；stale IMU 保留位置 PID 和前馈；最终 `abs(turn)<=forward`。

- [ ] **Step 2: 运行并确认旧接口失败**

Run: `python -m unittest tests.test_line_cascade_control -v`

Expected: FAIL，当前接口没有 `feedforward`，且 forward 仍来自重复表。

- [ ] **Step 3: 修改串级接口和中心死区**

```c
bool LineCascadeControl_Step(const LineEstimate *estimate,
                             const LineLookupCommand *feedforward,
                             const Mpu6050Snapshot *imu,
                             bool imu_fresh,
                             uint32_t now_ms,
                             LineControlOutput *output);
```

同时把 `LINE_CASCADE_MAX_COMMAND` 从 450 改为 140；电机安全层的 450 硬上限保持不变。串级控制器必须拒绝 `feedforward==NULL` 或 `feedforward->valid==false`。

位置环开头使用：

```c
float effective_error = absolute_float(estimate->error) <= 1.0f
                            ? 0.0f : estimate->error;
if (effective_error == 0.0f) {
    controller.filtered_derivative = 0.0f;
}
```

- [ ] **Step 4: 实现前馈 + 反馈融合**

```c
position_feedback = position_command(estimate, elapsed_ms);
desired_turn = (float)feedforward->diff + position_feedback;
combined_turn = angle_loop(desired_turn, imu, fresh_imu, elapsed_ms);
controller.forward = slew(controller.forward, feedforward->base,
                          LINE_CASCADE_ACCEL_STEP,
                          LINE_CASCADE_DECEL_STEP);
controller.turn = slew(controller.turn, round_command(combined_turn),
                       LINE_CASCADE_TURN_SLEW_STEP,
                       LINE_CASCADE_TURN_SLEW_STEP);
limit_wheels(&controller.forward, &controller.turn);
```

删除 `forward_by_position[]`，避免查表重复；保留原位置/角度 PID 参数。`limit_wheels` 继续保证 `abs(turn)<=forward`。

- [ ] **Step 5: 运行融合和 MPU 测试**

Run: `python -m unittest tests.test_line_cascade_control tests.test_mpu6050 tests.test_mpu6050_kalman -v`

Expected: PASS，且测试能区分 fresh/stale IMU。

- [ ] **Step 6: 提交 PID 融合**

```powershell
git add MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_cascade_control.c MSPM0G3507_LineFollowing_Car/modules/line_tracking/controller/line_cascade_control.h MSPM0G3507_LineFollowing_Car/config/line_cascade_config.h tests/line_cascade_control_harness.c tests/test_line_cascade_control.py
git commit -m "feat: fuse lookup feedforward into cascade PID"
```

---

### Task 4: 新增三帧丢线方向预测器

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/prediction/line_direction_predictor.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/prediction/line_direction_predictor.c`
- Create: `tests/line_direction_predictor_harness.c`
- Create: `tests/test_line_direction_predictor.py`

**Interfaces:**
- Produces: `Reset()`、`Record(int8_t)`、`Predict(void)->int8_t`；结果仅为 `-1/+1`。

- [ ] **Step 1: 写左趋势、右趋势、零趋势和无历史测试**

```c
LineDirectionPredictor_Reset();
CHECK(LineDirectionPredictor_Predict() == 1);
LineDirectionPredictor_Record(-1);
LineDirectionPredictor_Record(-3);
LineDirectionPredictor_Record(-5);
CHECK(LineDirectionPredictor_Predict() == -1);
LineDirectionPredictor_Reset();
LineDirectionPredictor_Record(1);
LineDirectionPredictor_Record(3);
LineDirectionPredictor_Record(5);
CHECK(LineDirectionPredictor_Predict() == 1);
```

- [ ] **Step 2: 运行并确认缺少模块**

Run: `python -m unittest tests.test_line_direction_predictor -v`

Expected: FAIL，缺少头文件/符号。

- [ ] **Step 3: 实现三帧历史**

```c
prediction = p2 + (p2 - p1) + (p1 - p0);
if (prediction < 0) return -1;
if (prediction > 0) return 1;
return last_nonzero == 0 ? 1 : last_nonzero;
```

只接受 `-7..+7`；不足三帧时使用最近非零方向；`Predict` 不修改历史。

- [ ] **Step 4: 验证并提交**

Run: `python -m unittest tests.test_line_direction_predictor -v`

Expected: PASS，MSVC 零警告。

```powershell
git add MSPM0G3507_LineFollowing_Car/modules/line_tracking/prediction tests/line_direction_predictor_harness.c tests/test_line_direction_predictor.py
git commit -m "feat: predict line loss direction from recent frames"
```

---

### Task 5: 重构单向持续找线状态机

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/config/line_recovery_config.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/recovery/line_recovery.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/recovery/line_recovery.c`
- Modify: `tests/line_recovery_harness.c`
- Modify: `tests/test_line_recovery.py`

**Interfaces:**
- States: `FOLLOW/SEEK_LEFT/SEEK_RIGHT/ALIGN/STOPPED`。
- Produces: `LineRecoveryDiagnostics {state,direction,yaw_delta_deg,yaw_fresh}`。

- [ ] **Step 1: 写会失败的“不反向、不倒车”测试**

```c
CHECK(LineRecovery_GetState() == LINE_RECOVERY_SEEK_LEFT);
CHECK(request.left_speed == 0 && request.right_speed > 0);
/* 5 s、360° 后仍然同向。 */
CHECK(step_with_imu(5000U, -360.0f, 30.0f, true, &request) == 0);
CHECK(LineRecovery_GetState() == LINE_RECOVERY_SEEK_LEFT);
CHECK(request.left_speed == 0 && request.right_speed > 0);
```

镜像测试右侧；所有状态断言 `left/right >= 0`；重复 sequence 不累计；连续三帧才 ALIGN；ALIGN 300 ms 后 FOLLOW。

- [ ] **Step 2: 运行并确认旧翻转逻辑失败**

Run: `python -m unittest tests.test_line_recovery -v`

Expected: FAIL，旧代码包含负轮速和 600 ms 翻转。

- [ ] **Step 3: 固定配置和状态**

```c
#define LINE_REACQUIRE_COUNT (3U)
#define LINE_ALIGN_DURATION_MS (300U)
#define LINE_SEEK_COMMAND (100)
#define LINE_SEEK_LIMITED_COMMAND (80)
#define LINE_SEEK_HIGH_YAW_DPS (120.0f)
#define LINE_ALIGN_COMMAND_LIMIT (80)
```

删除 forward-search、rotate timer、负速度常量和 `recovery_direction = -recovery_direction`。

- [ ] **Step 4: 实现锁定方向输出**

```c
speed = yaw_fresh && abs_yaw_rate >= LINE_SEEK_HIGH_YAW_DPS
            ? LINE_SEEK_LIMITED_COMMAND : LINE_SEEK_COMMAND;
if (direction < 0) publish_request(0, speed, now_ms, request);
else publish_request(speed, 0, now_ms, request);
```

首次 SEEK 保存 loss yaw；yaw 只更新诊断和速度档，不改变方向。ALIGN 对 PID+查表融合结果做 `0..80` 二次限幅。

- [ ] **Step 5: 验证并提交**

Run: `python -m unittest tests.test_line_recovery -v`

Expected: PASS，70°、100°、360°、5 s 均不翻转。

```powershell
git add MSPM0G3507_LineFollowing_Car/config/line_recovery_config.h MSPM0G3507_LineFollowing_Car/modules/line_tracking/recovery tests/line_recovery_harness.c tests/test_line_recovery.py
git commit -m "refactor: keep lost-line search in one direction"
```

---

### Task 6: 接入活动链、构建和 OLED

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/app/line/line_motion.c`
- Modify: `MSPM0G3507_LineFollowing_Car/app/log/runtime_observer.c`
- Modify: `MSPM0G3507_LineFollowing_Car/Makefile`
- Modify: `MSPM0G3507_LineFollowing_Car/.cproject`
- Modify: `tests/test_line_motion_contract.py`
- Modify: `tests/test_oled_contract.py`

**Interfaces:**
- Produces: `position -> lookup -> cascade PID -> recovery -> safety` 活动链。

- [ ] **Step 1: 写活动链合同**

```python
for token in ("LineLookupControl_Step", "LineCascadeControl_Step",
              "LineDirectionPredictor_Record", "LineRecovery_Step"):
    self.assertIn(token, line_motion)
self.assertLess(line_motion.index("LineLookupControl_Step"),
                line_motion.index("LineCascadeControl_Step"))
self.assertIn("line_lookup_control.c", makefile)
self.assertIn("line_cascade_control.c", makefile)
self.assertIn("line_direction_predictor.c", makefile)
```

- [ ] **Step 2: 运行并确认接线合同失败**

Run: `python -m unittest tests.test_line_motion_contract tests.test_oled_contract -v`

Expected: FAIL，当前活动链未调用 lookup/predictor，OLED 仍显示往返搜索标签。

- [ ] **Step 3: 按顺序接入控制链**

```c
if (sample->position.type == LINE_PATTERN_POSITION) {
    LineDirectionPredictor_Record(sample->position.stable_position);
    lookup = LineLookupControl_Step(sample->position.stable_position);
}
trend.direction = LineDirectionPredictor_Predict();
LineCascadeControl_Step(&estimate, &lookup,
                        imu_fresh ? &imu : 0, imu_fresh,
                        now_ms, &follow);
return LineRecovery_Step(&estimate, &trend, &follow,
                         imu.yaw_angle_deg, imu.yaw_rate_dps,
                         imu_fresh, false, now_ms, request);
```

噪声帧不记录历史、不发布请求。普通丢线保留 predictor 历史和已锁定方向，但让串级 PID 清空目标航向、微分和 slew 动态状态；重新捕获后 PID 从零修正平滑进入 ALIGN，不能继承丢线前的旧动态量。

- [ ] **Step 4: 更新构建源和 OLED**

Makefile 同时链接：

```make
modules/line_tracking/controller/line_lookup_control.c \
modules/line_tracking/controller/line_cascade_control.c \
modules/line_tracking/prediction/line_direction_predictor.c \
```

`.cproject` 加 prediction include path。OLED 状态改为 `LINE SEEK L/R`，保留 `LineCascadeControl_IsImuUsed()` 以证明角度环是否正在作用，并增加 recovery yaw delta。

- [ ] **Step 5: 运行集成合同**

Run: `python -m unittest tests.test_line_motion_contract tests.test_line_following_contract tests.test_oled_contract -v`

Expected: PASS。

- [ ] **Step 6: 提交集成**

```powershell
git add MSPM0G3507_LineFollowing_Car/app/line/line_motion.c MSPM0G3507_LineFollowing_Car/app/log/runtime_observer.c MSPM0G3507_LineFollowing_Car/Makefile MSPM0G3507_LineFollowing_Car/.cproject tests/test_line_motion_contract.py tests/test_oled_contract.py
git commit -m "feat: activate PID lookup assisted line control"
```

---

### Task 7: 全量验证、README 和 TI-TXT

**Files:**
- Modify: `README.md`
- Verify: `dist/firmware/MSPM0G3507_LineFollowing_Car.txt`

- [ ] **Step 1: 更新 README**

写明 X1/X8、2 ms/15位置、查表参数、PID融合公式、中心死区、三帧预测、单向找线、K1 启动、OLED 标签和 UniFlash 文件位置。

- [ ] **Step 2: 运行全量主机测试**

Run: `python -m unittest discover -s tests -p "test_*.py" -v`

Expected: 全部 PASS，记录总数。

- [ ] **Step 3: 编译前清理并完整重建**

```powershell
gmake -C MSPM0G3507_LineFollowing_Car clean
gmake -C MSPM0G3507_LineFollowing_Car -j4 all
gmake -C MSPM0G3507_LineFollowing_Car images
```

Expected: TI Arm Clang 和链接零 error，无新增 warning；必须先执行 clean。

- [ ] **Step 4: 验证 TI-TXT**

```powershell
$fw='dist\firmware\MSPM0G3507_LineFollowing_Car.txt'
Get-Item -LiteralPath $fw | Select-Object FullName,Length,LastWriteTime
Get-FileHash -Algorithm SHA256 -LiteralPath $fw
Get-Content -LiteralPath $fw -TotalCount 3
```

Expected: 文件非空、时间戳晚于源码修改、首行为 TI-TXT 地址记录，输出新 SHA-256。

- [ ] **Step 5: 完成需求审计**

逐项提供源码/测试/构建证据：固定接线、2 ms、15位置、防抖、动态平均/差速、PID仍活动、前馈已融合、三帧趋势、单向盲走、检测恢复、全程非负限幅、K1/软启动/看门狗未回退、新 TI-TXT。

- [ ] **Step 6: 提交 README**

```powershell
git add README.md
git commit -m "docs: explain PID lookup assisted line following"
```

不要提交 `build/`、根目录 `.obj`、`__pycache__` 或其他缓存。
