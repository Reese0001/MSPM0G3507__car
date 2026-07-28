# Task 6 活动链集成报告

## RED

先更新直接合同及 recovery harness，随后执行：

```powershell
python -m unittest tests.test_line_motion_contract tests.test_oled_contract tests.test_fault_diagnostics tests.test_app_scheduler tests.test_line_lookup_control tests.test_line_cascade_control tests.test_line_direction_predictor tests.test_line_recovery -v
```

结果：共 32 项，4 项失败、1 项错误。失败均为预期缺口：活动文件尚未包含 predictor/lookup，新模块未进入 Makefile/.cproject，OLED 仍使用旧 F/ROT 标签，pipeline 顺序缺少 predictor/lookup。

## Focused GREEN

完成生产重构后再次执行同一 focused 命令：32/32 通过，耗时 24.029 秒，无编译 warning。

## 全量验证

按约束只执行一次：

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

结果：226/226 通过，耗时 99.115 秒，无编译 warning。

## 实现与自审

- `AppLineMotion_BuildRequest` 现在只有 predictor -> lookup -> cascade -> recovery 一条活动数据流；lookup、cascade、recovery 调用各一次。
- 仅 POSITION 记录 predictor 历史；POSITION/WIDE 执行 lookup；LOST 保持 lookup invalid；NOISE 直接 invalid 并重置 cascade。
- 初始化与 IMU startup hold 均重置要求的动态状态；活动文件中 `LineTrendResult`、`PositionSign` 和旧 recovery 签名计数为 0。
- Makefile 中 lookup、cascade、predictor 源文件各出现一次，并补齐 Makefile/.cproject prediction include path；新模块未加入 exclusion。
- OLED 使用 L/R/ALIGN/SAFE STOP/FOLLOW 标签，保留 PID U/B 标记；SEEK 显示 recovery diagnostics 的 yaw delta，FOLLOW 显示绝对 yaw。
- 未新增 wrapper、feature flag、并行算法或赛题状态；未修改电机、安全、K1、MPU6050 底层或传感器调度生产代码。
- `git diff --check` 通过。触碰文件中的前序未提交运行/安全改动按要求完整保留。

## 提交范围

- 生产：`app/line/line_motion.c`、`app/log/runtime_observer.c`、`Makefile`、`.cproject`
- 直接迁移：`test_line_motion_contract.py`、`test_oled_contract.py`、`test_fault_diagnostics.py`、`test_app_scheduler.py`、`recovery_reachability_harness.c`
- 陈旧直接断言迁移：`test_runtime_observer_contract.py`、`test_line_features.py`
- 本报告

## 非本任务失败

无。

---

## 审查修复与依赖闭合

### 审查 RED

先为三项审查问题更新 `test_line_motion_contract.py`、`test_oled_contract.py` 和 `test_runtime_observer_contract.py`，执行：

```powershell
python -m unittest tests.test_line_motion_contract tests.test_oled_contract tests.test_runtime_observer_contract -v
```

结果：11 项中 3 项失败，分别证明 pattern 枚举未做边界保护、IMU startup hold 位于 NOISE 早退之后、OLED 仍使用会被时间戳挤占的 17 字符旧模板。

### 审查修复

- pattern 在索引 `event_by_pattern` 前以 unsigned 上界检查 fail-closed，并重置 cascade 动态状态。
- IMU startup hold 检查及位置/predictor/recovery/cascade 重置移动到 NOISE 和非法 pattern 早退之前。
- IMU OLED 模板缩短为 `U Y+000 G+000`；`payload[0]` 写 U/B，`payload[2]` 写 Y/D，数值从 3 和 9 开始，保证时间戳后仍完整显示。
- 未增加新算法、wrapper、feature flag 或赛题状态。

### 依赖闭合提交

1. `1d257bb chore: checkpoint native runtime prerequisites`
   - 活动生产：`control_runtime.c`、`run_controller.c/.h`、`safety_runtime.c`、`app_tasks.c`、`motor_adapter.c`、`motor_safety.c`。
   - MPU：`mpu6050.c/.h/.config.h`、新增 `mpu6050_kalman.c/.h`。
   - 直接验证：MPU/Kalman、时间片调度、K1 gate、安全桥和 M2/M4 映射对应的 harness/tests，共 17 个测试文件。
2. 本提交 `fix: harden Task 6 active runtime`
   - `line_motion.c`、`runtime_observer.c`、三份直接合同测试和本报告。

闭合依据：Makefile 已引用的 Kalman 源文件及其头文件已纳入 Git；`Mpu6050Snapshot.yaw_angle_deg` 的生产定义和实现已纳入；已提交 Task 6 合同依赖的 run/control/safety/task/motor 当前实现均已纳入。活动生产代码中 `BSP_I2C_Service` 仅在 `app/tasks/app_tasks.c` 轮询调用，`line_motion.c` 不再重复服务。checkpoint 后剩余 dirty 生产文件仅为本次 `line_motion.c`/`runtime_observer.c` 修复；README/docs、`.claude`、旧报告、文档测试与 optional FOC 均明确排除。

### 审查 GREEN 与全量

相关 focused 命令覆盖 line motion、OLED、runtime observer、scheduler、MPU/Kalman、run/safety/motor：83/83 通过，耗时 27.978 秒，无编译 warning。首次 focused GREEN 尝试仅有一项合同因把换行后的 C guard 当作单行字面量而误报；修正测试匹配后生产代码无需再次修改。

按要求只执行一次全量：

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

结果：227/227 通过，耗时 96.914 秒，无编译 warning；非本任务失败为 0。
