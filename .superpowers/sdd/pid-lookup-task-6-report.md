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
