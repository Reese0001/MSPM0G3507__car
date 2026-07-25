# 直角转弯接管丢线恢复设计

## 问题

实车在直角转弯中已经重新检测到黑线，两个循迹指示灯亮起，随后停车并出现
D1 常亮、D2 闪烁。

状态机进入 `CORNER_MANEUVER_SETTLE` 后，直角事件仍由
`LineEventClassifier` 锁存。若车身惯性或摆动让黑线短暂离开传感器：

- `LineController_Step()` 会把 `follow.valid` 置为 `false`；
- 锁存的直角事件使 `path_event->type` 仍不是 `LINE_PATH_LOST`；
- `SETTLE` 中原有的丢线退出分支无法触发；
- 随后的 `set_follow_request()` 失败并进入永久故障。

因此，一次可恢复的接管抖动被错误升级为 D1 常亮的永久停车。

## 方案

在 `CORNER_MANEUVER_SETTLE` 中，把短暂无效的循迹控制视为“接管失败，需要继续
找线”，而不是永久故障：

1. 若 `path_event` 明确报告丢线，或 `follow` 为空/无效：
   - 清零重新捕获帧计数；
   - 回到 `CORNER_MANEUVER_SEEK`；
   - 保持本次直角的原转向并继续原地寻线。
2. 再次获得连续三帧可靠黑线后重新进入 `SETTLE`。
3. 保留 `CORNER_TOTAL_TIMEOUT_MS = 2000U` 总超时；持续无法稳定接管时仍安全停车。

不修改速度、KP、转向方向、寻线方式、电机安全层或硬件配置。

## 验证

在现有 `corner_maneuver_harness.c` 增加实车对应序列：

1. 识别直角并进入 `SEEK`；
2. 连续三帧找到黑线并进入 `SETTLE`；
3. 模拟直角事件仍锁存、但 `follow.valid == false`；
4. 断言状态回到 `SEEK`、继续输出原方向寻线命令且不进入故障；
5. 再次连续三帧找到黑线，完成 `SETTLE`；
6. 保持现有总超时与左右镜像测试通过。

随后运行全部宿主测试、TI Arm Clang 编译链接，并重新生成 UniFlash HEX/TI-TXT。
