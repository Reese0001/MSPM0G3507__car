# MSPM0G3507 八路灰度循迹小车

本仓库是基于 TI MSPM0G3507、两轮差速底盘、L 型 520 减速电机、MPU6050 和八路灰度模块的裸机循迹小车工程。开发环境为 CCS Theia，编译器为 TI Arm Clang 4.0.4，工程不使用 RTOS。

## 主工程

CCS 工程目录：[`MSPM0G3507_LineFollowing_Car/`](MSPM0G3507_LineFollowing_Car/)

导入方法：

1. 在 CCS Theia 选择 **File → Import → Existing Projects into Workspace**。
2. 选择仓库中的 `MSPM0G3507_LineFollowing_Car` 目录。
3. 确认已安装 MSPM0 SDK 2.10.00.04 和 TI Arm Clang 4.0.4。
4. 执行 **Project → Clean**，然后 **Build Project**。

外设配置只修改 `MSPM0G3507_LineFollowing_Car/empty.syscfg`，不要手改构建目录中的 SysConfig 生成文件。

## 关键接线

| 功能 | MSPM0G3507 引脚 | 说明 |
|---|---|---|
| 调试串口 | PA10 TX / PA11 RX | 115200 |
| 电机驱动 UART | PB6 TX / PB7 RX | 115200，M2/M4 为驱动轮 |
| 灰度通道选择 | PA15 AD0 / PA16 AD1 / PA17 AD2 | 选择 X1～X8 |
| 灰度数字输出 | PA18 OUT | 当前通道电平 |
| MPU6050 软件 I2C | PA12 SCL / PA13 SDA | 地址 0x68 |

灰度模块不是通过 I2C 地址读取。当前代码按“OUT 高电平为白、低电平为黑”解释，实车前必须逐路确认极性和 X1～X8 左右顺序。

## 循迹控制

- 八路黑线位置使用 `-7,-5,-3,-1,+1,+3,+5,+7` 对称权重求平均。
- 第一阶段使用有界 PD，不启用积分。
- 偏差越大，基础速度越低。
- 正常循迹修正量被限制在基础速度的 80% 内，避免单侧车轮意外反转。
- X1～X8 权重描述传感器位置；`TRACKING_STEERING_POLARITY=-1` 根据本车实测反转控制器到实际底盘的角速度方向，不通过交换 M2/M4 修正循迹符号。
- 丢线前两个控制周期以低速按最后方向找线，第三个周期仍未检测到黑线则请求两轮零速。
- `LineWalking()` 不使用 100 ms 阻塞延时，主循环可以持续运行电机安全服务。

## 电机安全

- 所有速度请求必须经过 `Motor_Safety_RequestSpeed()`。
- 启动采用 0→30% 的 1000 ms soft-start。
- 200 ms 没有新的合法请求时，看门狗锁存故障并发送固定零速帧。
- M1/M3 保持零速，M2/M4 为左右驱动轮。

首次烧录与实车测试前：

```text
[ ] 首次烧录时断开 12.6 V 电机电源
[ ] MCU、灰度模块和电机驱动板共地
[ ] PB6/PB7 与驱动板 TX/RX 交叉关系已确认
[ ] PA15/PA16/PA17/PA18 接线已确认
[ ] 驱动轮架空，周围无人接触车轮
[ ] 可以立即切断电机电源
```

## 验证边界

自动测试：

```powershell
python -m unittest discover -s tests -v
```

离线测试和编译只能证明代码路径、配置合同及工具链兼容性。实际转向符号、传感器黑白极性、电机正方向和 PID 参数必须通过架空轮、低速封闭赛道逐项确认。

更多资料见 [`docs/README.md`](docs/README.md) 和 [`docs/notes/line-following-strategy-research.md`](docs/notes/line-following-strategy-research.md)。
