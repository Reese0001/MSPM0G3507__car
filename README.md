# MSPM0G3507 八路灰度循迹小车

本仓库是基于 TI MSPM0G3507、两轮差速底盘、L 型 520 减速电机、MPU6050、八路灰度模块和 SSD1306 OLED 的循迹小车工程。当前固件为裸机协作式时间片调度，不依赖 FreeRTOS：安全、传感、控制和显示任务由固定周期表驱动，应用层不使用动态内存。

八路灰度数据每 2 ms 采样并解码为 `-7`～`+7` 的 15 个合法位置。正常循迹以查表产生基础速度和差速前馈，再由“位置外环 + MPU6050 航向角内环”辅助修正；查表、PID 和电机安全层职责分开。

## 主工程与环境

CCS 工程目录：[`MSPM0G3507_LineFollowing_Car/`](MSPM0G3507_LineFollowing_Car/)

- CCS Theia：`ccs2050`
- TI Arm Clang：4.0.4 LTS
- MSPM0 SDK：2.10.00.04
- SysConfig：1.26.2
- 串口电机协议：115200

在 CCS Theia 中导入：

1. 选择 **File → Import → Existing Projects into Workspace**。
2. 选择仓库中的 `MSPM0G3507_LineFollowing_Car` 目录。
3. 确认 MSPM0 SDK 和 TI Arm Clang 版本正确。
4. 执行 **Project → Clean**，然后 **Build Project**。

`MSPM0G3507_LineFollowing_Car/empty.syscfg` 是外设配置的唯一真实来源。不要手改 `Debug/` 或 `build/cli/` 中的 SysConfig 生成文件。

命令行复现构建：

```powershell
 & 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C MSPM0G3507_LineFollowing_Car rebuild
```

产物：

- CCS/XDS110：`MSPM0G3507_LineFollowing_Car/Debug/MSPM0G3507_LineFollowing_Car.out`
- CLI：`build/cli/MSPM0G3507_LineFollowing_Car/MSPM0G3507_LineFollowing_Car.out`
- Intel HEX：`dist/firmware/MSPM0G3507_LineFollowing_Car.hex`
- UniFlash TI-TXT：`dist/firmware/MSPM0G3507_LineFollowing_Car.txt`

`build/` 和 `dist/` 不入库。修改源码后必须重新 clean build 并生成镜像，不能继续烧录旧文件。

## 当前接线

| 功能 | MSPM0G3507 引脚 | 说明 |
|---|---|---|
| 电机驱动 UART1 | PB6 TX / PB7 RX | 115200；M2/M4 为驱动轮 |
| SSD1306 OLED | PA10 SCL / PA11 SDA | 软件 I2C，7 位地址 `0x3C` |
| MPU6050 | PA12 SCL / PA13 SDA | 软件 I2C，地址 `0x68` |
| 灰度通道选择 | PA15 AD0 / PA16 AD1 / PA17 AD2 | 选择 X1～X8 |
| 灰度数字输出 | PA18 OUT | 低电平表示黑线；与 BSL Invoke 复用 |
| LED D1/D2 | PB2 / PB3 | 故障指示 / 心跳 |
| 按键 K1 | PA2 | GPIO 输入；RESET 后按下 K1 才允许运行 |
| 蜂鸣器 | PB24 | TIMA0 PWM |

面向车头时，X1 在右侧、X8 在左侧。灰度模块不是通过 I2C 地址读取；固件依次选通 X1～X8，再从 PA18 读取当前通道。

## UniFlash 烧录

首次烧录保持 12.6 V 电机电源断开，只连接 USB、MCU 和必要传感器。

1. 运行 `gmake ... rebuild`，先清理缓存再生成最新镜像。
2. 关闭 CCS 串口监视器及其他占用目标 COM 口的软件。
3. 在 UniFlash 中选择 MSPM0G3507 的串口 BSL 连接方式。
4. 只加载并烧录 `dist/firmware/MSPM0G3507_LineFollowing_Car.txt`（TI-TXT）。
5. 进入 BSL、执行下载并确认 Verify 成功。
6. 退出 BSL，重新上电运行。

PA18 同时连接灰度模块 OUT 和芯片 BSL Invoke。若无法进入 BSL，先断电，临时断开灰度模块的 PA18 信号线，完成烧录后再恢复。不要带电插拔。

## OLED 运行日志与启动

OLED 是启动和运行时的流动行为日志，不是固定诊断页。RESET 后完成外设和电机配置，但保持零速；按下 K1 只解除运行门控，没有有效循迹请求时仍保持停车。D1 常亮表示故障，D2 正常以 250 ms 周期心跳。常见日志顺序如下，时间戳会随硬件响应变化：

```text
0000 BOOT
0012 OLED OK
0022 MOTOR CFG
0525 CFG OK
.... SAFETY TASK
.... SENSOR FRAME
PRESS K1
.... MOTOR ARMED
.... SAFETY RUN
.... CONTROL REQ
.... TX Lxxx Rxxx
```

首次硬件测试必须先断开或架空驱动轮，再接通 12.6 V，按 RESET，配置完成后按 K1，并在放下驱动轮前观察 OLED 完整显示上述日志。OLED 使用 3.3 V 并与 MCU 共地；SCL 接 PA10、SDA 接 PA11，地址为 `0x3C`。调试串口保持 115200。

若 OLED 出现 `UART TIMEOUT`、`WATCHDOG`、`DIR WAIT` 或 `LINE SAFE STOP`，立即断开 12.6 V 电源并诊断；`LINE SEEK L/R` 是正常单向找线状态。驱动轮落地时不得反复按 RESET。OLED 不亮时先检查 3.3 V、共地、PA10/PA11 方向、地址和 UniFlash 是否烧录了刚生成的 `.txt`。

## 循迹控制

- 单个黑点或两个相邻黑点映射为 15 个合法位置：`-7`～`+7`。
- 面向车头从左侧 X8 移向右侧 X1 时，稳定位置应从 `-7` 递增到 `+7`。
- 查表前馈按偏差动态降低基础速度并增大左右轮差速：中心最高 140，边缘最低 60；轮速在进入电机层前限制为 `0..140`。
- 位置外环保留 `Kp=14.0`、`Kd=0.010`，在查表前馈基础上生成目标转向；中心死区避免直线左右小幅摆动。
- 角度内环使用 MPU6050 的 Kalman 航向与角速度修正转向，参数保留 `Kp=1.5`、`Kd=0.55`；MPU 数据过期时自动旁路，不阻塞循迹。
- 航向目标相对当前航向限制在 `±20°`，避免车辆被挡住或安全层停机时积分饱和。
- 前进速度按灰度偏差和角速度共同规划：直线最高 140，普通弯约 110，急弯和边缘位置降至 55～90。
- 前进速度每个控制周期最多增加 4，转向每周期最多变化 10；紧弯时允许内轮降至零速，但不会由循迹环直接反转。
- 输出限制在 `±450`；MPU6050 过期时自动退化为位置环，不阻塞运行。
- 控制器保存最近 3 个合法位置，丢线时根据位置及变化趋势锁定左或右方向；随后只沿该方向原地旋转找线，不倒车、不来回摆动、没有普通丢线总超时。
- 找到连续 3 个可信新帧后进入 300 ms 低速对齐，再恢复正常循迹；只有急停、传感器数据过期或非法状态才进入 `LINE SAFE STOP`。
- 丢线/无效帧会清空旧航向目标和输出斜坡；重新找到线后从低速重新爬升。
- 赛道不写死尺寸或固定路线：长圆弧、直角、折线、梯形边和回头弯统一按局部偏差、角速度与丢线恢复处理。
- 电机输出由安全运行层统一提交，UART 帧最快每 5 ms 一次，零速命令可立即发送。

## 时间片调度

任务表按安全 → 传感 → 控制 → 显示的固定顺序运行：

- 安全：1 ms，唯一拥有电机输出权限。
- 传感：2 ms，扫描灰度；每 10 ms 启动或收取一次 MPU6050 事务。
- 控制：有新传感帧且 K1 已启动时运行一次。
- 显示：10 ms，每次只发送一页流动日志；避免 OLED 刷屏阻塞传感和电机控制。

MPU6050 软件 I2C 由主循环每轮只推进一个状态，不在传感任务内忙等。若主循环曾被阻塞，调度器跳过过期时间片，不突发补跑旧任务。

OLED 会显示 `IMU START/CAL/READY/DEG`。进入运行后每 500 ms 显示 `U Y±xxx G±xxx`：`U` 表示本控制帧实际使用了 MPU6050 角度环，`B` 表示当前旁路；正常循迹时 `Y` 是 Kalman 航向角，找线时改为 `D` 并显示相对丢线角度，`G` 是滤波后的 Z 轴角速度。

## 电机安全

- 所有速度请求必须经过 `Motor_Safety_RequestSpeed()`。
- 启动采用 0→30% 的 1000 ms soft-start，禁止直接输出 100%。
- 200 ms 没有新的合法请求时，看门狗锁存故障并发送固定零速帧。
- M1/M3 始终保持零速，M2/M4 为左右驱动轮。
- 电机 UART、控制任务或传感任务失联会锁存故障并点亮 D1。

首次接通电机动力前：

```text
[ ] 12.6 V 当前断开，USB-only 诊断已通过
[ ] MCU、灰度模块、OLED、MPU6050 和电机驱动板共地
[ ] PB6→驱动 RX、PB7←驱动 TX 已确认
[ ] PA15/PA16/PA17/PA18 接线和 X1～X8 顺序已确认
[ ] M2/M4 对应左右驱动轮，M1/M3 不使用
[ ] 驱动轮已架空，周围无人且无线缠绕物
[ ] 可以立即切断 12.6 V 电机电源
```

验收顺序不得跳级：

```text
USB-only 诊断 → 架空轮 → 低速落地 → 丢线测试 → 直角/急弯 → 全程
```

详细判据和结果记录见 [`docs/verification/sensor-platform-test-record.md`](docs/verification/sensor-platform-test-record.md)。

## 软件验证

```powershell
python -m unittest discover -s tests -v
```

当前软件基线已通过离线测试和 TI Arm Clang clean build。以上结果只证明代码路径、配置合同和工具链兼容性；实际转向符号、电机方向、传感器极性、控制参数及最大任务延迟仍必须通过分阶段实车验收确认。

更多资料见 [`docs/README.md`](docs/README.md)、[`docs/hardware/final-wiring.md`](docs/hardware/final-wiring.md) 和 [`PROJECT_SKILLS.md`](PROJECT_SKILLS.md)。
