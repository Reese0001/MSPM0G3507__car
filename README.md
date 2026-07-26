# MSPM0G3507 八路灰度循迹小车

本仓库是基于 TI MSPM0G3507、两轮差速底盘、L 型 520 减速电机、MPU6050、八路灰度模块和 SSD1306 OLED 的循迹小车工程。当前固件使用 FreeRTOS 11.2.0，通过四个静态任务完成传感器采样、循迹控制、安全与电机输出、诊断显示；应用层不使用动态内存。

八路灰度数据被解码为 `-7`～`+7` 的 15 个合法位置，正常循迹采用有界速度/差速查表。MPU6050 的 yaw rate 只用于抑制急弯中过大的转向，不参与 PID 控制。

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

`MSPM0G3507_LineFollowing_Car/empty.syscfg` 是外设配置的唯一真实来源。不要手改 `Debug/` 或 `Build_LineFollowing/` 中的 SysConfig 生成文件。

命令行复现构建：

```powershell
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C MSPM0G3507_LineFollowing_Car clean all
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C MSPM0G3507_LineFollowing_Car images
```

产物：

- CCS/XDS110：`MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.out`
- Intel HEX：`firmware/MSPM0G3507_LineFollowing_Car.hex`
- UniFlash TI-TXT：`firmware/MSPM0G3507_LineFollowing_Car.txt`

`firmware/` 不入库。修改源码后必须重新 clean build 并生成镜像，不能继续烧录旧文件。

## 当前接线

| 功能 | MSPM0G3507 引脚 | 说明 |
|---|---|---|
| 电机驱动 UART1 | PB6 TX / PB7 RX | 115200；M2/M4 为驱动轮 |
| SSD1306 OLED | PA10 SCL / PA11 SDA | 软件 I2C，7 位地址 `0x3C` |
| MPU6050 | PA12 SCL / PA13 SDA | 软件 I2C，地址 `0x68` |
| 灰度通道选择 | PA15 AD0 / PA16 AD1 / PA17 AD2 | 选择 X1～X8 |
| 灰度数字输出 | PA18 OUT | 低电平表示黑线；与 BSL Invoke 复用 |
| LED D1/D2 | PB2 / PB3 | 故障指示 / 心跳 |
| 按键 K1 | PA2 | GPIO 输入 |
| 蜂鸣器 | PB24 | TIMA0 PWM |

X1 是小车前进方向上的最左侧传感器，X8 是最右侧。灰度模块不是通过 I2C 地址读取；固件依次选通 X1～X8，再从 PA18 读取当前通道。

## UniFlash 烧录

首次烧录保持 12.6 V 电机电源断开，只连接 USB、MCU 和必要传感器。

1. 运行 `gmake ... images` 生成最新镜像。
2. 关闭 CCS 串口监视器及其他占用目标 COM 口的软件。
3. 在 UniFlash 中选择 MSPM0G3507 的串口 BSL 连接方式。
4. 加载 `firmware/MSPM0G3507_LineFollowing_Car.txt`（TI-TXT）。
5. 进入 BSL、执行下载并确认 Verify 成功。
6. 退出 BSL，重新上电运行。

PA18 同时连接灰度模块 OUT 和芯片 BSL Invoke。若无法进入 BSL，先断电，临时断开灰度模块的 PA18 信号线，完成烧录后再恢复。不要带电插拔。

## OLED 诊断页

固件已经包含 SSD1306 驱动和固定诊断页。正常启动后，128×64 OLED 每 200 ms 更新一次：

```text
LINE CAR DIAG
B:xxxxxxxx Tn
P:+n C:+n
L:+nnn R:+nnn
Sn Rn Mn
F:OK
```

- `B`：X1～X8 的黑线有效位，X1 显示在最左侧。
- `T`：当前灰度位型类别。
- `P/C`：稳定位置和候选位置。
- `L/R`：左右驱动轮命令。
- `S/R/M`：安全、丢线恢复和 MPU6050 状态。
- `F`：`OK` 或 `C-SEARCH`、`L-LOST`、`OLED-I2C`、`M-UART`、`CTRL-HB`、`SENS-HB`。

OLED 不显示时按以下顺序检查：

1. OLED 使用 3.3 V，且与 MCU 共地。
2. SCL 接 PA10，SDA 接 PA11，没有接反。
3. 模块地址是 `0x3C`；当前固件未扫描其他地址。
4. UniFlash 烧录的是刚生成的 `.txt`，而不是旧镜像。
5. 若页面曾出现后消失，检查是否显示 `OLED-I2C` 或总线接触不良。

OLED 初始化或通信失败属于非锁存故障，不会停止传感、控制和电机安全任务。

## 循迹控制

- 单个黑点或两个相邻黑点映射为 15 个合法位置：`-7`～`+7`。
- 从左到右移动黑线时，OLED 的稳定位置应从 `-7` 递增到 `+7`。
- 正常控制使用对称查表，偏差越大，基础速度越低、差速越大。
- 输出限制在 `±450`；MPU6050 过期时进一步限制速度。
- 普通丢线不会永久锁存 D1：先按最近稳定方向前探，再同向旋转搜索；搜索耗尽后停车。
- 电机输出由 SafetyTask 统一提交，UART 帧最快每 5 ms 一次，零速命令可立即发送。

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

当前软件基线已通过 192 项离线测试、TI Arm Clang clean build，以及 HEX/TI-TXT 地址与数据一致性检查。以上结果只证明代码路径、配置合同和工具链兼容性；实际转向符号、电机方向、传感器极性、控制参数及最大任务延迟仍必须通过分阶段实车验收确认。

更多资料见 [`docs/README.md`](docs/README.md)、[`docs/hardware/final-wiring.md`](docs/hardware/final-wiring.md) 和 [`PROJECT_SKILLS.md`](PROJECT_SKILLS.md)。
