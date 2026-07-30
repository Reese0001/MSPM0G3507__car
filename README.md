# MSPM0G3507 赛题循迹小车

本工程当前只实现 2026 电赛 H 题的循迹阶段：

- 小车放在 A 点停车基准线上；
- RESET 后电机保持零速；
- 按下 K1 启动计时并顺时针循迹；
- 完成一圈再次识别 A 点停车线后立即停车；
- OLED 保留总用时，便于检查“≤20 s”目标。

钢球平衡控制暂未接入本固件，避免它干扰循迹调试。

## 唯一运行链

```text
八路灰度（后台分时完整帧） + MPU6050 Z 轴角速度
                ↓
          LineFollower
                ↓
             Drive
                ↓
       UART 电机板（M2/M4）
```

正式固件不使用 FreeRTOS、mailbox、Yb 九轴 IMU、双控制模式或多套安全层。当前使用 MPU6050（PA12/PA13）进行非阻塞陀螺仪读取；主循环采用固定周期的裸机协作调度，只有 `Drive` 可以发送电机速度帧。

### 循迹规则

- 面向车头：X1 在右侧，X8 在左侧。
- X1～X8 权重为 `+7,+5,+3,+1,-1,-3,-5,-7`。
- 单点、相邻双点解码为 15 个合法位置；宽黑线按所有有效 X 通道加权求中心，保留左右方向。
- 启动首帧立即接受；运行中任何合法位置变化都需要连续两帧确认，减少直线小摆动。
- 主控制为灰度位置查表：中心命令 125，弯道逐级降速；同向快速外移时预减速，最终限制在 `0..140`。
- 多段噪声最多保持上一命令 20 ms，之后进入丢线处理。
- 丢线后沿最后偏差/趋势方向单向原地找线，不倒车、不左右来回摆动；首次就是全白时默认向右。
- 连续 3 帧重新检测到可信线后恢复正常循迹。

### IMU 辅助

MPU6050 使用地址 `0x68`，先写 `PWR_MGMT_1=0` 唤醒，再读取陀螺仪寄存器 `0x43..0x48`。当前只使用 Z 轴角速度：

```text
低通后的 gyro_z × 0.18 → 转向阻尼，死区 2°/s，限幅 ±24
```

IMU 不是启动条件。通信失败或数据过期时修正严格归零，灰度循迹继续运行：

- `IMU OFF`：没有新鲜数据；
- `IMU OK`：数据有效但处于死区；
- `IMU USED`：本帧产生了非零转向修正。

没有编码器可靠速度反馈，因此当前不使用速度 PID；也不使用绝对 yaw 锁定或 FOC。

## A 点计时与停车

A 点的启停线会让较多灰度通道同时变黑。固件用独立 `LapTracker` 识别：

1. K1 按下时开始计时，但忽略车底当前的起点标线；
2. 连续 3 帧离开宽线后确认已经驶离 A 点；
3. 运行至少 3 s 后，再连续 3 帧检测到 6 路及以上黑线，判定返回 A 点；
4. 电机请求立即归零，并冻结 OLED 总时间。

该逻辑对应赛题的 5 cm 停车基准线。停车偏差 ≤2 cm 仍需通过实车调整传感器前伸距离、制动惯性和返回线阈值。

## 接线

| 功能 | MSPM0G3507 引脚 | 说明 |
|---|---|---|
| 电机 UART | PB6 TX / PB7 RX | 115200；M2 右轮、M4 左轮，M1/M3 始终为零 |
| SSD1306 OLED | PA10 SCL / PA11 SDA | 软件 I2C，地址 `0x3C` |
| MPU6050 | PA12 SCL / PA13 SDA | VCC 接 3.3V、GND 共地、AD0 接 GND，地址 `0x68`；INT 暂不接 |
| 灰度选通 | PA15 AD0 / PA16 AD1 / PA17 AD2 | 依次选择 X1～X8 |
| 灰度 OUT | PA18 | 低电平表示黑线；与 BSL Invoke 复用 |
| K1 | PA2 | RESET 后按一次开始 |
| D1 / D2 | PB2 / PB3 | 故障 / 心跳 |

`empty.syscfg` 是引脚与外设配置的唯一真实来源，不要手改 `Debug/` 或 `build/cli/` 内的生成文件。

## OLED 诊断页

```text
RUN K1 SAFE / WAIT / STOP t
LINE Bxx P+x
REQ Lxxx Rxxx
OUT M2xxx M4xxx
MODE FOLLOW / SEEK L / SEEK R
IMU OFF / OK / USED
GYR +xxx COR +xx
ERR NONE / 错误原因
```

- RESET 后 `REQ`、`OUT` 都应为 0。
- K1 后中心线时 M2/M4 应平滑上升。
- `B00` 时应进入 `SEEK L/R`，不能静止卡死。
- 手动绕 Z 轴转动车体时 `GYR` 应变化；超过死区后显示 `IMU USED`。
- 断开 IMU 应显示 `IMU OFF`，但小车仍按灰度运行。
- `STOP` 后显示冻结的整圈时间，输出为零。

OLED 每 200 ms 更新一次诊断快照，并把每页拆成 16 字节小块发送；
每个 2 ms 调度周期最多发送一块，避免整页软件 I2C 阻塞循迹。

## 电机安全

- K1 是唯一启动门控。
- 1 s 内限制在目标的 30%，之后继续平滑达到目标。
- 每 5 ms 的输出变化最多 3 个命令单位，降低一冲一冲和机械抖动。
- 命令超过 50 ms 未更新，或 1 ms 看门狗达到 200 ms，立即发送零速。
- 换向先归零并等待 120 ms。
- UART 超时锁定零速，RESET 后重新初始化。

第一次接通 12.6 V 前：

```text
[ ] 电机轮已架空
[ ] MCU、灰度、OLED、MPU6050、电机板已共地
[ ] M2 是右轮，M4 是左轮
[ ] 可以随时切断 12.6 V
```

## 干净构建与 UniFlash

工程使用 TI Arm Clang 4.0.4、MSPM0 SDK 2.10.00.04 和 SysConfig 1.26.2。

```powershell
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C MSPM0G3507_LineFollowing_Car clean
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C MSPM0G3507_LineFollowing_Car -j4 all
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C MSPM0G3507_LineFollowing_Car images
```

UniFlash 只加载：

```text
dist/firmware/MSPM0G3507_LineFollowing_Car.txt
```

烧录时先断开 12.6 V。PA18 与 BSL Invoke 复用；若无法进入 BSL，断电后临时拔掉灰度 OUT，烧录完成再恢复，禁止带电插拔。

## 测试

```powershell
python -m unittest discover -s tests -p 'test_*.py' -v
```

软件测试和 clean build 只能证明数据流、限幅、门控和工具链正确。首次落地仍应按“架空轮 → 低速直线 → 弯道 → 丢线 → 整圈停车”的顺序验收。
## PWM tuning

Tune the line-following PWM only at the top of
`modules/line_tracking/line_follower.c`:

| Constant | Current value | Purpose |
|---|---:|---|
| `LINE_STRAIGHT_COMMAND` | 125 | Center-line base PWM. Increase by 5 only after the straight is stable. |
| `LINE_CURVE_COMMAND` | 105 | Base PWM for medium curves. Reduce first if the car enters a bend too fast. |
| `LINE_TURN_COMMAND` | 75 | Base PWM for sharp turns. Increase by 5 only if it lacks turning authority. |
| `LINE_ENTRY_BRAKE_COMMAND` | 14 | Extra braking when the line position expands outward on the same side. |
| `LINE_TURN_SLEW_STEP` | 24 | Maximum target differential change per 2 ms line frame. |
| `DRIVE_SLEW_STEP` | 3 | Maximum applied PWM change per 5 ms motor output frame. Lower is smoother. |

Tune in this order: straight speed, curve speed, entry braking, then sharp-turn
speed. Change one value at a time. Do not add an integral term to line following.
