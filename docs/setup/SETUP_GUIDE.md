# MSPM0G3507 小车开发与烧录指南

## 1. 唯一 CCS 工程

仓库只保留一个活动 CCS 工程：

```text
MSPM0G3507_LineFollowing_Car/
```

在 CCS Theia 中选择 **File → Import → Existing Projects into Workspace**，导入这个目录。外设配置只修改：

```text
MSPM0G3507_LineFollowing_Car/empty.syscfg
```

不要手改 `Debug/`、`build/cli/` 或 `dist/` 中的生成文件。

## 2. 当前接线

| 功能 | 引脚 | 说明 |
|---|---|---|
| 调试 UART0 | PA10 TX / PA11 RX | 115200 |
| 电机 UART1 | PB6 TX / PB7 RX | 115200 |
| OLED SSD1306 | PA10 SCL / PA11 SDA | 软件 I2C，`0x3C` |
| YbImu 九轴 IMU | PA1 SCL / PA0 SDA | 3.3V、共地，软件 I2C，`0x23` |
| 灰度选通 | PA15 AD0 / PA16 AD1 / PA17 AD2 | X1～X8 |
| 灰度输出 | PA18 OUT | 黑线有效电平由配置定义 |
| LED D1/D2 | PB2 / PB3 | 故障 / 心跳 |
| 按键 K1 | PA2 | GPIO 输入；RESET 后按下 K1 才运行 |
| 蜂鸣器 | PB24 | TIMA0 PWM |

面向车头时，X1 在右侧、X8 在左侧。软件位置约定为：

```text
X8/左侧  → -7
中心      →  0
X1/右侧  → +7
```

## 3. 软件验证

仓库根目录执行：

```powershell
python -m unittest discover -s tests -p "test_*.py"
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' `
-C MSPM0G3507_LineFollowing_Car rebuild
```

构建产物：

```text
CCS/XDS110:
MSPM0G3507_LineFollowing_Car/Debug/MSPM0G3507_LineFollowing_Car.out

CLI:
build/cli/MSPM0G3507_LineFollowing_Car/MSPM0G3507_LineFollowing_Car.out

UniFlash TI-TXT:
dist/firmware/MSPM0G3507_LineFollowing_Car.txt

HEX:
dist/firmware/MSPM0G3507_LineFollowing_Car.hex
```

## 4. UniFlash 烧录

1. 断开 12.6 V 电机电源，只保留 USB、MCU 和必要传感器。
2. 运行上面的 `gmake ... rebuild`（先清理缓存）。
3. 在 UniFlash 中选择 MSPM0G3507 的串口 BSL。
4. 只加载并烧录 `dist/firmware/MSPM0G3507_LineFollowing_Car.txt`。
5. 下载并执行 Verify。
6. 退出 BSL，重新上电。

PA18 与灰度 OUT 和 BSL Invoke 复用。无法进入 BSL 时，先断电并临时断开灰度模块 PA18 信号线，烧录后恢复。

## 5. 上电检查顺序

按 RESET 后固件完成配置并保持零速；按下 K1 只解除运行门控，灰度循迹尚未产生有效请求时电机仍保持零速。OLED 使用 PA10=SCL、PA11=SDA、地址 `0x3C`，调试串口保持 115200。典型流动日志为：

```text
0000 BOOT
0012 OLED OK
0022 MOTOR CFG
0525 CFG OK
PRESS K1
.... MOTOR ARMED
.... SAFETY RUN
.... CONTROL REQ
.... Bxx P+n DR
.... G+000 C+000 I0
.... CMD lll/rrr
.... TX Lxxx Rxxx
```

`Bxx` 是八路灰度位图，`P` 是位置，`D` 是锁定的找线方向；`G` 是 YbImu Z 轴角速度，`C` 是阻尼修正，`I1/I0` 表示本帧使用/未使用阻尼。`IMU BYPASS` 只表示 IMU 数据无效或超过 50 ms，灰度循迹继续运行。`LINE SEEK L/R` 表示车辆按最后可靠方向单向找线，不倒车、不换向；`LINE ALIGN` 表示重新找到线后的低速对齐。

默认控制模式在 `config/line_following_profile.h`：

```c
#define LINE_FOLLOWING_CONTROL_MODE (LINE_CONTROL_MODE_OFFICIAL_BASELINE)
```

该模式只用 `gyro_rad_s[2]` 做转向阻尼，不使用磁航向、绝对 yaw 或四元数。若绕 Z 轴转动车身而 `G` 不变化，先检查 PA1/PA0、地址 `0x23` 和 `YBIMU_BODY_Z_SIGN`，不要通过增加停车条件掩盖接线或方向问题。

第一次接通电机动力前必须确认：

```text
[ ] MCU、传感器、OLED、电机驱动板共地
[ ] PB6/PB7 UART 方向正确
[ ] M2/M4 为左右驱动轮，M1/M3 保持零速
[ ] soft-start 和 200 ms 看门狗已启用
[ ] 输出上限不超过 ±450
[ ] 可以立即切断 12.6 V 电源
```

第一次硬件测试必须先断开或架空轮（驱动轮悬空），再连接 12.6 V，按 RESET，并在放下驱动轮前观察完整 OLED 日志。若显示 `UART TIMEOUT`、`WATCHDOG`、`DIR WAIT` 或 `LINE LOST`，立即断开 12.6 V 并诊断；驱动轮落地时不得重复 RESET。

详细验收记录见 [`docs/verification/sensor-platform-test-record.md`](../verification/sensor-platform-test-record.md)。
