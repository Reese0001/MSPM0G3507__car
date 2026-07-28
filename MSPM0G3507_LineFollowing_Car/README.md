# MSPM0G3507_LineFollowing_Car CCS 工程

这是仓库中唯一的 CCS 工程，直接在 CCS Theia 中导入本目录即可。

## 目录职责

- `empty.c`：硬件初始化、应用创建和 FreeRTOS 启动入口。
- `empty.syscfg`：引脚与外设配置的唯一真实来源。
- `app/`：启动、任务、邮箱和安全编排。
- `config/`：循迹、电机安全和传感器参数。
- `modules/`：按功能组织的显示、循迹、电机、MPU6050、时间和基础模块。
- `shared/`：跨模块共享的数据结构。
- `application/`：仍在使用的比赛状态机/运动原语兼容代码；新增功能优先放到 `app/` 或 `modules/`。

未启用的 K230、超声波、YB-IMU 和旧 MPU6050 代码统一保留在 `modules/optional/` 下，不在当前构建源列表中。

## CCS 构建

要求：

- CCS Theia
- TI Arm Clang 4.0.4 LTS
- MSPM0 SDK 2.10.00.04
- SysConfig 1.26.2

在 CCS 中执行 **Project → Clean**，再执行 **Build Project**。CCS/XDS110 产物位于：

```text
MSPM0G3507_LineFollowing_Car/Debug/MSPM0G3507_LineFollowing_Car.out
```

不要手改 `Debug/` 或 `build/cli/` 中的 SysConfig 生成文件；修改外设时只改 `empty.syscfg`，然后重新生成。

## 命令行构建与 UniFlash

在仓库根目录执行：

```powershell
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' `
-C MSPM0G3507_LineFollowing_Car rebuild
```

产物路径：

```text
build/cli/MSPM0G3507_LineFollowing_Car/MSPM0G3507_LineFollowing_Car.out
dist/firmware/MSPM0G3507_LineFollowing_Car.hex
dist/firmware/MSPM0G3507_LineFollowing_Car.txt
```

UniFlash 只选择并烧录 `dist/firmware/MSPM0G3507_LineFollowing_Car.txt`（TI-TXT）。烧录前保持 12.6 V 电机电源断开，Verify 成功后再进入下一阶段。

## 灰度与方向约定

面向车头时，X1 在右侧、X8 在左侧。PA15/PA16/PA17 选择通道，PA18 读取数字电平。

- X1/right → 位置 `+7`
- 中心 → 位置 `0`
- X8/left → 位置 `-7`

上车前先确认黑白电平、X1/X8 顺序和 M2/M4 正方向。未确认前只做断电测试或架空轮测试。

## OLED 运行日志与 RESET 启动

### 临时 FreeRTOS 启动诊断判读

下表仅适用于当前的**临时启动诊断固件**；确认根因并修复后必须删除这些诊断检查点，恢复 D1/D2 的原用途。

| OLED 停在 `SCHED START` 时的静态 D1:D2 | 含义 |
|---|---|
| `00` | 阶段 03：调度器的 port 启动前 trace hook 未到；断言会显示 `E1`，不会显示静态 `00`。 |
| `01` | 阶段 04：port 启动前 trace hook 已执行，但尚未进入 SVC。 |
| `10` | 已进入 SVC，但尚未开始首次上下文恢复。 |
| `11` | 已开始首次上下文恢复，但尚未到达任务 C 入口。 |

- D1 熄灭且 D2 重复 1～3 个 100 ms 短脉冲，并带明显长间隔：已有 1～3 个任务入口上线，任务位图尚未达到 `0x0F`。
- D1 常亮且 D2 重复闪烁 1～5 次：分别是致命码 `E1`～`E5`。
- D1 熄灭且 D2 以 250 ms 心跳：四个任务均已达到 `0x0F`。`TASK MASK 0F` 只记录一次，可能已从流动 OLED 日志中滚走，此后以 D2 心跳为准；随后继续观察 `SAFETY RUN`、`MOTOR ARMED` 和非零 `TX`，不要仅据此宣称电机已修复。

OLED 使用 PA10=SCL、PA11=SDA、地址 `0x3C`，作为流动行为日志；调试串口保持 115200。按下 RESET 后固件自动启动，K1 不是启动门，也不等待循迹有效帧才允许电机软启动。正常启动时 OLED 必须完整显示：

```text
0000 BOOT
0012 OLED OK
0020 AUTO START
0022 MOTOR CFG
0525 CFG OK
0526 SAFETY RUN
0526 MOTOR ARMED
0626 TX L030 R030
0726 TX L060 R060
```

第一次硬件测试：先断开或架空驱动轮，连接 12.6 V，按 RESET；在放下驱动轮前观察完整 OLED 日志。出现 `UART TIMEOUT`、`WATCHDOG`、`DIR WAIT` 或 `LINE LOST` 时，立即断开 12.6 V 并诊断，驱动轮落地时不要重复 RESET。OLED 不亮时检查 3.3 V、共地、PA10/PA11、地址和 UniFlash 的最新 `.txt`。

## 电机安全

- 所有速度请求必须经过 `Motor_Safety_RequestSpeed()`。
- 启动保持 0→30% soft-start，禁止直接输出 100%。
- 200 ms 没有合法请求时看门狗锁存故障并发送零速帧。
- M1/M3 保持零速，M2/M4 是左右驱动轮。

首次接通电机动力前，必须完成架空轮、共地、UART、X1/X8 方向、D2 心跳和 OLED 诊断检查，并确保可以立即切断 12.6 V 电源。
