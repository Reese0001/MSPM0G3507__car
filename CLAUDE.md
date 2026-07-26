# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> 本文件以 UTF-8 无 BOM 保存。请勿用 GBK/ANSI 编辑器另存，否则会再次乱码（旧版本即因此损坏）。
> 更新时间：2026-07-26。旧版描述的 `BSP/Task` + `BSP/Questions` 架构已重构，见下方"代码架构"。

## 项目概述

2026 电赛小车平台：基于 **TI MSPM0G3507**（Cortex-M0+, 80MHz）的嵌入式工程，使用 **TI Arm Clang** 工具链，在 **CCS Theia** 中开发。
硬件构成：MSPM0G3507 核心板 + 扩展板 + L 型 520 减速电机 ×2（两驱，M2/M4 为驱动轮）+ 八路灰度循迹（PA15~PA18 选通/读取）+ MPU6050 + YB-IMU + 超声波 + K230 视觉模块 + 12.6V 电源（电机驱动板 12V，MCU/传感器 3.3V）。电机经 **UART1 协议帧**驱动电机板，非直接 PWM。

固件工程在 `MSPM0G3507_LineFollowing_Car/`；`K230_Vision/` 为 K230 端 Python 代码；`tests/` 为离线测试套件。

**当前正在进行 FreeRTOS 迁移**：按 `docs/superpowers/plans/2026-07-26-freertos-oled-lookup-control.md` 执行（8 个任务：FreeRTOS 静态四任务、15 位置查表控制取代 PID、PA10/PA11 改作 SSD1306 OLED、非锁存丢线恢复）。Task 1（FreeRTOS 骨架 + 静态内核库 `freertos_kernel/`）已提交；`empty.c` 已改为 `vTaskStartScheduler()` 启动。

## 行为准则（务必遵守）

- **安全第一**：任何电机速度请求必须经过 `Motor_Safety_RequestSpeed()`（`modules/motor/motor_safety.c`）；启动为 0→30% 的 1000 ms soft-start，**禁止直接输出 100%**；200 ms 无合法请求时看门狗锁存故障并发零速帧；命令限幅 ±450。改动电机通信/启动逻辑前先在回复中给出 Checklist 确认。
- **不越权**：只依据已确认的硬件信息编码，缺信息时**先询问用户**。改动关键外设/接线/SysConfig 前先询问用户。
- **分步推进**：按 `计划（docs/superpowers/plans/）→ 失败测试 → 实现 → 离线测试全绿 → CCS 构建 → 架空轮低速实测` 循环推进；每完成一步向用户汇报。
- **不掩盖符号问题**：不要交换 M2/M4 来修循迹方向；转向极性用 `TRACKING_STEERING_POLARITY` / `LINE_STEERING_POLARITY` 常量表达。
- **技能沉淀**：可复用的排查流程写入 `PROJECT_SKILLS.md`。

## 构建与测试

### 离线测试（改动后必跑）

```powershell
python -m unittest discover -s tests -v
```

- 测试为 Python `unittest` + C host harness：`tests/test_*.py` 会调用 **MSVC（VsDevCmd.bat + cl.exe）** 编译 `tests/*_harness.c` 并运行断言，因此**只能在装有 Visual Studio 2022 的 Windows 上运行**。
- `tests/test_*_contract.py` 直接对源码文本做合同检查（架构、编码、SysConfig 等）。
- `test_text_encoding.py` 强制 `MSPM0G3507_LineFollowing_Car/` 与 `docs/` 下所有文本文件为**合法 UTF-8**。

### CCS 构建

- CCS Theia 中 **Project → Clean** 后 **Build Project**；或命令行（工作目录必须是 Debug）：

```powershell
Set-Location MSPM0G3507_LineFollowing_Car\Debug
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' clean
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -j4 all
```

- 工具链：`ti-cgt-armllvm 4.0.4`，`-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft`；SDK `mspm0_sdk_2_10_00_04`；SysConfig 1.26。
- FreeRTOS 静态内核库在 `freertos_kernel/`（TI Arm Clang 编译的 `Debug/freertos_kernel_ticlang.lib`）。

### 烧录

- CCS/XDS110（SWD）：烧 `Debug/MSPM0G3507_LineFollowing_Car.out`。
- UniFlash 串口 BSL：烧 TI-TXT（`.txt`）。注意 **PA18 同时是灰度 OUT 和 BSL Invoke**，烧录时临时断开灰度模块 PA18；COM 口可能被 CCS 占用，先关串口监视器。
- 改源码后必须重新 clean build 再烧录，不能用旧产物。

## 代码架构

### 启动流程

`empty.c: main()` → `SYSCFG_DL_init()` → `App_Main_Init()`（`application/app_main.c`）→ `AppTasks_Create()`（`application/freertos/app_tasks.c`，静态任务）→ `vTaskStartScheduler()`。任一步失败则 `Motor_Safety_Disarm()` 后死循环。

- **`empty.syscfg`** 是外设配置的**唯一真实来源**。改引脚/外设改此文件并重新生成，**不要手改** `Debug/ti_msp_dl_config.c`（生成产物）。

### `application/` — 应用层

- `app_scheduler.c` — 裸机时代的协作式调度器（FreeRTOS 迁移中逐步被静态任务取代）。
- `corner_maneuver.c` / `line_recovery.c` — 转角机动与丢线恢复状态机。
- `motion_primitives.c` — 运动原语；`safety_supervisor.c` — 安全监督。
- `freertos/app_tasks.[ch]` — FreeRTOS 静态任务创建（含 idle/timer task 静态内存回调）。
- `config/*.h` — 集中的可调参数（`line_control_config.h`、`line_recovery_config.h`、`safety_config.h` 等）。
- `legacy_questions/`、`legacy_task/` — 旧比赛状态机与旧调度器，仅存档。

### `modules/` — 功能模块

- **`line_tracking/`** — 循迹流水线：`line_scanner`（PA15~PA17 选通道、PA18 读 OUT，低电平=黑）→ `line_features` → `line_estimator` → `line_event_classifier` → `line_trend_detector` → `line_controller`（有界 PD，`LINE_STEERING_POLARITY=-1`）。FreeRTOS 计划将新增 `line_position`（15 位置解码）与 `line_lookup_control`（查表控制）。
- **`motor/`** — `app_motor.c` 运动学；`app_motor_usart.c`/`bsp_motor_usart.c` UART1 协议帧；`motor_adapter.c` 适配；**`motor_safety.c` 安全层（软启动/看门狗/限幅，勿绕过）**。`MOTOR_TYPE=5`（L 型 520），M1/M3 恒零速。
- **`mpu6050/`** — 非阻塞软件 I2C 读取，提供 yaw rate 供循迹融合限幅；`legacy_mpu6050/` 为旧 DMP 方案存档。
- **`ybimu/`** / **`ultrasonic/`** / **`k230_link/`** — YB-IMU 串口协议、超声波测距、K230 视觉链路（对应 `K230_Vision/protocol/frame.py`）。
- **`led/` `key/` `buzzer/` `common/`** — 指示灯、按键、蜂鸣器、公共状态类型（`module_status.h`）。

### `bsp/` — 板级支持

`bsp_line_mux`（灰度选通）、`bsp_i2c`、`bsp_k230_uart`、`bsp_ultrasonic`、`debug_uart`、`delay`、`time/`（1 ms 时基与电机看门狗计时，`Motor_Safety_Tick1ms()` 保持在定时器 ISR 中）。

### 引脚分配（当前）

| 外设 | 引脚 | 说明 |
|------|------|------|
| 电机驱动 UART1 | PB6 TX / PB7 RX | 115200，协议帧，M2/M4 驱动轮 |
| 八路灰度选通 | PA15 AD0 / PA16 AD1 / PA17 AD2 | 选择 X1~X8（X1 最左） |
| 八路灰度输出 | PA18 OUT | 低电平=黑线；与 BSL Invoke 复用 |
| MPU6050 软 I2C | PA12 SCL / PA13 SDA | 地址 0x68 |
| PA10 / PA11 | 调试 UART0 → SSD1306 OLED | FreeRTOS 计划中改作 OLED 软 I2C，UART0 调试停用 |
| LED D1/D2 | PB2 / PB3 | GPIO |
| 按键 K1 | PA2 | GPIO 输入 |
| 蜂鸣器 | PB24 | PWM (TIMA0) |

## 文档与约定

- **计划/设计**：每次迭代的实施计划在 `docs/superpowers/plans/`、设计在 `docs/superpowers/specs/`（按日期命名），是了解演进历史的首选入口。硬件核对清单在 `docs/hardware/`。
- **编码**：`MSPM0G3507_LineFollowing_Car/` 与 `docs/` 下所有文本（含 C 源码）必须是**合法 UTF-8**（由 `test_text_encoding.py` 强制）；Markdown 用 UTF-8 无 BOM。新增 C 代码注释用英文或 UTF-8 中文均可，但不得引入 GBK。
- **验证边界**：离线测试与编译只能证明代码路径和合同；转向符号、传感器黑白极性、电机正方向、PID/查表参数必须经"断电检查 → 架空轮 → 低速封闭赛道"逐项实测确认。
- **参考资料**：`docs/reference/`、`docs/docs_backup/` 为 2024 年前小车资料，仅供参考。
