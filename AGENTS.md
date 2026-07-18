# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

> 本文件以 UTF-8 无 BOM 保存。请勿用 GBK/ANSI 编辑器另存，否则会再次乱码（旧版本即因此损坏）。

## 项目概述

2026 电赛小车平台：基于 **TI MSPM0G3507**（Cortex-M0+）的裸机（no-RTOS）嵌入式工程，使用 **TI Arm Clang** 工具链，在 **CCS Theia** 中开发。
硬件构成：MSPM0G3507 核心板 + MSPMO 扩展板 + L 型 520 减速电机 ×2（两驱）+ MPU6050 姿态传感器 + 八路灰度循迹传感器 + 12.6V 电源（电机驱动板 12V，MCU/传感器 3.3V）。

工程主体在 `MSPM0G3507_LineFollowing_Car/`，由 TI 的 `empty` DriverLib 示例改造而来；旧目录名 `empty_LP_MSPM0G3507_nortos_ticlang` 仅保留在历史记录中。

## 行为准则（务必遵守）

- **安全第一**：涉及 12.6V 电源 / 电机驱动的代码，任何电机启动必须做 **Soft-start**（占空比 0→30% 渐进 ramp），**禁止直接输出 PWM 100%**；需带定时器中断的失控监控。改动电机通信前先在回复中给出 Checklist 确认。
- **不越权**：只依据已确认的硬件信息编码，不臆测扩展板/引脚超出范围的硬件。当前硬件仍在准备中，缺信息时**先询问用户**，确认后再进入下一阶段。
- **分步推进**：按 `硬件规划 → 编写前 Checklist 确认 → 模块编码 → 分阶段测试 → 调试算法` 循环。改动关键外设/接线前先询问用户，得到确认后方可继续。
- **注释规范**：优先使用 TI DriverLib `DL_` 前缀 API；关键寄存器/函数需中文注释说明用途（现有代码为 GBK 注释，见下方编码说明）。
- **每步汇报**：每完成一步向用户汇报进度；有需要沉淀的经验/技能写入 `PROJECT_SKILLS.md`（不存在相关 skill 时先询问用户是否新增）。

### 子代理分工（Sub-agents）
- **Hardware**：根据硬件信息推荐/校验接线与外设配置，确保安全。
- **C-Code**：MSPM0 寄存器 / DriverLib 函数编写。
- **Logic**：测试算法、PWM、PID 等控制逻辑。

## 构建与烧录

在 CCS Theia 中操作（非命令行工程）：

- **编译**：右键项目 → *Build Project*（底层调用 `Debug/` 下的 gmake makefile）。
- **烧录 / 运行**：右键项目 → *Run As → Code Composer Studio Application*，通过板载 XDS110 (SWD) 下载。
- **串口调试**：波特率 **115200**。`ccs-serial` MCP server（见 `.mcp.json`）可用于串口交互。

工具链与依赖（取自最近一次成功构建日志 `Debug/*_build.log`）：
- 编译器：`ti-cgt-armllvm 4.0.4`（tiarmclang），`-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O2`
- SDK：`mspm0_sdk_2_10_00_04`
- CCS：Theia 版 `css2051`（ccsVersion 70.5.1）

**重要路径提示**：项目最初在 `E:\workspace_ccstheia\...` 构建，现位于 `D:\DevProject\MSPM0G3507__car\...`。`.cproject`、`.mcp.json`、`docs/setup/SETUP_GUIDE.md` 和历史构建日志中仍可能残留 `E:\` 与 `E:\Software\ti\...` 绝对路径。若构建/串口失败，优先检查 SDK、编译器和工作区路径。

## 代码架构

### 启动与初始化流程
`empty.c: main()` → `SYSCFG_DL_init()`（由 SysConfig 生成）→ 各 BSP 模块 init → 主循环。

- **`empty.syscfg`** 是外设配置的**唯一真实来源**。修改引脚/外设应改此文件并重新生成，而非手改 `Debug/ti_msp_dl_config.c`（该文件为生成产物）。当前 `empty.c` 为 MPU6050 I2C 扫描测试版本，非最终主程序。

### 协作式调度器（`BSP/Task/`）
裸机无 RTOS，采用**时间片轮询调度**。`task.c` 中的 `tasks[]` 数组注册 `{interval_ms, last_call, 函数指针}`，`Scheduler_Run()` 在主循环反复调用，靠 `Get_Time()` 时间差触发（依赖定时器毫秒计数，`BSP/Timer/`）。新增周期任务 = 往 `tasks[]` 加一条。

### 任务状态机（`BSP/Questions/`）
`questions.c` 用嵌套状态机（`struct state_machine`：`Main_State` + `Q1..Q4_State`）实现比赛各题流程。按键切换题目（`STOP_STATE`/`QUESTION_1..4`），短按启动、长按切题。这是比赛逻辑的顶层编排。

### BSP 模块（`MSPM0G3507_LineFollowing_Car/BSP/`）
- **`Motor/`** — 两驱电机控制。`app_motor.c` 高层运动学（`Motion_Car_Control`、里程计 `Get_Odometry`，`Car_APB` 轴距常数）；`app_motor_usart.c`/`bsp_motor_usart.c` 通过 **UART 向电机驱动板发协议帧**（非直接 PWM）。`Set_Motor(MOTOR_TYPE)` 选电机型号，本项目 `MOTOR_TYPE=5`（L 型 520）。M2/M4 为驱动轮，M1/M3 为万向轮（空）。
- **`MPU6050/`** — 姿态传感器。`bsp_mpu6050.c` 软件 I2C 底层读写（`MPU6050_ReadData`）；`app_mpu6050.c` 上层角度/PID（`Dir_PID`、`dir_kp/dir_kd`）。
- **`eMPL/`** — InvenSense DMP 库（`inv_mpu.c` + DMP 固件），MPU6050 硬件姿态解算，勿随意改。
- **`Eight_Tracking/`** — 八路灰度循迹，I2C1 读取（`app_irtracking.c`，循迹 PID `IRTrack_Trun_KP/KI/KD`）。
- **`Key/` `LED/` `Buzzer/` `Timer/`** — 按键、指示灯、蜂鸣器（PWM/TIMA0）、毫秒计时基。
- **`usart.c` / `delay.c`** — UART0 调试打印（`USART_SendData`，支持 printf 重定向）与延时。

### 引脚分配（见 `docs/setup/SETUP_GUIDE.md`）
| 外设 | 引脚 | 说明 |
|------|------|------|
| UART0 调试 | PA10 TX / PA11 RX | 115200, printf |
| UART1 电机 | PB6 TX / PB7 RX | 电机驱动板 |
| I2C1 灰度 | PA15 SCL / PA16 SDA | 八路灰度 (地址 0x12) |
| MPU6050 | PA12 SCL / PA13 SDA | 软件 I2C (地址 0x68) |
| LED D1/D2 | PB2 / PB3 | GPIO |
| 按键 K1 | PA2 | GPIO 输入 |
| 蜂鸣器 | PB24 | PWM (TIMA0) |

## 文档与约定

- **参考资料**：精选资料位于 `docs/reference/` 和 `docs/hardware/`，保留/删除范围记录在 `docs/archive-manifest.md`。这些多为 2024 年前的小车源码/教程，仅供参考，本项目目标基于 2026 赛题。
- **编码**：现有 C 源码/头文件注释为 **GBK** 编码（Windows 中文），`AGENTS.md`、`PROJECT_SKILLS.md` 和 `docs/setup/SETUP_GUIDE.md` 为 UTF-8。编辑 C 文件时保持其原有编码，避免注释乱码。
- **技能沉淀**：可复用的工程规范/排查流程记录在 `PROJECT_SKILLS.md`（如 Motor-Safety-Layer、Debug-Protocol 排查顺序：SysConfig 引脚冲突 → 共地 → PWM 频率匹配）。
