# MSPM0G3507 循迹小车项目 AI 交接说明

更新时间：2026-07-26
仓库：`D:\DevProject\MSPM0G3507__car`

> 旧版（2026-07-18）描述的 `fix/line-tracking-turn` 分支与 `app_irtracking.c` 未提交高风险改动的现场已成历史：相关工作已整合进 `main`，循迹代码已模块化重构。本文件描述当前现场。

## 1. 接手后先做什么

1. 完整阅读根目录 `CLAUDE.md`（已更新到当前架构），严格遵守电机安全和编码要求。
2. 执行 `git status --short --branch` 和 `git log --oneline -10`，确认现场后再动手。
3. 阅读当前进行中的计划：`docs/superpowers/plans/2026-07-26-freertos-oled-lookup-control.md`。
4. 涉及电机启动、串口电机协议、引脚或 SysConfig 的修改，先向用户给出 Checklist 并获得确认。

## 2. 当前 Git 状态（2026-07-26）

- 工作分支：`main`，与 `origin/main` 同步于 `98b94f0 fix: bind FreeRTOS handlers for TI Arm Clang`。
- 计划约定：直接在 `main` 上工作，每完成一个任务立即 commit 并 push `origin main`。
- 另有 worktree 分支 `codex/line-following-burn`（`.worktrees/line-following-burn`，停在 `2eb201e`），不要误删。

## 3. 当前进行中的工作

正在执行 `docs/superpowers/plans/2026-07-26-freertos-oled-lookup-control.md`（配套设计 `docs/superpowers/specs/2026-07-25-freertos-oled-control-design.md`），共 8 个任务：

1. **[已完成]** FreeRTOS 构建骨架：`FreeRTOSConfig.h`（1 kHz、纯静态分配）、`application/freertos/app_tasks.[ch]`、静态内核库 `freertos_kernel/`，`empty.c` 改为 `vTaskStartScheduler()` 启动（提交 `2446c6c`、`748e9c7`、`98b94f0`）。
2. 15 位置线位解码与去抖（`modules/line_tracking/line_position.[ch]`）。
3. 开环速度/差速查表（`modules/line_tracking/line_lookup_control.[ch]`）。
4. 非锁存的转角/丢线恢复（重写 `application/line_recovery.[ch]`）。
5. 感知/控制/安全/电机输出四个静态 FreeRTOS 任务。
6. PA10/PA11 SSD1306 OLED 诊断显示（UART0 调试停用）。
7. MPU6050 过弯 yaw 限制与故障诊断。
8. 可烧录固件构建与分阶段上车验收。

关键约束：1 ms 定时器 ISR 内的 `Motor_Safety_Tick1ms()` 与硬件安全逻辑保持不变；软启动 0→30%、命令限幅 ±450 保持；纯静态 FreeRTOS（无应用堆）；电机命令 5 ms 限频。

## 4. 工程和硬件概要

- MCU：TI MSPM0G3507；IDE：CCS Theia；工具链 TI Arm Clang 4.0.4；SDK 2.10.00.04；FreeRTOS 11.2.0 CM0+ 静态内核。
- CCS 工程：`MSPM0G3507_LineFollowing_Car/`；SysConfig 唯一真实来源：`empty.syscfg`。
- 架构：`application/`（应用层 + `config/` 参数头 + `freertos/`）、`modules/`（line_tracking 流水线、motor 含安全层、mpu6050、ybimu、ultrasonic、k230_link 等）、`bsp/`（时基、灰度选通、I2C、K230 串口）。`legacy_*` 目录为存档。
- 引脚：电机 UART1 PB6/PB7；灰度 PA15/PA16/PA17=AD0-2、PA18=OUT（低=黑，X1 最左）；MPU6050 软 I2C PA12/PA13；PA10/PA11 原 UART0 调试，计划改作 OLED。
- 电机命令必须经过 `Motor_Safety_RequestSpeed()`；禁止 100% 直出；M1/M3 恒零速，M2/M4 为驱动轮。

## 5. 构建与测试

离线测试（需 Windows + Visual Studio 2022，harness 用 MSVC 编译）：

```powershell
python -m unittest discover -s tests -v
```

CCS clean build（工作目录必须是 Debug）：

```powershell
Set-Location MSPM0G3507_LineFollowing_Car\Debug
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' clean
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -j4 all
```

不要手改 `Debug/ti_msp_dl_config.c`（SysConfig 生成文件）。

## 6. 烧录文件与 UniFlash

- CCS/XDS110：`Debug/MSPM0G3507_LineFollowing_Car.out`
- UniFlash 串口 BSL：TI-TXT `Debug/MSPM0G3507_LineFollowing_Car.txt`
- 修改源码后必须重新 clean build 再烧录，不能烧旧文件。
- PA18 同时用于灰度 OUT 和 BSL Invoke；进入 BSL/烧录时临时断开灰度模块 PA18。
- COM 口曾被 CCS/ccs-server 占用；烧录前关闭 CCS 串口监视器。

## 7. 禁止事项

- 不要绕过 `Motor_Safety_RequestSpeed()`，不要移除软启动/看门狗。
- 不要把未验证的高速度/高增益参数直接烧录到落地小车；实车顺序：断电检查 → 架空轮 → 低速封闭赛道。
- 不要交换 M2/M4 来掩盖循迹符号问题（用极性常量表达）。
- 不要引入非 UTF-8 文本（`test_text_encoding.py` 会失败）。
- 不要手改 SysConfig 生成文件；不要擅自强制重置或删除分支/worktree。

## 8. 给下一位 AI 的一句话摘要

`main` 已同步远端，FreeRTOS 迁移计划（2026-07-26）的 Task 1 骨架已完成，接下来按计划从 Task 2（15 位置线位解码）继续，改动前先跑离线测试、涉及电机/引脚先给 Checklist。
