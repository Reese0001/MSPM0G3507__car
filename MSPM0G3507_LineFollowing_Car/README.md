# MSPM0G3507_LineFollowing_Car CCS 工程

这是仓库中唯一的 CCS 工程，直接在 CCS Theia 中导入本目录即可。

## 目录职责

- `empty.c`：硬件初始化和裸机时间片轮询入口。
- `empty.syscfg`：引脚与外设配置的唯一真实来源。
- `app/`：启动、任务、邮箱和安全编排。
- `config/`：循迹、电机安全和传感器参数。
- `modules/`：按功能组织的显示、循迹、电机、MPU6050、时间和基础模块。
- `shared/`：跨模块共享的数据结构。
- `modules/optional/`：未接入当前构建的旧版/比赛参考代码，需要用时再显式调用。

当前构建不再使用 `application/` 目录；启动、循迹、控制、安全和日志均从 `app/` 进入，再调用 `modules/` 中的单独功能模块。未启用的 K230、超声波、YB-IMU、旧 MPU6050、旧循迹和比赛运动原语统一保留在 `modules/optional/` 下，不在当前构建源列表中。

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

## OLED 运行日志与 K1 启动

OLED 使用 PA10=SCL、PA11=SDA、地址 `0x3C`，作为流动行为日志。RESET 后完成配置并保持零速，按下 K1 只解除运行门控；有效循迹请求尚未产生时电机仍保持零速。D1 常亮表示故障，D2 正常以 250 ms 周期心跳。典型日志为：

```text
0000 BOOT
0012 OLED OK
0022 MOTOR CFG
0525 CFG OK
PRESS K1
.... MOTOR ARMED
.... SAFETY RUN
.... CONTROL REQ
.... IMU READY
.... IMU U Y+000 G+000
.... TX Lxxx Rxxx
```

`U Y...` 表示角度环正在使用 MPU6050，`B Y...` 表示暂时旁路。丢线时根据最近 3 个合法位置锁定方向，显示 `LINE SEEK L` 或 `LINE SEEK R`，只沿该方向原地旋转，不倒车也不来回摆动；连续 3 帧重新找到线后显示 `LINE ALIGN`，低速对齐 300 ms。只有急停、传感器数据过期或非法状态才显示 `LINE SAFE STOP`。

正常循迹每 2 ms 读取八路灰度并解码为 `-7..+7`。查表输出基础速度/差速前馈，位置 PD（`Kp=14.0`、`Kd=0.010`）和 MPU6050 角度环（`Kp=1.5`、`Kd=0.55`）只做辅助修正；最终命令仍由软启动、限幅和失控监控统一约束。

第一次硬件测试：先断开或架空驱动轮，连接 12.6 V，按 RESET，配置完成后按 K1；在放下驱动轮前观察完整 OLED 日志。出现 `UART TIMEOUT`、`WATCHDOG`、`DIR WAIT` 或 `LINE SAFE STOP` 时，立即断开 12.6 V 并诊断，驱动轮落地时不要重复 RESET。OLED 不亮时检查 3.3 V、共地、PA10/PA11、地址和 UniFlash 的最新 `.txt`。

## 电机安全

- 所有速度请求必须经过 `Motor_Safety_RequestSpeed()`。
- 启动保持 0→30% soft-start，禁止直接输出 100%。
- 200 ms 没有合法请求时看门狗锁存故障并发送零速帧。
- M1/M3 保持零速，M2/M4 是左右驱动轮。

首次接通电机动力前，必须完成架空轮、共地、UART、X1/X8 方向、D2 心跳和 OLED 诊断检查，并确保可以立即切断 12.6 V 电源。
