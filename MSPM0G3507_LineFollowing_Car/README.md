# MSPM0G3507_LineFollowing_Car CCS 工程

这是仓库当前维护的 CCS Theia 固件工程，可作为 Existing Project 导入。

## 入口与模块状态

- `empty.c` 是当前可执行入口，主循环调用 `Scheduler_Run()`、`LineWalking()` 和
  `Motor_Safety_Service()`。
- 当前实际行为仍是基础 `LineWalking()` 循迹，不是比赛路线状态机。
- `BSP/CarControl/car_route.*` 已提供故障闭锁的 `CarRoute` 状态机，但
  **CarRoute 尚未接入 empty.c**；待按键启动、反馈输入和硬件验收后再集成。
- `BSP/CarControl/car_motion.*` 已提供安全动作边界，但距离/角度单位尚未完成
  编码器与偏航标定，所以相关动作保持不可用并故障闭锁。
- `empty.syscfg` 是引脚与外设配置的唯一真实来源。

## 硬件映射

| 模块 | 映射 | 说明 |
|---|---|---|
| L 型 520 两驱 | PB6/PB7 电机 UART | M2 左轮、M4 右轮；M1/M3 保持零速 |
| 八路灰度 | PA15/PA16/PA17 选通，PA18 OUT | GPIO 多路选择，低电平黑线约定待实测 |
| MPU6050 | PA12/PA13 | 软件 I2C，不占用灰度引脚 |
| 调试 UART | PA10/PA11 | 115200 |

未购买摄像头，本阶段不包含视觉固件或视觉 SysConfig 修改。

## 构建要求

- CCS Theia
- TI Arm Clang 4.0.4 LTS
- MSPM0 SDK 2.10.00.04
- 兼容的 SysConfig

在 CCS 中先执行 **Project → Clean**，再执行 **Build Project**。当前工作树没有
生成的 `Debug/` 目录，所以本分支尚未完成 CCS 完整构建，不能声称输出文件已经
得到验证。部分纯模块已由测试调用 TI Arm Clang 做翻译单元编译，这不等价于
完整工程链接。

## 安全契约

- 所有速度请求都必须经过 `Motor_Safety_RequestSpeed()`。
- M2/M4 启动必须经过 0→30% 软启动，M1/M3 始终为零。
- 电机反馈年龄 **>= 200 ms** 即为过期，路线模块必须请求零速并锁存故障。
- `CarSensorFrame` 没有时间戳或序列号；只允许消费当前
  `CarSensor_ReadFrame` 调用刚生成的帧。
- 12.6 V 满充电池高于扩展/驱动板标称 **5-12 V** 上限。在确认板卡规格前，
  不得接入满充电池。

完整验证顺序见
[`../docs/setup/CAR_CONTROL_TEST_MATRIX.md`](../docs/setup/CAR_CONTROL_TEST_MATRIX.md)。
在完成离线检查、CCS 构建、断电接线检查和架空轮低电压验证以前，不进行低速
封闭赛道测试。
