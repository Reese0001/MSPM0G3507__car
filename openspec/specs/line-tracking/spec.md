# Line Tracking

## Purpose

通过 PA15/PA16/PA17 通道选择和 PA18 OUT 读取八路灰度传感器，结合加权误差与 PID 算法实现循迹行驶。

## Requirements

### Requirement: 灰度传感器读取
通过三个选择 GPIO 和一个 OUT GPIO 轮询八路灰度数字状态。

#### Scenario: 读取八通道值
- **GIVEN** PA15/PA16/PA17 为通道选择输出，PA18 为 OUT 输入
- **WHEN** 调用 `Gray_ReadAll()`
- **THEN** 每个通道只采样一次，返回 8 个数字状态

#### Scenario: 黑线检测
- **GIVEN** 传感器在白色赛道上方且已确认模块极性
- **WHEN** 某通道 OUT 为低电平
- **THEN** 判断该通道检测到黑线

### Requirement: 循迹 PID 控制
基于八路灰度的加权偏差计算 PID 输出，控制小车循迹。

#### Scenario: 计算循迹偏差
- **GIVEN** 8 通道灰度值已读取
- **WHEN** 调用 `IRTrack_CalcError()`
- **THEN** 返回加权偏差值 (负=偏左, 正=偏右)

#### Scenario: PID 输出转向
- **GIVEN** 循迹偏差 error
- **WHEN** `IRTrack_PID(error)` 执行
- **THEN** 输出转向修正量 = KP*error + KI*∫error + KD*d(error)/dt

#### Scenario: 循迹行驶
- **GIVEN** 小车在赛道上
- **WHEN** 主循环调用循迹任务
- **THEN** 基础速度 + PID 转向修正 → `Motion_Car_Control(base_speed, pid_output)`

### Requirement: 循迹参数可调
PID 参数通过全局变量暴露，支持运行时调整。

#### Scenario: 修改 PID 参数
- **GIVEN** 程序运行中
- **WHEN** 修改 `IRTrack_Trun_KP/KI/KD` 变量
- **THEN** 下次循迹 PID 计算使用新参数
