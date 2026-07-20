# Line Tracking

## Purpose

通过八路灰度传感器 (I2C, 地址 0x12) 检测赛道黑线位置，结合 PID 算法实现循迹行驶。

## Requirements

### Requirement: 灰度传感器读取
通过 I2C1 (PA15=SCL, PA16=SDA) 读取八路灰度传感器数据。

#### Scenario: 读取八通道值
- **GIVEN** I2C1 已初始化，传感器地址 0x12
- **WHEN** 调用 `IRTrack_Read()`
- **THEN** 返回 8 个通道的灰度值 (0~4095)

#### Scenario: 黑线检测
- **GIVEN** 传感器在白色赛道上方
- **WHEN** 某通道值低于阈值
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
