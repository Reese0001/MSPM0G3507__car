# IMU Attitude (MPU6050)

## Purpose

通过 MPU6050 六轴传感器 + DMP 硬件解算获取小车姿态 (yaw/pitch/roll)，用于航向保持和转弯控制。

## Requirements

### Requirement: MPU6050 初始化与 DMP 解算
通过软件 I2C (PA12=SCL, PA13=SDA) 初始化 MPU6050 并加载 DMP 固件。

#### Scenario: 正常初始化
- **GIVEN** MPU6050 模块已连接到扩展板 IIC 接口
- **WHEN** 调用 `MPU6050_Init()`
- **THEN** I2C 通信建立，DMP 固件加载成功，返回 OK

#### Scenario: 静止校准
- **GIVEN** DMP 初始化成功
- **WHEN** 进入校准流程 (约14秒)
- **GIVEN** 小车保持静止
- **THEN** 计算陀螺仪零偏，校准完成后输出 "Calibration OK"

#### Scenario: 初始化失败处理
- **GIVEN** MPU6050 未接线或损坏
- **WHEN** I2C 读取无应答
- **THEN** 返回错误，串口输出失败信息

### Requirement: 姿态数据获取
DMP 解算后实时获取 yaw/pitch/roll 角度。

#### Scenario: 读取航向角
- **GIVEN** DMP 已初始化且校准完成
- **WHEN** 调用 `MPU6050_GetAngle()`
- **THEN** 返回当前 yaw 角度 (0~360° 或 -180°~+180°)

#### Scenario: 数据更新频率
- **GIVEN** DMP 配置为 100Hz 输出
- **WHEN** 主循环以 10ms 周期读取
- **THEN** 每次读取获得最新姿态数据

### Requirement: 航向 PID 控制
基于 yaw 角偏差计算 PID 输出，用于直线行驶时航向保持。

#### Scenario: 航向保持
- **GIVEN** 小车目标航向 target_yaw, 当前航向 current_yaw
- **WHEN** `Dir_PID(target_yaw, current_yaw)` 执行
- **THEN** 输出转向修正量 = KP * yaw_error + KD * d(yaw_error)/dt

#### Scenario: PID 参数可调
- **GIVEN** 程序运行中
- **WHEN** 修改 `dir_kp` / `dir_kd` 变量
- **THEN** 下次航向 PID 使用新参数
