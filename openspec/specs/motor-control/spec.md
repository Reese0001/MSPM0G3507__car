# Motor Control

## Purpose

通过 UART 协议帧控制四路电机驱动板 (AT8236)，实现两驱小车的差速运动控制，包含 Soft-start 安全层。

## Requirements

### Requirement: 电机驱动板通信协议
通过 UART1 (115200) 以 `$command:data#` 格式与电机驱动板通信。

#### Scenario: 发送速度控制命令
- **GIVEN** UART1 已初始化
- **WHEN** 调用 `Contrl_Speed(M1, M2, M3, M4)`
- **THEN** 发送 `$spd:M1,M2,M3,M4#`，速度范围 -1000~1000

#### Scenario: 设置电机类型
- **GIVEN** 系统初始化阶段
- **WHEN** 调用 `Set_Motor(MOTOR_TYPE=5)`
- **THEN** 发送 `$mtype:5#` (L型520)，并配置减速比40、磁环线数11、轮径以已购 65 mm 车轮为初始值、死区1900；轮径和编码器脉冲换算必须在实车标定后确认

#### Scenario: 接收编码器反馈
- **GIVEN** 电机驱动板数据上报已启用
- **WHEN** 驱动板发送数据帧
- **THEN** 解析 `$MSPD:s1,s2,s3,s4#` 获取各轮速度 (mm/s)

### Requirement: Motor Safety Layer
任何电机启动必须经过安全层，禁止业务代码直接调用 `Contrl_Speed()`。

#### Scenario: Soft-start 上电 ramp
- **GIVEN** 系统刚上电或电机刚使能
- **WHEN** `Motor_Safety_RequestSpeed()` 收到速度请求
- **THEN** 上电后 1000ms 内分 10 级从 0 提升到 30%，之后每 100ms 放宽 10%，直至目标速度

#### Scenario: 超时故障锁存
- **GIVEN** 电机正在运行
- **WHEN** 200ms 未收到新的速度请求
- **THEN** 锁存故障，定时器中断发送零速帧

#### Scenario: 故障恢复
- **GIVEN** 电机处于故障锁存状态
- **WHEN** 重新初始化电机模块
- **THEN** 故障解除，可重新接收速度请求

### Requirement: 两驱差速控制
M2(左后) 和 M4(右后) 为驱动轮，M1/M3 输出 0。

#### Scenario: 直行
- **GIVEN** 目标速度 V
- **WHEN** `Motion_Car_Control(V, 0)` (线速度V, 角速度0)
- **THEN** M2 = V, M4 = V, M1 = 0, M3 = 0

#### Scenario: 差速转向
- **GIVEN** 目标线速度 V, 角速度 ω
- **WHEN** `Motion_Car_Control(V, ω)`
- **THEN** M2 = V - ω*Car_APB/2, M4 = V + ω*Car_APB/2

### Requirement: 里程计
通过编码器反馈计算小车行驶距离和角度。

#### Scenario: 获取里程
- **GIVEN** 编码器数据已上报
- **WHEN** 调用 `Get_Odometry()`
- **THEN** 返回累计行驶距离和航向角变化
