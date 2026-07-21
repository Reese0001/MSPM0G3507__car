# Hardware Pinout

## Purpose

定义 MSPM0G3507 核心板与外设的引脚分配，确保 SysConfig 配置与实际接线一致。

## Requirements

### Requirement: UART0 调试串口
MCU 通过 UART0 输出调试信息 (printf 重定向)，波特率 115200。

#### Scenario: 串口打印初始化日志
- **GIVEN** 系统上电完成
- **WHEN** 主程序执行 `USART_Init()`
- **THEN** UART0 以 115200 波特率就绪，PA10(TX)/PA11(RX) 可收发

#### Scenario: printf 重定向
- **GIVEN** UART0 已初始化
- **WHEN** 代码调用 `printf()`
- **THEN** 输出通过 PA10(TX) 发送，串口助手可见

### Requirement: UART1 电机驱动通信
MCU 通过 UART1 与四路电机驱动板通信，波特率 115200。

#### Scenario: 发送速度命令
- **GIVEN** UART1 已初始化
- **WHEN** 调用 `Contrl_Speed(M1, M2, M3, M4)`
- **THEN** 通过 PB6(TX) 发送 `$spd:M1,M2,M3,M4#` 帧

#### Scenario: 接收编码器数据
- **GIVEN** 电机驱动板已启用数据上报
- **WHEN** 驱动板发送 `$MSPD:s1,s2,s3,s4#`
- **THEN** 通过 PB7(RX) 接收并解析速度值

### Requirement: GPIO 多路选择八路灰度传感器
当前八路灰度模块不是 I2C 地址设备；MCU 通过三个 GPIO 选择通道，再从一个 GPIO 读取数字输出。

#### Scenario: 读取灰度通道值
- **GIVEN** PA15/PA16/PA17 配置为通道选择输出，PA18 配置为灰度 OUT 输入
- **WHEN** 调用 `Gray_ReadAll()`
- **THEN** 依次输出 0~7 的选择码并读取 8 个黑线低电平数字值

#### Scenario: PA18 启动复用检查
- **GIVEN** PA18 同时具有 BSL 启动复用功能
- **WHEN** 小车上电或复位
- **THEN** 必须确认灰度模块不会把 PA18 固定在触发 BSL 的电平；未确认前不得把参考板引脚图直接当作接线许可

### Requirement: MPU6050 软件 I2C
MPU6050 通过软件 I2C 连接，地址 0x68。

#### Scenario: 读取姿态数据
- **GIVEN** MPU6050 已初始化 (PA12=SCL, PA13=SDA)
- **WHEN** DMP 解算完成
- **THEN** 返回 yaw/pitch/roll 角度值

### Requirement: GPIO 外设
LED (PB2/PB3)、按键 K1 (PA2)、蜂鸣器 PB24 (PWM TIMA0)。

#### Scenario: 按键检测
- **GIVEN** K1 连接 PA2，上拉输入
- **WHEN** 按键按下
- **THEN** PA2 读到低电平

#### Scenario: 蜂鸣器响
- **GIVEN** TIMA0 已配置 PWM
- **WHEN** 调用蜂鸣器响函数
- **THEN** PB24 输出 PWM 驱动蜂鸣器

### Requirement: 电机映射 (两驱)
M2(左后) 和 M4(右后) 为驱动轮，M1/M3 为万向轮(空)。

#### Scenario: 两驱差速转向
- **GIVEN** 小车直行中
- **WHEN** 需要左转
- **THEN** M4(右后) 增速，M2(左后) 减速，M1/M3 输出 0
