# 2026电赛小车通用平台开发计划

## 硬件配置总结

| 硬件 | 型号/规格 | 状态 |
|------|----------|------|
| 主控 | MSPM0G3507 (LQFP48, 80MHz, Cortex M0+) | 已接 |
| 扩展板 | MSPM0机器人扩展板 | 已接 |
| 电机驱动 | 四路编码器电机驱动模块 (AT8236芯片) | 已接(串口) |
| 电机 | L型520编码器电机 (12V, 1:40减速比, 11线磁环, 300rpm) | 已装 |
| 底盘 | 两驱底盘 228×148mm (后2轮驱动 + 前2万向轮) | 已装 |
| 巡线传感器 | 八路灰度传感器 (I2C, 地址0x12) | 已接 |
| MPU6050 | 待接入 | 未接 |

## 引脚分配（已确认）

| 外设 | 引脚 | 说明 |
|------|------|------|
| UART0 (调试串口) | PA10(TX) / PA11(RX) | 115200, printf重定向 |
| UART1 (电机驱动) | PB6(TX) / PB7(RX) | 115200, 与电机驱动板通信 |
| I2C1 (灰度传感器) | PA16(SDA) / PA15(SCL) | 硬件I2C, 地址0x12 |
| MPU6050 I2C | PA13(SDA) / PA12(SCL) | 软件I2C (扩展板IIC接口) |
| LED D1 | PB2 | GPIO |
| LED D2 | PB3 | GPIO |
| 按键 K1 | PA2 | GPIO输入 |
| 蜂鸣器 | PB24 | PWM (TIMA0) |

## 电机映射（两驱底盘）

```
M1 (左前) → 万向轮（空，不接电机）
M2 (左后) → L型520电机（驱动轮）
M3 (右前) → 万向轮（空，不接电机）
M4 (右后) → L型520电机（驱动轮）
```

## 电机驱动板通信协议（串口）

- 波特率: 115200
- 命令格式: `$command:data#`
- 发送命令:
  - `$spd:M1,M2,M3,M4#` — 速度控制 (范围-1000~1000)
  - `$pwm:M1,M2,M3,M4#` — PWM直接控制
  - `$mtype:N#` — 电机类型 (1:520, 5:L型520)
  - `$phase:N#` — 减速比 (L型520=40)
  - `$pline:N#` — 磁环线数 (L型520=11)
  - `$diameter:XX#` — 轮径mm (L型520=67.00)
  - `$deadzone:N#` — 死区 (L型520=1900)
  - `$upload:0/1,0/1,0/1#` — 数据上报控制
- 接收数据:
  - `$MTEP:e1,e2,e3,e4#` — 10ms编码器增量
  - `$MAll:e1,e2,e3,e4#` — 累计编码器值
  - `$MSPD:s1,s2,s3,s4#` — 当前速度mm/s

---

## 开发任务分解

### 第1步: 确认CCS Theia环境 [需用户]
- 配置MSPM0 SDK (mspm0_sdk@2.02.00.05)
- 测试编译空项目能否成功
- **用户需操作**: 打开CCS Theia，确认SDK已配置，能编译 `empty_LP_MSPM0G3507_nortos_ticlang` 项目

### 第2步: 复制参考代码到新项目
- 从 `docs/8.程序源码汇总/2024年国赛赛题H题/四驱/CCS/Competition_PJ/` 复制BSP目录
- 复制的模块: delay, usart, Motor, MPU6050, eMPL, Eight_Tracking, LED, Key, Buzzer, Timer, Task, Questions
- **Agent执行**

### 第3步: 配置SysConfig
- 基于参考代码的 `.syscfg` 配置:
  - UART0: PA10/PA11, 115200 (调试)
  - UART1: PB6/PB7, 115200 (电机驱动)
  - I2C1: PA15/PA16 (灰度传感器)
  - TIMER_0: 1ms周期
  - PWM BUZZER: PB24 (TIMA0)
  - GPIO: LED(PB2/PB3), KEY(PA2), MPU6050(PA13/PA12 软件I2C)
- **Agent执行**

### 第4步: 适配两驱配置
- 修改 `app_motor.c` 中的电机控制:
  - `Motion_Car_Control()`: 简化为两驱控制
  - 万向轮对应通道(M1/M3)输出0
  - 驱动轮通道(M2/M4)输出差速控制
- 修改 `Set_Motor()` 中 `MOTOR_TYPE=5` (L型520)
- 修改 `bsp_mpu6050.c` 中的I2C引脚为PA13(SDA)/PA12(SCL)
- **Agent执行**

### 第5步: 适配灰度传感器
- 修改 `app_irtracking.c` 中的I2C引脚配置
- 确认八路灰度传感器使用硬件I2C1 (PA15/PA16)
- **Agent执行**

### 第6步: 主程序框架
- 修改 `empty.c`:
  - 初始化: USART_Init, MPU6050_Init, Set_Motor(MOTOR_TYPE=5)
  - 任务调度: Scheduler_Run()
  - 状态机: 主循环调用 Question_Task_N()
- **Agent执行**

### 第7步: 编译测试
- 编译项目，确保无错误
- **Agent执行**

---

## 用户需确认/操作的事项

1. **CCS Theia配置 [必须先完成]**:
   - 打开CCS Theia
   - 导入MSPM0 SDK (路径: mspm0_sdk@2.02.00.05)
   - 导入 `empty_LP_MSPM0G3507_nortos_ticlang` 项目
   - 测试编译是否成功
   - 如遇到问题请告知具体错误信息

2. **MPU6050模块**: 需要你将MPU6050模块连接到扩展板的"IIC接口"（PA13/PA12引脚）

3. **编译器**: 项目使用TI Clang编译器（.clangd配置确认）

## 安全注意事项
- 首次测试电机先断开电机电源线（只测试通信）
- MPU6050校准需保持小车静止14秒
- L型520电机堵转电流4A，确保电源充足
- 使用PWM控制时从小值开始
