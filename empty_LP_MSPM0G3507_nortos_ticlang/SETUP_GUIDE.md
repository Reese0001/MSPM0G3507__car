# 2026电赛小车平台 - 开发指南

## 已完成的工作

### 1. 项目结构搭建
- ✅ 复制BSP代码到项目目录
- ✅ 配置SysConfig外设 (UART0/UART1/I2C1/Timer/PWM/GPIO)
- ✅ 适配两驱电机控制 (MOTOR_TYPE=5, L型520电机)
- ✅ 适配MPU6050引脚 (PA13/PA12, 软件I2C)
- ✅ 适配八路灰度传感器 (I2C1, PA15/PA16)
- ✅ 编写主程序框架

### 2. 引脚分配
| 外设 | 引脚 | 说明 |
|------|------|------|
| UART0 (调试) | PA10(TX)/PA11(RX) | 115200, printf |
| UART1 (电机) | PB6(TX)/PB7(RX) | 115200, 电机驱动板 |
| I2C1 (灰度) | PA15(SCL)/PA16(SDA) | 八路灰度传感器 |
| MPU6050 | PA12(SCL)/PA13(SDA) | 软件I2C |
| LED D1 | PB2 | GPIO |
| LED D2 | PB3 | GPIO |
| 按键 K1 | PA2 | GPIO输入 |
| 蜂鸣器 | PB24 | PWM (TIMA0) |

### 3. 电机映射 (两驱)
- M1 (左前) → 万向轮 (空)
- M2 (左后) → L型520电机 (驱动轮)
- M3 (右前) → 万向轮 (空)
- M4 (右后) → L型520电机 (驱动轮)

---

## 用户需要完成的步骤

### 第一步: 配置CCS Theia开发环境

1. **打开CCS Theia**
2. **导入MSPM0 SDK**:
   - 菜单: Window → Preferences → Code Composer Studio → Products
   - 点击 "Add" 添加SDK路径
   - SDK路径应该类似于: `C:\ti\mspm0_sdk_2_02_00_05`
3. **导入项目**:
   - 菜单: File → Import → Existing Projects into Workspace
   - 选择 `E:\workspace_ccstheia\empty_LP_MSPM0G3507_nortos_ticlang` 目录
4. **测试编译**:
   - 右键项目 → Build Project
   - 确保编译成功，无错误

### 第二步: 连接MPU6050模块

1. 将MPU6050模块连接到扩展板的 **"IIC接口"**
2. 确认接线:
   - VCC → 3.3V
   - GND → GND
   - SCL → PA12
   - SDA → PA13

### 第三步: 烧录测试

1. 连接USB线到MSPM0G3507开发板
2. 右键项目 → Run As → Code Composer Studio Application
3. 打开串口助手 (波特率115200)
4. 观察输出:
   - 应该看到 "System Init..."
   - "MPU6050 Init..."
   - "DMP Init OK"
   - "Calibrating, keep still..."
   - (等待14秒校准)
   - "Calibration OK"
   - "System Ready!"
   - "Short press: Start | Long press: Select Question"

### 第四步: 功能测试

1. **按键测试**:
   - 短按K1: 蜂鸣器响500ms，启动小车
   - 长按K1: 切换题目 (1→2→3→4→1...)

2. **电机测试**:
   - 短按后，小车应该开始执行对应的题目任务
   - 观察电机是否转动
   - 如果电机不动，检查串口是否有电机通信数据

3. **传感器测试**:
   - MPU6050: 观察串口是否有角度数据
   - 灰度传感器: 观察巡线是否正常

---

## 常见问题

### 1. 编译错误: 找不到头文件
- 确保BSP目录在项目根目录下
- 检查 `.cproject` 文件中的包含路径

### 2. MPU6050初始化失败
- 检查接线是否正确
- 确认MPU6050模块供电正常 (3.3V)
- 检查I2C地址是否为0x68

### 3. 电机不转
- 检查电机驱动板电源 (12V)
- 确认串口通信正常 (PA10/PA11)
- 检查电机接线 (M2和M4)

### 4. 灰度传感器不工作
- 检查I2C接线 (PA15/PA16)
- 确认传感器地址为0x12
- 检查传感器供电

---

## 下一步开发

### 1. 调试电机参数
- 修改 `app_motor.c` 中的 `Set_Motor()` 函数
- 调整 `send_motor_deadzone()` 参数
- 测试不同速度下的电机响应

### 2. 调试PID参数
- 修改 `app_mpu6050.c` 中的 `Dir_PID()` 函数
- 调整 `dir_kp` 和 `dir_kd` 参数
- 测试航向保持效果

### 3. 调试巡线参数
- 修改 `app_irtracking.c` 中的PID参数
- 调整 `IRTrack_Trun_KP`, `IRTrack_Trun_KI`, `IRTrack_Trun_KD`
- 测试巡线效果

### 4. 添加新功能
- 在 `questions.c` 中添加新的题目任务
- 实现更复杂的路径规划
- 添加避障功能

---

## 文件说明

### 主要文件
- `empty.c` - 主程序，包含初始化和主循环
- `empty.syscfg` - SysConfig配置文件
- `BSP/` - 板级支持包
  - `Motor/` - 电机控制
  - `MPU6050/` - 陀螺仪驱动
  - `Eight_Tracking/` - 八路灰度传感器
  - `Task/` - 任务调度器
  - `Questions/` - 题目状态机

### 配置文件
- `.cproject` - CCS项目配置
- `.clangd` - Clang语言服务器配置
- `.project` - Eclipse项目配置

---

## 联系方式

如有问题，请检查:
1. 串口输出 (115200波特率)
2. 硬件接线
3. 电源供应
4. 代码配置

祝调试顺利！
