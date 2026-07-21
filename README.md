# MSPM0G3507 两驱循迹小车

本仓库是面向 2026 电赛控制类备赛的裸机小车工程。主控为 TI
MSPM0G3507，底盘为两驱 L 型 520 霍尔编码器电机，传感器包括八路灰度
模块和 MPU6050。开发环境为 CCS Theia，工具链为 TI Arm Clang，工程不使用
RTOS。

主工程位于
[`MSPM0G3507_LineFollowing_Car/`](MSPM0G3507_LineFollowing_Car/)。外设配置的
唯一真实来源是 `MSPM0G3507_LineFollowing_Car/empty.syscfg`，不要手改
`Debug/` 下的 SysConfig 生成文件。

## 怎样开始阅读这个项目

不要从 `BSP/` 中随便挑一个文件逐行读，也不要先钻进参考资料。理解这个工程最
快的方法是先回答四个问题：程序从哪里开始、传感器数据怎样产生、控制量怎样
变成左右轮命令、安全层怎样让电机停下来。

### 先建立一张运行地图

当前真正运行的固件链路如下：

```text
上电/复位
  -> main()                         empty.c
  -> SYSCFG_DL_init()               初始化 SysConfig 配置的外设
  -> Motor_Usart_init()             打开电机驱动板 UART 接收中断
  -> Timer_Init()                   启动 1 ms 时间基
  -> Motor_Safety_Init()/Arm()      电机安全层初始化并进入受控状态
  -> while (1)
       -> Scheduler_Run()           按周期处理按键、姿态和里程计
       -> LineWalking()             读取八路灰度并计算循迹修正量
       -> Motion_Car_Control()      将前进量/转向量换算成 M2、M4 命令
       -> Motor_Safety_RequestSpeed()
       -> Motor_Safety_Service()    限幅、软启动后发送 UART 速度帧
```

同时，定时器中断每 1 ms 调用 `Motor_Safety_Tick1ms()`。如果主循环长期没有提交
新的合法速度请求，看门狗会锁存故障并发送零速帧。因此，阅读运动代码时不能只
看“速度是怎样算出来的”，还必须继续追到 Motor Safety。

### 推荐阅读顺序

按下面顺序阅读，每一阶段只解决一个问题：

1. **先看项目边界，而不是算法**

   - 本文件：知道项目目标、已完成内容和未完成内容。
   - [`AGENTS.md`](AGENTS.md)：知道电源、电机和文件编码方面不能违反的规则。
   - [`docs/setup/CAR_PLATFORM_CONTRACT.md`](docs/setup/CAR_PLATFORM_CONTRACT.md)：
     对照实物理解底盘、电源、接口和左右轮定义。

2. **找到程序入口和硬件配置**

   - [`MSPM0G3507_LineFollowing_Car/empty.c`](MSPM0G3507_LineFollowing_Car/empty.c)：
     先只看初始化顺序和 `while (1)`，这是理解当前实际行为的起点。
   - [`MSPM0G3507_LineFollowing_Car/empty.syscfg`](MSPM0G3507_LineFollowing_Car/empty.syscfg)：
     查看 UART、定时器和 GPIO 的真实配置。它是外设配置的唯一真实来源。
   - `Debug/ti_msp_dl_config.*` 是构建时生成的结果，可以帮助排错，但不能作为
     修改入口。

3. **理解时间是怎样流动的**

   - `BSP/Timer/timer.c`：1 ms 计数、蜂鸣器服务和电机看门狗。
   - `BSP/Task/task.c`：裸机时间片调度器；`tasks[]` 决定各周期任务多久运行一次。
   - 阅读时记录每个任务的周期，并检查任务是否会长时间阻塞。这个工程没有
     RTOS，任何长延时都会拖慢其他任务和安全服务。

4. **沿着一次循迹计算向下追踪**

   从 `BSP/Eight_Tracking/app_irtracking.c` 的 `LineWalking()` 开始，依次查看：

   ```text
   Gray_ReadAll()
     -> Tracking_PackBlackSensors()
     -> Tracking_ComputeWeightedError()
     -> APP_ELE_PID_Calc()
     -> Tracking_LimitTurn()
     -> Motion_Car_Control()
   ```

   阅读时重点记下：X1～X8 与 bit0～bit7 的对应关系、黑线的有效电平、误差正负
   代表哪一侧、`TRACKING_STEERING_POLARITY` 为什么需要取反、丢线多少周期后停车。
   不要通过交换 M2/M4 来掩盖误差符号问题。

5. **继续追到电机驱动板，而不是停在 PID 输出**

   - `BSP/Motor/app_motor.c`：`Motion_Car_Control()` 把车体前进/转向命令换算为
     四路格式；当前只有 M2 左轮和 M4 右轮参与驱动。
   - `BSP/Motor/motor_safety.c`：命令限幅、软启动、200 ms 看门狗和故障锁存。
   - `BSP/Motor/app_motor_usart.c`：把四路速度编码成驱动板协议帧，并解析反馈。
   - `BSP/Motor/bsp_motor_usart.c`：UART 字节收发、中断入口和紧急零速帧。

   到这里应当能够用一句话解释完整因果链：灰度位置改变了加权误差，误差改变
   左右轮差速，差速请求经过安全层后才会发给电机驱动板。

6. **最后再看尚未接入主程序的新控制层**

   - `BSP/CarControl/car_sensor_events.*`：把八路灰度帧分类为居中、边缘、十字、
     停车标记或丢线。
   - `BSP/CarControl/car_motion.*`：统一运动接口和电机反馈新鲜度检查。编码器与
     轮径尚未标定，所以距离/角度动作会主动拒绝执行。
   - `BSP/CarControl/car_route.*`：非阻塞路线状态机。当前代码已实现，但还没有
     从 `empty.c` 调用。
   - `BSP/Questions/questions.c`：历史比赛题目的顶层状态机，只用于理解旧设计，
     不等于当前运行入口。

### 目录怎样分工

| 目录或文件 | 阅读用途 | 是否是当前运行代码 |
|---|---|---|
| `MSPM0G3507_LineFollowing_Car/` | CCS 固件工程和真正的 MCU 源码 | 是 |
| `MSPM0G3507_LineFollowing_Car/BSP/` | 传感器、电机、时间基和控制模块 | 是，具体以 `empty.c` 的调用链为准 |
| `tests/` | 用 Python 检查接口、安全合同、方向和决策表 | 主机侧验证，不烧入 MCU |
| `docs/setup/` | 接线、平台合同、构建和分阶段验收说明 | 文档 |
| `docs/reference/` | 历史项目和外部资料 | 仅参考，不代表本车实现 |
| `my-project/` | 面向需求、阶段和验收的项目规划 | 规划资料 |
| `openspec/` | 结构化变更规格和设计约束 | 规格资料 |
| `Debug/` | CCS/SysConfig 生成物和构建产物 | 生成目录，不直接编辑、不提交 |

判断“某段代码现在是否会运行”的可靠方法不是看文件名，而是从 `empty.c` 沿函数
调用向下追踪；没有连到这条调用链的模块，即使已经写完，也不会自动执行。

### 阅读时建议做的三张表

为了避免只看懂局部函数，建议边读边手写或维护以下三张小表：

1. **引脚表**：外设、MCU 引脚、方向、电压、连接对象，对照 `empty.syscfg` 和实物。
2. **数据流表**：变量的单位、正方向、生产者、消费者、更新时间，例如灰度误差、
   M2/M4 速度、编码器计数和 MPU6050 偏航角。
3. **状态与故障表**：启动条件、正常转移、停止条件、超时值，以及最终是否确实
   发送了零速命令。

第一次阅读的完成标准不是“看完所有文件”，而是能够不看代码画出传感器到电机
的链路，并回答以下问题：

- 哪个文件决定引脚？
- 黑线位于左侧时，误差符号和左右轮速度应怎样变化？
- M2、M4 分别是哪一侧，M1、M3 为什么必须为零？
- 主循环卡住超过 200 ms 后，哪里负责停车？
- 为什么当前不能按“毫米”和“角度”控制小车？
- 为什么 `CarRoute` 已存在却不会在当前固件中运行？

### 从阅读过渡到动手验证

先运行主机侧测试，测试名本身也是一份可执行的设计说明：

```powershell
python -m unittest discover -s tests -v
```

然后在 CCS 中只编译，不接电机电源；再观察调试串口和静态灰度读数；最后按照
[`docs/setup/CAR_CONTROL_TEST_MATRIX.md`](docs/setup/CAR_CONTROL_TEST_MATRIX.md)
依次完成断电检查、架空轮低速测试和封闭赛道测试。不要为了“读懂代码”直接
跳到带载运行。

### 文件编码提示

Markdown 文档使用 UTF-8。部分历史 C/H 文件保留 GBK 中文注释；如果 CCS 中
只看到注释乱码，应先按原编码重新打开文件，不要批量另存或转换整个源码树。
修改源码前先确认该文件当前编码，避免把正常代码再次损坏。

## 当前可执行状态

当前 `empty.c` 仍运行已有基础循迹主循环：

```text
Scheduler_Run() -> LineWalking() -> Motor_Safety_Service()
```

`BSP/CarControl/car_route.*` 已实现非阻塞、故障闭锁的路线状态机，但 **CarRoute
尚未接入 empty.c**。在电机反馈、按键启动流程和硬件验收完成之前，不把它
静默接入现有入口。距离和角度动作也尚未标定，不能宣称已经能够按毫米或角度
闭环行驶。

## 已确认的硬件边界

| 功能 | 接口或映射 | 当前约定 |
|---|---|---|
| 两驱电机 | UART：PB6/PB7，115200 | L520；M2 左轮、M4 右轮，M1/M3 始终为零 |
| 八路灰度选择 | PA15/PA16/PA17 | GPIO 多路选择 AD0/AD1/AD2，不是 I2C 数据读取 |
| 八路灰度输出 | PA18 | OUT 数字输入；低电平暂按黑线解释，实车仍需确认 |
| MPU6050 | PA12/PA13 | 软件 I2C，3.3 V |
| 调试串口 | PA10/PA11 | 115200 |

电池满充电压为 **12.6 V**，而所购扩展/驱动板图片标注输入 **5-12 V**。
这是上电硬门槛：未取得准确板卡规格并确认可承受 12.6 V 前，不得把满充电池
接入该输入端。首次软件验证阶段必须断开电机电源。

摄像头尚未购买（camera has not been purchased），当前不增加视觉接口，也不
修改 SysConfig 预占视觉引脚。

## 安全与数据新鲜度

- 所有运动命令必须经过 Motor Safety；启动使用 0→30% 的 1000 ms 软启动。
- 200 ms 内没有新的合法运动请求时，安全看门狗锁存故障并发送固定零速帧。
- 电机反馈的年龄 **>= 200 ms** 时，`CarMotion`/`CarRoute` 必须故障停车。
- `CarSensorFrame` 当前没有时间戳或序列号；调用方只能使用本周期成功执行
  `CarSensor_ReadFrame` 后立即得到的帧，不能缓存旧帧冒充新数据。

平台、电源和首次架空检查的完整约定见
[`docs/setup/CAR_PLATFORM_CONTRACT.md`](docs/setup/CAR_PLATFORM_CONTRACT.md)。

## 导入与验证

1. 在 CCS Theia 选择 **File → Import → Existing Projects into Workspace**。
2. 选择仓库中的 `MSPM0G3507_LineFollowing_Car` 目录。
3. 确认 MSPM0 SDK 2.10.00.04 与 TI Arm Clang 4.0.4 可用。
4. 执行 **Project → Clean** 和 **Build Project**。

当前离线测试命令：

```powershell
python -m unittest discover -s tests -v
```

截至 2026-07-22，Python 契约测试 **51/51 通过**，CCS 全工程已经完成
SysConfig 生成、TI Arm Clang 编译和链接，输出文件为
`MSPM0G3507_LineFollowing_Car/Debug/MSPM0G3507_LineFollowing_Car.out`。这只证明
当前源码能够构建；烧录、目标板通信、传感器极性、电机方向和带载运行仍需按
实车验证矩阵确认，不能把构建成功解释为可以直接给电机上电。

分阶段验证顺序与通过标准见
[`docs/setup/CAR_CONTROL_TEST_MATRIX.md`](docs/setup/CAR_CONTROL_TEST_MATRIX.md)，
详细设置见 [`docs/setup/SETUP_GUIDE.md`](docs/setup/SETUP_GUIDE.md)。
