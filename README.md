# MSPM0G3507 两驱循迹小车

本仓库是面向 2026 电赛控制类备赛的裸机小车工程。主控为 TI
MSPM0G3507，底盘为两驱 L 型 520 霍尔编码器电机，传感器包括八路灰度
模块和 MPU6050。开发环境为 CCS Theia，工具链为 TI Arm Clang，工程不使用
RTOS。

主工程位于
[`MSPM0G3507_LineFollowing_Car/`](MSPM0G3507_LineFollowing_Car/)。外设配置的
唯一真实来源是 `MSPM0G3507_LineFollowing_Car/empty.syscfg`，不要手改
`Debug/` 下的 SysConfig 生成文件。

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

Python 契约测试和部分 TI Arm Clang 翻译单元编译已经可离线运行；当前工作树
没有 CCS 生成的 `Debug/` 构建目录，因此 **CCS 完整构建、烧录和目标板运行均
未验证**。不得把离线通过解释为可以直接给电机上电。

分阶段验证顺序与通过标准见
[`docs/setup/CAR_CONTROL_TEST_MATRIX.md`](docs/setup/CAR_CONTROL_TEST_MATRIX.md)，
详细设置见 [`docs/setup/SETUP_GUIDE.md`](docs/setup/SETUP_GUIDE.md)。
