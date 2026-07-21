# 小车控制备赛平台升级实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有 MSPM0G3507 两驱 L 型 520 小车工程上，建立可覆盖循迹、定距直行、定角转向、识别/避障和比赛路线状态机的安全控制平台。

**Architecture:** 保留现有 `Eight_Tracking`、`Motor`、`MPU6050` 和 `motor_safety` 底层驱动，在其上新增统一的 `CarControl` 运动与路线层。主循环只负责调度传感器、路线状态机和安全服务；所有电机请求继续经过 `Motor_Safety_RequestSpeed()`。摄像头和避障模块先用抽象输入接口接入，只有硬件型号和 SysConfig 引脚确认后才启用具体驱动。

**Tech Stack:** MSPM0G3507、TI DriverLib、TI Arm Clang 4.0.4、MSPM0 SDK 2.10.00.04、CCS Theia、C 裸机、Python `unittest`。

## Global Constraints

- `empty.syscfg` 是引脚与外设配置的唯一真实来源；未确认摄像头/测距模块型号和引脚前，不新增 UART、I2C 或 GPIO 配置。
- 当前底盘保持 M2=左驱动轮、M4=右驱动轮、M1/M3=0；不改为履带或麦克纳姆轮。
- 所有电机请求必须经过 `Motor_Safety_RequestSpeed()`，保留 0→30% soft-start 和 200 ms watchdog。
- 所有运动任务必须非阻塞；禁止在 5 ms/10 ms 调度路径中使用 `delay_ms(20)`、`delay_ms(100)` 或更长阻塞延时。
- 实车验证前必须确认整车外廓 ≤35 cm×25 cm、编码器反馈格式、左右轮方向、灰度极性和共地；首次测试架空驱动轮并可立即断开 12.6 V 电源。
- 不修改或提交未跟踪的 `my-project/`。

## 当前代码审计结论

- `empty.c` 当前没有调用 `Scheduler_Run()`，因此 `task.c` 注册的 `Get_EulerAngles()`、`Get_CalibratedAngles()` 和 `Get_Odometry()` 实际不会执行。
- `Get_EulerAngles()` 内含 `delay_ms(20)`，与任务表中的 5 ms 周期冲突，必须改成单次非阻塞采样。
- `Get_Odometry()` 只累加原始 `Encoder_Offset`，没有统一的 mm、角度和时间接口，路线控制不能可靠使用距离阈值。
- `questions.c` 中的旧题目状态机未被当前 `main()` 调用，且把速度、角度、里程阈值写死在旧全局变量中；不直接扩展旧状态机，新增独立路线层。
- 当前 `LineWalking()` 已有加权误差、转向极性和丢线停车，但没有向上层输出“黑线/横线/十字/丢线”等事件。
- 当前工程没有摄像头、避障和无线任务输入接口；这些只能先做抽象接口，不能猜测硬件协议。

---

### Task 1: 建立硬件与软件验收基线

**Files:**
- Create: `docs/setup/CAR_PLATFORM_CONTRACT.md`
- Modify: `README.md`
- Test: `tests/test_car_platform_contract.py`

**Interfaces:**
- Consumes: 当前 `AGENTS.md`、`docs/setup/SETUP_GUIDE.md`、`empty.syscfg`、电机 UART 协议
- Produces: `CarPlatformContract` 文档中的确定参数：车体尺寸、M2/M4 映射、编码器帧格式、灰度电平和候选外设接口

- [ ] **Step 1: 写入基线测试**

在 `tests/test_car_platform_contract.py` 中检查以下字符串和数值存在：`MOTOR_TYPE=5`、`PB6`、`PB7`、`PA15`、`PA16`、`PA17`、`PA18`、`M2`、`M4`、`MOTOR_SAFETY_WATCHDOG_MS`；同时检查主工程路径为 `MSPM0G3507_LineFollowing_Car`。

- [ ] **Step 2: 运行基线测试**

Run: `python -m unittest tests.test_car_platform_contract -v`

Expected: 新测试在文档/API尚未完成时失败，失败项明确指出缺失的硬件契约。

- [ ] **Step 3: 编写平台契约**

文档必须逐项记录：整车最大尺寸 35 cm×25 cm、两驱 M2/M4、灰度 PA15～PA18、UART0 调试 115200、UART1 电机 115200、黑线低电平约定、编码器帧 `MTEP`/`MAll`/`MSPD` 的含义，以及“摄像头/测距模块型号确认前不改 SysConfig”。

- [ ] **Step 4: 复测并提交**

Run: `python -m unittest tests.test_car_platform_contract -v`

Expected: PASS；提交：`git add docs/setup/CAR_PLATFORM_CONTRACT.md README.md tests/test_car_platform_contract.py && git commit -m "docs: define car platform contract"`。

### Task 2: 修复周期调度和传感器采样所有权

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/empty.c`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Task/task.c`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Task/task.h`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/MPU6050/app_mpu6050.c`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/MPU6050/app_mpu6050.h`
- Test: `tests/test_scheduler_contract.py`

**Interfaces:**
- Produces: `Scheduler_Run()` 被主循环调用；`Get_EulerAngles()` 只做一次采样，不再延时；`Get_Odometry()` 由周期任务统一调用

- [ ] **Step 1: 写失败测试**

测试必须检查 `empty.c` 含有 `Scheduler_Run();`，且位于 `LineWalking();` 与 `Motor_Safety_Service();` 的同一循环中；检查 `Get_EulerAngles()` 函数体不含 `delay_ms`；检查任务表仍包含 5 ms、15 ms 和 30 ms 周期。

- [ ] **Step 2: 运行失败测试**

Run: `python -m unittest tests.test_scheduler_contract -v`

Expected: 当前版本因没有 `Scheduler_Run()` 且存在 `delay_ms(20)` 而失败。

- [ ] **Step 3: 修改主循环和采样函数**

主循环固定为以下顺序：

```c
while (1) {
    Scheduler_Run();
    LineWalking();
    Motor_Safety_Service();
    delay_ms(10);
}
```

`Get_EulerAngles()` 删除内部延时，保留一次 `mpu_dmp_get_data()`；采样节拍由任务表管理。`Get_Odometry()` 不再在多个业务状态中自行开关底层解析。

- [ ] **Step 4: 复测、编译并提交**

Run: `python -m unittest tests.test_scheduler_contract.py -v`

Run from `MSPM0G3507_LineFollowing_Car/Debug`: `D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe -j4 all`

Expected: 测试 PASS，CCS exit code 0；提交：`git commit -m "fix: run sensor scheduler in car loop"`。

### Task 3: 统一编码器和底盘运动接口

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/BSP/CarControl/car_motion.h`
- Create: `MSPM0G3507_LineFollowing_Car/BSP/CarControl/car_motion.c`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Motor/app_motor_usart.h`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Motor/app_motor_usart.c`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Motor/app_motor.h`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Motor/app_motor.c`
- Test: `tests/test_car_motion_contract.py`

**Interfaces:**
- Produces these exact APIs:

```c
typedef struct {
    int32_t left_ticks;
    int32_t right_ticks;
    float left_speed_mm_s;
    float right_speed_mm_s;
    uint32_t timestamp_ms;
    bool units_valid;
} CarMotionFeedback;

void CarMotion_Reset(void);
bool CarMotion_ReadFeedback(CarMotionFeedback *feedback);
void CarMotion_Command(int16_t linear_speed, int16_t angular_command);
void CarMotion_Stop(void);
bool CarMotion_DriveDistanceStart(int32_t distance_mm, int16_t speed);
bool CarMotion_DriveDistanceStep(void);
bool CarMotion_TurnAngleStart(float angle_deg, int16_t speed);
bool CarMotion_TurnAngleStep(void);
```

- [ ] **Step 1: 写接口契约测试**

检查新头文件存在上述类型和函数；检查 `CarMotion_Command()` 的实现只调用 `Motion_Car_Control()`，而 `Motion_Car_Control()` 继续只调用 `Motor_Safety_RequestSpeed()`。

- [ ] **Step 2: 实现反馈快照**

在 `app_motor_usart.c` 中保留 `Encoder_Offset[]`、`g_Speed[]` 的协议解析，但新增带时间戳的快照读取；只使用 M2 和 M4 计算左右轮反馈，并明确正负方向。未确认编码器单位前，`CarMotionFeedback` 必须标记 `units_valid=false` 或由 `CarMotion_ReadFeedback()` 返回 `false`，禁止把原始计数伪装成毫米。

同时从 `app_motor.h` 移除对 `questions.h` 的反向包含，保留 `app_motor.h` 只依赖 DriverLib 和电机协议头，避免新 `CarControl` 层与旧题目状态机形成循环包含。

- [ ] **Step 3: 实现非阻塞定距/定角状态**

`CarMotion_DriveDistanceStart()` 保存起始左右轮计数和目标距离；`CarMotion_DriveDistanceStep()` 每次调用只读取一次反馈、计算剩余误差并发出有界速度；误差进入校准阈值后调用 `CarMotion_Stop()` 并返回 `true`。转角动作使用左右轮差分或已确认的 yaw 数据，禁止使用固定 `delay_ms()` 完成转弯。

- [ ] **Step 4: 复测、构建并提交**

Run: `python -m unittest tests.test_car_motion_contract -v`

Run from Debug: `D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe -j4 all`

Expected: 测试 PASS，构建 PASS；提交：`git commit -m "feat: add nonblocking car motion primitives"`。

### Task 4: 增加循迹事件和丢线恢复接口

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.h`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.c`
- Create: `MSPM0G3507_LineFollowing_Car/BSP/CarControl/car_sensor_events.h`
- Create: `MSPM0G3507_LineFollowing_Car/BSP/CarControl/car_sensor_events.c`
- Test: `tests/test_car_sensor_events.py`

**Interfaces:**
- Produces:

```c
typedef enum {
    CAR_LINE_NONE = 0,
    CAR_LINE_CENTER,
    CAR_LINE_LEFT_EDGE,
    CAR_LINE_RIGHT_EDGE,
    CAR_LINE_CROSS,
    CAR_LINE_STOP_MARKER
} CarLineEvent;

typedef struct {
    uint8_t sensor_bits;
    float weighted_error;
    bool line_valid;
    CarLineEvent event;
} CarSensorFrame;

bool CarSensor_ReadFrame(CarSensorFrame *frame);
CarLineEvent CarSensor_Classify(const CarSensorFrame *frame);
```

- [ ] **Step 1: 写模式测试**

测试至少覆盖：中心黑线、左边缘黑线、右边缘黑线、全白丢线、两端同时黑线/十字、连续黑线停止标志；验证 X1 权重为负、X8 权重为正。

- [ ] **Step 2: 实现单次采样缓存**

`CarSensor_ReadFrame()` 只调用一次 `Gray_ReadAll()`，将黑线低电平转换为 `sensor_bits`，调用现有 `Tracking_ComputeWeightedError()`，不得让路线层重复读取 8 路传感器。

- [ ] **Step 3: 保持现有安全循迹行为**

`LineWalking()` 继续负责正常 PD 和丢线三周期停车；事件层只提供分类结果，不直接发送电机命令。所有事件识别必须无阻塞。

- [ ] **Step 4: 复测并提交**

Run: `python -m unittest tests.test_car_sensor_events -v`

Expected: PASS；提交：`git commit -m "feat: expose line tracking sensor events"`。

### Task 5: 新增比赛路线状态机

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/BSP/CarControl/car_route.h`
- Create: `MSPM0G3507_LineFollowing_Car/BSP/CarControl/car_route.c`
- Modify: `MSPM0G3507_LineFollowing_Car/empty.c`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/LED/led.c`
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/Buzzer/buzzer.c`
- Test: `tests/test_car_route_contract.py`

**Interfaces:**
- Produces:

```c
typedef enum {
    CAR_ROUTE_IDLE = 0,
    CAR_ROUTE_LINE_FOLLOW,
    CAR_ROUTE_DRIVE_DISTANCE,
    CAR_ROUTE_TURN_ANGLE,
    CAR_ROUTE_SEARCH_LINE,
    CAR_ROUTE_TARGET_APPROACH,
    CAR_ROUTE_STOPPED,
    CAR_ROUTE_FAULT
} CarRouteState;

void CarRoute_Init(void);
void CarRoute_Start(void);
void CarRoute_Stop(void);
void CarRoute_Run(const CarSensorFrame *sensor, const CarMotionFeedback *motion);
CarRouteState CarRoute_GetState(void);
```

- [ ] **Step 1: 写状态迁移测试**

测试 `IDLE→LINE_FOLLOW`、`LINE_FOLLOW→DRIVE_DISTANCE`、`DRIVE_DISTANCE→TURN_ANGLE`、`TURN_ANGLE→SEARCH_LINE`、`SEARCH_LINE→STOPPED`，以及 watchdog 故障或传感器超时进入 `FAULT` 后必须调用 `CarMotion_Stop()`。

- [ ] **Step 2: 实现最小路线**

先实现固定演示路线：启动→循迹→检测停止标志→定距直行→定角转向→搜索黑线→停止。每个状态只在 `CarRoute_Run()` 中执行一次非阻塞 step，不直接访问 `Contrl_Speed()`。

- [ ] **Step 3: 接入按键、LED和蜂鸣器**

K1 启动/停止路线；LED 显示 `IDLE`、运行和故障；到达关键节点由蜂鸣器短鸣。启动必须先确认 `Motor_Safety_Arm()`，故障状态不能自动重启电机。

- [ ] **Step 4: 替换主循环调用**

主循环按 `Scheduler_Run()`→`CarSensor_ReadFrame()`→`CarMotion_ReadFeedback()`→`CarRoute_Run()`→`Motor_Safety_Service()` 顺序运行；`LineWalking()` 只作为 `CAR_ROUTE_LINE_FOLLOW` 状态内部动作，不再和路线层并行调用。

- [ ] **Step 5: 复测、构建并提交**

Run: `python -m unittest tests.test_car_route_contract.py -v`

Run from Debug: `D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe -j4 all`

Expected: PASS；提交：`git commit -m "feat: add nonblocking car route state machine"`。

### Task 6: 预留摄像头、避障和无线输入适配层

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/BSP/CarControl/car_target_input.h`
- Create: `MSPM0G3507_LineFollowing_Car/BSP/CarControl/car_target_input.c`
- Modify only after hardware confirmation: `MSPM0G3507_LineFollowing_Car/empty.syscfg`
- Modify: `MSPM0G3507_LineFollowing_Car/README.md`
- Test: `tests/test_car_target_input_contract.py`

**Interfaces:**
- Produces:

```c
typedef struct {
    bool valid;
    uint8_t target_id;
    int16_t offset_x;
    int16_t offset_y;
    uint16_t distance_mm;
    uint32_t timestamp_ms;
} CarTargetObservation;

bool CarTargetInput_Init(void);
bool CarTargetInput_Read(CarTargetObservation *observation);
```

- [ ] **Step 1: 先写无硬件适配测试**

测试检查目标数据结构、时间戳有效性、无数据时返回 `false`，并确认该模块不直接调用电机 API。

- [ ] **Step 2: 实现空适配层**

默认 `CarTargetInput_Read()` 返回 `false`，不占用任何新引脚；路线层把 `valid=false` 视为“暂无目标”，不能误判为目标丢失或立刻启动电机动作。

- [ ] **Step 3: 确认硬件后选择具体驱动**

只有拿到摄像头/测距模块型号、供电电压、UART/I2C 协议和所需引脚后，才修改 `empty.syscfg` 并生成配置文件；不允许把电脑 USB 摄像头协议直接写入 MSPM0 固件。

- [ ] **Step 4: 复测并提交**

Run: `python -m unittest tests.test_car_target_input_contract.py -v`

Expected: 空适配层 PASS；提交：`git commit -m "feat: reserve target input adapter"`。具体硬件驱动单独提交。

### Task 7: 集成验证、文档和烧录包

**Files:**
- Modify: `README.md`
- Modify: `MSPM0G3507_LineFollowing_Car/README.md`
- Modify: `docs/setup/SETUP_GUIDE.md`
- Create: `docs/setup/CAR_CONTROL_TEST_MATRIX.md`
- Test: `tests/test_line_following_contract.py`
- Test: `tests/test_motor_safety_contract.py`

**Interfaces:**
- Consumes: Tasks 1–6 的公共 API
- Produces: 可导入 CCS、可离线测试、可生成 UniFlash TI-TXT 的比赛基线

- [ ] **Step 1: 建立测试矩阵**

文档必须记录：传感器极性、X1～X8 左右顺序、M2/M4 前进方向、定距误差、定角误差、丢线停车时间、watchdog 停车时间、车体尺寸和电池安全检查。

- [ ] **Step 2: 运行完整离线测试**

Run: `python -m unittest discover -s tests -v`

Expected: 原有 22 项测试加新增契约测试全部通过，0 failures。

- [ ] **Step 3: 编译 CCS 工程**

Run from `MSPM0G3507_LineFollowing_Car/Debug`: `D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe -j4 all`

Expected: 生成 `MSPM0G3507_LineFollowing_Car.out`，exit code 0。

- [ ] **Step 4: 生成 UniFlash 文件**

Run from Debug: `D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmhex.exe --ti_txt --outfile=MSPM0G3507_LineFollowing_Car.txt MSPM0G3507_LineFollowing_Car.out`

Expected: TI-TXT 以 `@0000` 开始并以 `q` 结束。

- [ ] **Step 5: 提交前检查**

Run: `git diff --check; git status --short --branch`

Expected: 无空白错误；不包含 `Debug/` 生成物、不包含 `my-project/`；每个任务保持独立提交。

## 不在本计划内的内容

- 不购买或切换到新的履带/麦克纳姆/Arduino 一体化平台。
- 不在未知摄像头或测距硬件下猜测 SysConfig 引脚。
- 不把旧 `questions.c` 的硬编码比赛参数直接复制进新路线层。
- 不在没有架空轮和断电措施的情况下自动启动真实电机。
