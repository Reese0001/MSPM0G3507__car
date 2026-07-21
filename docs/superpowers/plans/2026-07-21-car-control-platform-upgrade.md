# 小车控制备赛平台升级实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 基于已购买的 228 mm×148 mm 两驱 L 型 520 霍尔编码器底盘、MSPM0G3507、MSPM0 扩展板和四路编码器驱动板，建立先不依赖摄像头、可覆盖循迹、定距直行、定角转向和比赛路线状态机的安全控制平台。

**Architecture:** 保留现有 `Eight_Tracking`、`Motor`、`MPU6050` 和 `motor_safety` 底层驱动，在其上新增统一的 `CarControl` 运动与路线层。主循环先完成灰度、编码器和 IMU 三闭环；摄像头和避障属于后置可选阶段，目前不购买、不改 SysConfig、不阻塞基础小车开发。

**Tech Stack:** MSPM0G3507、TI DriverLib、TI Arm Clang 4.0.4、MSPM0 SDK 2.10.00.04、CCS Theia、C 裸机、Python `unittest`。

## Global Constraints

- `empty.syscfg` 是引脚与外设配置的唯一真实来源；摄像头尚未购买，未确认型号和引脚前，不新增 UART、I2C 或 GPIO 配置。
- 当前底盘保持 M2=左驱动轮、M4=右驱动轮、M1/M3=0；不改为履带或麦克纳姆轮。
- 已购底盘尺寸为 228 mm×148 mm×102.15 mm，满足当前已知 35 cm×25 cm 长宽限制；正式题目若增加高度限制，重新复核含传感器后的整车高度。
- 已购电机为 12 V L 型 520、11 线 AB 相增量编码器，减速后标称约 300 rpm；编码器接口电平和驱动板协议必须以实测/手册为准。
- 图片标注驱动板和扩展板输入范围为 5–12 V，而电池满电最高 12.6 V；在确认绝对最大额定电压前，禁止把满电电池直接接入板卡。
- 12 V 电池标注约 1.3C/6 A，两个 L 型 520 电机的堵转电流可能同时超过电池持续放电能力；首轮测试必须限制加速度、避免堵转，并准备保险丝或可立即断电的电源开关。
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
- 当前工程没有摄像头、避障和无线任务输入接口；摄像头未购买，第一阶段不实现视觉功能，只保留后续适配边界。

---

### Task 0: 对齐 OpenSpec 与项目宪章

**Files:**
- Modify: `openspec/config.yaml`
- Modify: `openspec/specs/motor-control/spec.md`
- Modify: `openspec/specs/competition-state-machine/spec.md`
- Modify: `my-project/.specify/memory/constitution.md`（当前为未跟踪的 Spec Kit 工具目录，需单独保留其变更）

**Interfaces:**
- Produces: 与已购 65 mm 两驱平台、电源安全门槛、摄像头未购买边界和 `CarRoute`/旧 `questions.c` 兼容策略一致的规范基线。

- [x] **Step 1: 解决已确认冲突**

以购买资料中的 65 mm 作为初始轮径，编码器换算保持未标定；保留旧 `questions.c` 接口兼容，由 `CarRoute` 承担实际运动流程。

- [x] **Step 2: 移除项目宪章占位符**

将 `my-project/.specify/memory/constitution.md` 替换为 MSPM0 小车项目规则；Spec Kit 模板和脚本不修改、不删除。

- [x] **Step 3: 记录规范变更**

在 OpenSpec 配置中记录 5–12 V 板卡与 12.6 V 电池的硬件验收门槛、65 mm 初始轮径和摄像头后置策略。

- [x] **Step 4: 复核并提交**

Run: `rg -n "\[PROJECT_NAME\]|67mm|67 mm" my-project/.specify/memory/constitution.md openspec/specs openspec/config.yaml`

Expected: 项目宪章无模板占位符，运动规格不再使用未经确认的 67 mm；提交时不得把 `.superpowers/` 或构建产物加入 Git。

---

### Task 1: 固化已购平台与电源验收基线

**Files:**
- Create: `docs/setup/CAR_PLATFORM_CONTRACT.md`
- Modify: `README.md`
- Test: `tests/test_car_platform_contract.py`

**Interfaces:**
- Consumes: 当前 `AGENTS.md`、`docs/setup/SETUP_GUIDE.md`、`empty.syscfg`、电机 UART 协议和已购平台图片
- Produces: `CarPlatformContract` 文档中的确定参数：228×148×102.15 mm 底盘、M2/M4 映射、11 线 AB 编码器、灰度电平、电源输入验收和未购买摄像头的明确边界

- [ ] **Step 1: 写入基线测试**

在 `tests/test_car_platform_contract.py` 中检查以下字符串和数值存在：`MOTOR_TYPE=5`、`PB6`、`PB7`、`PA15`、`PA16`、`PA17`、`PA18`、`M2`、`M4`、`MOTOR_SAFETY_WATCHDOG_MS`、`228`、`148`、`102.15`、`12.6`；同时检查主工程路径为 `MSPM0G3507_LineFollowing_Car`，并检查计划文档写明“摄像头未购买”。

- [ ] **Step 2: 运行基线测试**

Run: `python -m unittest tests.test_car_platform_contract -v`

Expected: 新测试在文档/API尚未完成时失败，失败项明确指出缺失的硬件契约。

- [ ] **Step 3: 编写平台契约**

文档必须逐项记录：底盘尺寸 228×148×102.15 mm、整车长宽限制 35 cm×25 cm、两驱 M2/M4、L 型 520 12 V/11 线 AB 编码器、灰度 PA15～PA18、UART0 调试 115200、UART1 电机 115200、黑线低电平约定、编码器帧 `MTEP`/`MAll`/`MSPD` 的含义、驱动板 5–12 V 标注与 12.6 V 满电电池的电压核验门槛，以及“摄像头未购买，暂不改 SysConfig”。

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

### Task 3: 统一编码器和底盘运动接口（安全接口阶段）

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

在 `app_motor_usart.c` 中保留 `Encoder_Offset[]`、`g_Speed[]` 的协议解析，但新增带时间戳的快照读取；只使用 M2 和 M4 计算左右轮反馈，并明确正负方向。11 线 AB 编码器的脉冲/圈、减速比和轮径必须由驱动板上报或实测确认；未确认编码器单位前，`CarMotionFeedback` 必须标记 `units_valid=false` 或由 `CarMotion_ReadFeedback()` 返回 `false`，禁止把原始计数伪装成毫米。

同时从 `app_motor.h` 移除对 `questions.h` 的反向包含，保留 `app_motor.h` 只依赖 DriverLib 和电机协议头，避免新 `CarControl` 层与旧题目状态机形成循环包含。

- [ ] **Step 3: 建立安全门控的非阻塞动作接口**

在编码器脉冲/圈、驱动板速度单位、有效轮径和轴距未实测前，`CarMotion_DriveDistanceStart()` 与 `CarMotion_TurnAngleStart()` 必须 fail-closed：不发起运动、经安全层停车并返回 `false`。不得用参考资料中的 260 脉冲/圈或 67 mm 轮径替代实测值。完成标定后，另行执行 Task 3B 实现真正的起始反馈、误差推进、阈值停车和角度闭环；禁止使用固定 `delay_ms()` 完成转弯。

- [ ] **Step 4: 复测、构建并提交**

Run: `python -m unittest tests.test_car_motion_contract -v`

Run from Debug: `D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe -j4 all`

Expected: 测试 PASS；若 CCS Debug 已由 CCS 生成则构建 PASS，否则记录环境阻塞；提交：`git commit -m "feat: add safe car motion interfaces"`。

### Task 3B: 硬件标定后启用定距/定角闭环

**前置条件：** 已确认驱动板编码器反馈格式、脉冲/圈、减速比、有效轮径、轴距、左右轮符号和 MPU6050 yaw 可用；未满足时保持 Task 3 的 fail-closed 行为。

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/BSP/CarControl/car_motion.c`
- Modify: `tests/test_car_motion_contract.py`

**Requirements:**
- `DriveDistanceStart()` 保存 M2/M4 起始计数和目标距离；每次 `Step()` 读取一次有效快照，计算剩余误差，使用有界速度并在阈值内调用 `CarMotion_Stop()` 返回 `true`。
- `TurnAngleStart()` 保存起始 yaw 或左右轮差分；每次 `Step()` 使用新鲜反馈计算角度误差，完成后停车返回 `true`。
- 所有路径非阻塞、可超时、经 Motor Safety；增加有效反馈、完成停车和超时测试。

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

### Task 6: 暂缓摄像头，保留后置目标输入边界

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/README.md`
- Modify: `docs/setup/CAR_PLATFORM_CONTRACT.md`
- Create only after camera/测距硬件确认: `MSPM0G3507_LineFollowing_Car/BSP/CarControl/car_target_input.h`
- Create only after camera/测距硬件确认: `MSPM0G3507_LineFollowing_Car/BSP/CarControl/car_target_input.c`
- Modify only after hardware confirmation: `MSPM0G3507_LineFollowing_Car/empty.syscfg`
- Test only after adapter is created: `tests/test_car_target_input_contract.py`

**Interfaces:**
- 当前阶段只产出“视觉未启用”的文档约束，不新增固件 API。
- 后续购买摄像头或测距模块后，再按已确认的通信协议产出统一目标观测接口。

- [ ] **Step 1: 标记当前阶段不启用视觉**

在 README 和平台契约中明确：摄像头尚未购买，第一阶段不加入摄像头驱动、不修改 SysConfig、不把电脑 USB 摄像头接入 MSPM0 固件；小车基础功能只依赖灰度、编码器和 MPU6050。

- [ ] **Step 2: 暂不创建视觉代码**

完成 Task 1–5 前不创建 `car_target_input.c/.h`，避免为了不存在的硬件引入无效接口；路线层先只支持循迹、定距、定角和搜索线。

- [ ] **Step 3: 摄像头购买后再开独立变更**

只有拿到摄像头/测距模块型号、供电电压、UART/I2C 协议、处理器输出格式和所需引脚后，才创建 `CarTargetObservation` 适配层并修改 `empty.syscfg`；不允许把电脑 USB 摄像头协议直接写入 MSPM0 固件。

- [ ] **Step 4: 后置变更的接口**

摄像头购买后再创建以下接口，不提前编译进基础固件：

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

- [ ] **Step 5: 硬件确认后单独测试和提交（不属于本轮执行范围）**

Run: `python -m unittest tests.test_car_target_input_contract.py -v`

Expected: 适配层契约测试 PASS；提交：`git commit -m "feat: add confirmed target input adapter"`。未确认硬件时跳过本步骤，禁止创建空驱动占位。

### Task 7: 集成验证、文档和烧录包

**Files:**
- Modify: `README.md`
- Modify: `MSPM0G3507_LineFollowing_Car/README.md`
- Modify: `docs/setup/SETUP_GUIDE.md`
- Create: `docs/setup/CAR_CONTROL_TEST_MATRIX.md`
- Test: `tests/test_line_following_contract.py`
- Test: `tests/test_motor_safety_contract.py`

**Interfaces:**
- Consumes: Tasks 1–5 的公共 API，以及 Task 6 记录的“视觉未启用”约束
- Produces: 可导入 CCS、可离线测试、可生成 UniFlash TI-TXT 的比赛基线

- [ ] **Step 1: 建立测试矩阵**

文档必须记录：传感器极性、X1～X8 左右顺序、M2/M4 前进方向、定距误差、定角误差、丢线停车时间、watchdog 停车时间、车体尺寸和电池安全检查。

- [ ] **Step 2: 运行完整离线测试**

Run: `python -m unittest discover -s tests -v`

Expected: 原有 22 项测试加 Tasks 1–5 的新增契约测试全部通过，0 failures；摄像头适配测试仅在后续硬件确认后计入。

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

- 不更换已经购买的两驱 L 型 520 底盘，不切换到履带/麦克纳姆/Arduino 一体化平台。
- 当前不购买摄像头，不在未知摄像头或测距硬件下猜测 SysConfig 引脚。
- 不在确认驱动板 5–12 V 输入能否承受 12.6 V 满电电池前接通电池。
- 不把旧 `questions.c` 的硬编码比赛参数直接复制进新路线层。
- 不在没有架空轮和断电措施的情况下自动启动真实电机。
