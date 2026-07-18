# 最新 CCS 工程整合与编码治理设计

日期：2026-07-18

## 目标

以临时目录 `workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang/` 中的工程作为最新源码来源，替换仓库根目录现有的同名 CCS 工程。整合时统一维护文本为 UTF-8、修复中文显示问题，并在最新版循迹程序进入主线前补齐电机软启动与定时器失控保护。验证完成后删除整个临时 `workspace_ccstheia/`。

## 权威范围

仅以下目录作为最新版代码来源：

```text
workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang/
```

临时目录中的下列内容不进入主线：

- `docs_backup/`
- `.env`
- `.claude/`、`.theia/`、`.vscode/`
- `Yuan/`
- `download*.txt`
- 临时目录自身的 `CLAUDE.md`、`PROJECT_SKILLS.md`、`.mcp.json`
- `Debug/` 和其他生成物

仓库根目录现有 Git、规范文档、精选 `docs/` 和私有远程配置保持不变。

## 已确认差异

排除 `Debug/` 后，新旧 CCS 工程有 19 个文件哈希不同，其中18个是源码或 SysConfig 文件。最新版 `empty.c` 已从 MPU6050 地址扫描切换为八路灰度循迹主程序。

最新版文本编码审计结果：

- 41个维护文本文件为有效 UTF-8；
- `BSP/eMPL/inv_mpu.h` 为 GBK；
- `BSP/Motor/app_motor_usart.h` 为 GBK；
- `BSP/MPU6050/app_mpu6050.h` 为 GBK。

最终工程的维护文本全部转换为 UTF-8无BOM。转换必须先严格判断编码，只对无法按严格UTF-8解码且能按GBK正确解码的文件执行 GBK→UTF-8；禁止对已是UTF-8的文件再次转码。

## 替换策略

1. 记录当前工程与临时最新版工程的相对路径、大小和 SHA-256。
2. 将当前工程复制到可恢复备份区，仅用于整合失败时恢复。
3. 从临时最新版复制非生成文件到根目录同名工程。
4. 删除根目录工程中最新版已不存在的旧文件，但不删除 CCS 必需元数据。
5. 对维护文本执行严格编码审计和三文件定向转换。
6. 在最新版代码上实现电机安全层。
7. 运行结构、编码、静态检查和可用的 CCS 构建验证。
8. 验证通过后删除恢复备份和整个 `workspace_ccstheia/`。

任何步骤失败都不得删除临时最新版来源。

## 电机安全 Checklist

已确认并按以下前提设计：

- L型520电机，配置入口使用 `MOTOR_TYPE=5`；
- 电机通信继续使用 UART1：PB6 TX / PB7 RX；
- 带电测试前断开12.6V电机电源或将车轮悬空；
- 禁止直接输出100% PWM；主程序使用速度协议而非直接PWM；
- 上电首先发送零速；
- 软启动为1秒、10级，从0提升到目标速度的30%；
- 软启动之后仍限制单次速度上升，避免从30%跳到完整目标；
- 控制更新超时200 ms后，由1 ms定时器监控触发零速并锁定；
- 故障锁定后必须重新初始化才能恢复。

## 安全层架构

新增：

```text
BSP/Motor/motor_safety.h
BSP/Motor/motor_safety.c
```

### 状态

- `DISARMED`：上电默认状态，只允许零速。
- `RAMPING`：初始化后执行1秒10级软启动。
- `RUNNING`：软启动完成，仍应用速度上升斜率限制。
- `FAULT_LATCHED`：超过200 ms无控制更新，持续保持零速，普通控制请求不能清除。

### 接口

- `Motor_Safety_Init(void)`：清零目标和输出，发送零速，进入 `DISARMED`。
- `Motor_Safety_Arm(void)`：仅在电机类型配置完成后进入 `RAMPING`，记录起始时刻。
- `Motor_Safety_RequestSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)`：记录目标速度并刷新控制心跳，不直接绕过限幅。
- `Motor_Safety_Service(void)`：由主循环周期调用，根据10级软启动和持续斜率限制计算并发送安全速度。
- `Motor_Safety_Tick1ms(void)`：由1 ms定时器中断调用；累计控制心跳年龄，达到200 ms时锁存故障并调用有界紧急零速发送。
- `Motor_Safety_IsFaultLatched(void)`：供调试和主程序判断。

`Motion_Car_Control()` 和 `Motion_Yaw_Calc()` 不再直接调用 `Contrl_Speed()`，而是提交给 `Motor_Safety_RequestSpeed()`。显式零速命令可以立即降低输出，不受下降斜率限制。

### 紧急零速

定时器中断内禁止使用 `sprintf`、动态内存或无界等待。新增固定帧紧急发送函数，发送：

```text
$spd:0,0,0,0#
```

每字节等待必须有固定循环上限。无论发送结果如何，状态均保持 `FAULT_LATCHED`，主循环后续服务继续发送零速。

## 最新主程序调整

最新版主程序的控制流程调整为：

1. `SYSCFG_DL_init()`；
2. `USART_Init()` 和新增的 `Timer_Init()`；`Timer_Init()` 必须清除待处理中断、使能 `TIMER_0_INST_INT_IRQN` 并启动 `TIMER_0_INST`；
3. `Motor_Safety_Init()`，立即零速；
4. `Set_Motor(5)`，只执行一次经过确认的L型520配置，不再每秒重复配置；
5. `Motor_Safety_Arm()`；
6. 主循环执行 `LineWalking()`；
7. 每轮调用 `Motor_Safety_Service()`；
8. 保持约10 ms控制周期。

删除原最新版每100次循环重复三次 `Set_Motor(1)` 的逻辑。重复配置不是心跳机制，也会阻塞控制循环约1.5秒。

## 编码治理

最终审计范围包括 CCS 工程内的：

- `.c`、`.h`
- `.md`、`.txt`
- `.syscfg`
- `.project`、`.cproject`、`.ccsproject`
- `.json`、`.xml`、`.prefs`

排除二进制、目标文件和重新生成的 `Debug/`。验收条件：

- 所有维护文本均可被严格UTF-8解码；
- 不包含 Unicode替换字符 `U+FFFD`；
- 不包含已知二次乱码特征；
- 三个原GBK头文件的中文注释经转换后可读；
- C源码字面量和预处理器结构未因转码改变。

## 验证

### 文件与工程结构

- 新版来源清单与落位清单逐项对应；
- `.project`、`.cproject` 可解析；
- `empty.syscfg` 存在；
- `.cproject` 中所有 `${PROJECT_ROOT}/BSP/...` 路径存在；
- `Debug/` 不进入Git。

### 安全静态检查

- `empty.c` 不出现 `Set_Motor(1)`；
- `Set_Motor(5)` 仅在初始化阶段出现一次；
- `Motion_Car_Control()` 和 `Motion_Yaw_Calc()` 不直接发送非零速度；
- 主循环调用 `Motor_Safety_Service()`；
- 定时器ISR调用 `Motor_Safety_Tick1ms()`；
- 主程序在 `Motor_Safety_Arm()` 前调用 `Timer_Init()`，且 `Timer_Init()` 同时使能NVIC和启动计数器；
- 超时常量固定为200 ms；
- 软启动常量固定为1000 ms和10级；
- 紧急路径只发送固定零速帧。

### 构建

- 优先通过 CCS Theia执行一次 Clean Project和Build Project；
- 若工具链绝对路径失效，记录准确缺失路径，不用旧日志冒充新构建；
- 构建成功后检查新 `Debug/*.out`，但保持其Git忽略状态。

### 硬件测试边界

本次整合默认只进行断电/车轮悬空验证。任何12.6V带载测试必须由用户现场确认后单独进行，不能由目录整合任务自动触发。

## 删除临时目录门禁

只有以下条件全部满足后才能删除 `workspace_ccstheia/`：

1. 最新源码全部落位；
2. UTF-8审计通过；
3. 电机安全静态检查通过；
4. CCS结构验证通过；
5. 新工程已提交并推送私有GitHub；
6. 用户确认不再需要临时来源副本。

删除前解析绝对路径，必须精确等于：

```text
D:\DevProject\MSPM0G3507__car\workspace_ccstheia
```

禁止使用通配符或递归删除其他工作区路径。
