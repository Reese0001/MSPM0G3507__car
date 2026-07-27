# RESET 自动启动与 OLED 流动日志实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 恢复 RESET 后自动进入电机运行流程，并让 SSD1306 以 8 行滚动行为日志记录启动、配置、ARM、实际速度和故障位置。

**Architecture:** `RuntimeLog` 只负责固定长度环形缓冲区和文本快照；启动阶段记录并立即刷新关键启动事件，运行阶段由最低优先级 `DisplayTask` 统一采集状态、写入事件并刷新 OLED。控制任务和安全任务只更新已有邮箱/安全状态，不直接访问 OLED。电机安全层提供只读诊断快照，保证日志能区分实际输出、方向等待、UART 超时和看门狗。

**Tech Stack:** MSPM0G3507 裸机 + FreeRTOS 静态任务、TI Arm Clang 4.0.4、TI DriverLib、SSD1306 软件 I2C、Python `unittest` 主机契约测试、TI `gmake` clean build。

## Global Constraints

- RESET 后自动启动；K1 不作为启动门，`line_start_ready` 不得控制 ARM。
- 所有电机输出继续经过 `Motor_Safety_*`；保留 0→30% 软启动、命令限幅、方向切换停顿和 200 ms 看门狗。
- OLED 仅为诊断终端；OLED I2C 失败不得绕过安全层或直接操作电机。
- OLED 只显示 ASCII，最多 8 行、每行最多 21 个字符；禁止 `malloc`。
- 电机配置发送必须有硬件时间上限，不能用无限 busy-wait。
- 首次实车验证必须架空驱动轮，再接通 12.6 V；先确认完整 OLED 日志。
- 每个代码任务先写失败测试，再写最小实现；每个任务结束运行对应测试并单独提交。

---

## 文件边界

- Create: `MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.h` — 日志容量、事件 API、快照 API。
- Create: `MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.c` — 固定环形缓冲、重复事件抑制、SSD1306 绘制。
- Modify: `MSPM0G3507_LineFollowing_Car/app/boot/app_boot.c` — 记录 BOOT/OLED/AUTO START/MOTOR CFG/CFG OK，并暴露配置成功状态。
- Modify: `MSPM0G3507_LineFollowing_Car/app/boot/app_boot.h` — 暴露 `AppBoot_IsMotorConfigured()` 与 `AppBoot_IsDisplayReady()`。
- Modify: `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c` — 移除循迹启动门，使用自动启动；DisplayTask 统一记录运行事件。
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/safety/motor_safety.h` — 增加安全诊断快照类型和 getter。
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/safety/motor_safety.c` — 保存实际 M2/M4 输出、方向等待和故障原因。
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/configuration/motor_configuration.h` — `Set_Motor` 返回 `bool`。
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/configuration/motor_configuration.c` — 配置帧改用有界发送，任一帧失败立即返回 false。
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/protocol/motor_protocol.h` — 配置发送函数返回 `bool`。
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/protocol/motor_protocol.c` — 配置帧统一调用 `Motor_Usart_SendArrayBounded`。
- Modify: `MSPM0G3507_LineFollowing_Car/Makefile` — 加入 `modules/display/runtime_log.c`，保持 `rebuild` 产物目录规则。
- Modify: `tests/test_freertos_contract.py` — 验证 RESET 自动启动且不再由 `LineStartGate` 控制 ARM。
- Modify: `tests/test_oled_contract.py` — 验证 RuntimeLog、100 ms 刷新和 OLED 不触碰电机安全层。
- Create: `tests/runtime_log_harness.c` — 主机环形缓冲和去重测试桩。
- Create: `tests/test_runtime_log.py` — 编译运行 RuntimeLog 主机测试。
- Modify: `tests/motor_direction_interlock_harness.c` — 验证安全诊断 getter 的实际输出和方向等待。
- Modify: `tests/test_motor_direction_interlock.py` — 验证新增诊断接口仍使用生产安全层。
- Create: `tests/test_motor_configuration_contract.py` — 验证配置函数返回值和有界 UART 发送。
- Modify: `README.md`、`MSPM0G3507_LineFollowing_Car/README.md`、`docs/setup/SETUP_GUIDE.md` — 写入新启动日志示例、架空轮检查点和 clean build/UniFlash-TXT 流程。

### Task 1: 添加固定长度 RuntimeLog 模块

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.c`
- Create: `tests/runtime_log_harness.c`
- Create: `tests/test_runtime_log.py`

**Interfaces:**
- Produces `RuntimeLog_Init`, `RuntimeLog_Push`, `RuntimeLog_PushMotor`, `RuntimeLog_Snapshot`, `RuntimeLog_Draw`.
- `RuntimeLog_Snapshot(char out[8][22])` 按最旧到最新返回当前行数，空行填 `'\0'`。

- [ ] **Step 1: 写失败测试**

在 `tests/test_runtime_log.py` 中编译 `runtime_log_harness.c` 与生产 `runtime_log.c`，断言：

```python
def test_ring_order_and_capacity(self):
    result = subprocess.run([str(self.exe)], capture_output=True, text=True)
    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
    self.assertIn("0000 BOOT", result.stdout)
    self.assertIn("0007 E7", result.stdout)
    self.assertNotIn("0000 E0", result.stdout)

def test_duplicate_payload_is_suppressed(self):
    self.assertIn("DEDUP_OK", self.run_harness())
```

`runtime_log_harness.c` 先调用尚不存在的 API，并提供 `Ssd1306_DrawText`/`Ssd1306_FlushDirty` 桩。

- [ ] **Step 2: 运行失败测试**

Run:

```powershell
python -m unittest tests.test_runtime_log -v
```

Expected: FAIL，因为 `runtime_log.h/.c` 尚不存在。

- [ ] **Step 3: 写最小实现**

`runtime_log.h` 固定接口：

```c
#define RUNTIME_LOG_CAPACITY 8U
#define RUNTIME_LOG_LINE_CHARS 21U
#define RUNTIME_LOG_LINE_BUFFER (RUNTIME_LOG_LINE_CHARS + 1U)

void RuntimeLog_Init(void);
bool RuntimeLog_Push(uint32_t now_ms, const char *event);
bool RuntimeLog_PushMotor(uint32_t now_ms, int16_t left, int16_t right);
uint8_t RuntimeLog_Snapshot(
    char out[RUNTIME_LOG_CAPACITY][RUNTIME_LOG_LINE_BUFFER]);
void RuntimeLog_Draw(void);
```

实现使用 `char lines[8][22]`、`head`、`count` 和 `last_payload[18]`；用 `snprintf` 生成
`%04lu EVENT`，时间取 `now_ms % 10000U`，超过 21 个字符时在第 21 个字符截断并写入终止符。
重复判断只比较事件正文，不比较时间戳；`RuntimeLog_PushMotor` 生成
`TX L%03d R%03d`，并将左右值限制在 `-999..999`。`RuntimeLog_Draw` 只调用
`Ssd1306_DrawText(page, 0, line)`，不调用任何电机 API。

- [ ] **Step 4: 运行测试**

Run:

```powershell
python -m unittest tests.test_runtime_log -v
```

Expected: PASS，容量为 8、顺序为旧到新、相同事件正文不连续刷屏。

- [ ] **Step 5: Commit**

```powershell
git add tests/test_runtime_log.py tests/runtime_log_harness.c MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.h MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.c
git commit -m "feat: add fixed OLED runtime log buffer"
```

### Task 2: 让电机配置发送有界，并记录配置成败

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/protocol/motor_protocol.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/protocol/motor_protocol.c`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/configuration/motor_configuration.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/configuration/motor_configuration.c`
- Modify: `MSPM0G3507_LineFollowing_Car/app/boot/app_boot.h`
- Modify: `MSPM0G3507_LineFollowing_Car/app/boot/app_boot.c`
- Create: `tests/test_motor_configuration_contract.py`

**Interfaces:**
- `bool send_motor_type(...)`, `bool send_pulse_phase(...)`, `bool send_pulse_line(...)`,
  `bool send_wheel_diameter(...)`, `bool send_motor_deadzone(...)`.
- `bool Set_Motor(int MOTOR_TYPE)`.
- `bool AppBoot_IsMotorConfigured(void)` and `bool AppBoot_IsDisplayReady(void)`.

- [ ] **Step 1: 写失败契约测试**

在 `tests/test_motor_configuration_contract.py` 断言所有 5 个配置函数和 `Set_Motor`
声明中包含 `bool`，实现中包含 `Motor_Usart_SendArrayBounded`，且不存在配置函数内部的
`Send_Motor_ArrayU8`；断言 `app_boot.c` 按 `MOTOR CFG` → `Set_Motor(5)` →
`CFG OK`/`UART TIMEOUT` 记录。

- [ ] **Step 2: 运行失败测试**

```powershell
python -m unittest tests.test_motor_configuration_contract -v
```

Expected: FAIL，因为当前配置 API 为 `void` 且使用无限等待发送。

- [ ] **Step 3: 写最小实现**

每个协议函数先 `snprintf` 到现有 `send_buff`，检查长度范围，再执行：

```c
return Motor_Usart_SendArrayBounded(send_buff, (uint16_t)length);
```

`Set_Motor(5)` 采用 `bool ok = true`，每一步用 `ok = ok && send_...();`
并在每个成功步骤后保留原有 100 ms 延时；任一步失败立即返回 false，不继续向驱动板发送。
`AppBoot_Init` 在 `Timer_Init` 后调用：

```c
RuntimeLog_Init();
RuntimeLog_Push(0U, "BOOT");
display_ready = Ssd1306_Init();
RuntimeLog_Push(Get_Time(), display_ready ? "OLED OK" : "OLED FAIL");
RuntimeLog_Push(Get_Time(), "AUTO START");
RuntimeLog_Push(Get_Time(), "MOTOR CFG");
motor_configured = Set_Motor(5);
RuntimeLog_Push(Get_Time(), motor_configured ? "CFG OK" : "UART TIMEOUT");
```

每次写入关键事件后若 `display_ready` 为 true，调用 `RuntimeLog_Draw` 和
`Ssd1306_FlushDirty`；OLED 失败只把 `display_ready` 置 false，不调用电机 API。

- [ ] **Step 4: 运行测试**

```powershell
python -m unittest tests.test_motor_configuration_contract tests.test_boot_timebase_contract -v
```

Expected: PASS；配置发送都经过有界 UART，`Timer_Init` 仍先于 `Set_Motor(5)`。

- [ ] **Step 5: Commit**

```powershell
git add tests/test_motor_configuration_contract.py MSPM0G3507_LineFollowing_Car/modules/motor/protocol/motor_protocol.* MSPM0G3507_LineFollowing_Car/modules/motor/configuration/motor_configuration.* MSPM0G3507_LineFollowing_Car/app/boot/app_boot.*
git commit -m "fix: bound motor configuration transmission"
```

### Task 3: 暴露电机安全层只读诊断快照

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/safety/motor_safety.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/safety/motor_safety.c`
- Modify: `tests/motor_direction_interlock_harness.c`
- Modify: `tests/test_motor_direction_interlock.py`

**Interfaces:**

```c
typedef enum {
    MOTOR_SAFETY_FAULT_NONE = 0,
    MOTOR_SAFETY_FAULT_UART_TIMEOUT,
    MOTOR_SAFETY_FAULT_WATCHDOG
} MotorSafetyFaultReason;

typedef struct {
    int16_t left_applied;
    int16_t right_applied;
    bool direction_wait;
    MotorSafetyFaultReason fault_reason;
} MotorSafetyDiagnostics;

void Motor_Safety_GetDiagnostics(MotorSafetyDiagnostics *out);
```

- [ ] **Step 1: 写失败测试**

扩展现有 host harness：ARM 后请求左轮正转、调用 `Motor_Safety_Service`，断言
快照为 `left_applied == 0`/`right_applied == 0`（初始软启动阶段）；推进时间并再次
请求反向，断言 `direction_wait` 为 true；模拟发送失败和 200 ms 无刷新，分别断言
故障原因是 `MOTOR_SAFETY_FAULT_UART_TIMEOUT` 与 `MOTOR_SAFETY_FAULT_WATCHDOG`。

- [ ] **Step 2: 运行失败测试**

```powershell
python -m unittest tests.test_motor_direction_interlock.MotorDirectionInterlockRuntime -v
```

Expected: FAIL，因为当前没有诊断类型和 getter。

- [ ] **Step 3: 写最小实现**

在 `motor_safety.c` 中新增 `fault_reason`、`direction_wait`，成功 `apply_speed`
时从实际 `output[1]`/`output[3]` 更新左右值；任一发送失败时写入 UART 超时原因；
`Motor_Safety_Tick1ms` 触发看门狗时写入 WATCHDOG。`Motor_Safety_GetDiagnostics`
使用现有临界区复制 4 个字段，返回后不暴露内部数组。`Motor_Safety_Init` 清零故障原因。

- [ ] **Step 4: 运行测试**

```powershell
python -m unittest tests.test_motor_direction_interlock -v
```

Expected: PASS，原有方向互锁行为不变，新增 getter 能观察实际输出与故障来源。

- [ ] **Step 5: Commit**

```powershell
git add tests/motor_direction_interlock_harness.c tests/test_motor_direction_interlock.py MSPM0G3507_LineFollowing_Car/modules/motor/safety/motor_safety.*
git commit -m "feat: expose motor safety diagnostics"
```

### Task 4: 恢复 RESET 自动启动并接入流动日志

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c`
- Modify: `MSPM0G3507_LineFollowing_Car/Makefile`
- Modify: `tests/test_freertos_contract.py`
- Modify: `tests/test_oled_contract.py`

**Interfaces:**
- `SafetyTask` 固定提供 `inputs.start_pressed = true`，不再读取 `line_start_ready`。
- `DisplayTask` 每 100 ms 读取 `Motor_Safety_GetDiagnostics`、`SafetySupervisor_GetState`
  和 `LineRecovery_GetState`，按状态变化调用 `RuntimeLog_Push`。

- [ ] **Step 1: 写失败契约测试**

修改测试断言：

```python
self.assertIn("inputs.start_pressed = true;", tasks)
self.assertNotIn("inputs.start_pressed = line_start_ready;", tasks)
self.assertNotIn("line_start_ready = LineStartGate_Update", tasks)
self.assertIn("RuntimeLog_Draw", tasks)
self.assertIn("pdMS_TO_TICKS(100U)", tasks)
```

同时断言 DisplayTask 不出现 `Motor_Safety_RequestSpeed`、`Motor_Safety_Arm` 或
`Motor_Safety_Disarm`，保证 OLED 仍是只读诊断端。

- [ ] **Step 2: 运行失败测试**

```powershell
python -m unittest tests.test_freertos_contract tests.test_oled_contract -v
```

Expected: FAIL，因为当前仍由 `line_start_ready` 控制启动且 DisplayTask 是 200 ms 固定面板。

- [ ] **Step 3: 写最小实现**

在 `SafetyTask` 删除 `LineStartGate` 的 include、状态变量、Reset/Update 调用，将
`inputs.start_pressed` 设为 `true`；将 `inputs.power_qualified` 改为配置宏与
`AppBoot_IsMotorConfigured()` 的 AND，配置失败时永远不能进入 RUNNING。

将 `DisplayTask` 改为 100 ms 周期，并维护上次观察值：

```c
if (diag.fault_reason == MOTOR_SAFETY_FAULT_UART_TIMEOUT)
    RuntimeLog_Push(now_ms, "UART TIMEOUT");
if (diag.fault_reason == MOTOR_SAFETY_FAULT_WATCHDOG)
    RuntimeLog_Push(now_ms, "WATCHDOG");
if (diag.direction_wait)
    RuntimeLog_Push(now_ms, "DIR WAIT");
if (diag.left_applied != last_left || diag.right_applied != last_right)
    RuntimeLog_PushMotor(now_ms, diag.left_applied, diag.right_applied);
if (recovery != last_recovery && recovery == LINE_RECOVERY_STOPPED)
    RuntimeLog_Push(now_ms, "LINE LOST");
if (!motor_armed && state == SAFETY_RUNNING)
    RuntimeLog_Push(now_ms, "MOTOR ARM");
RuntimeLog_Draw();
display_ready = Ssd1306_FlushDirty();
```

仅在事件值变化或首次出现时写入；OLED 初始化失败时每周期重试，成功后写 `OLED OK`
并恢复绘制。删除旧 `Dashboard_Render` 调用，但保留未使用的 dashboard 源文件，不扩大本次范围。
Makefile 的 `SOURCES` 增加 `modules/display/runtime_log.c`。

- [ ] **Step 4: 运行测试**

```powershell
python -m unittest tests.test_freertos_contract tests.test_oled_contract tests.test_motor_authority -v
```

Expected: PASS；RESET 自动经过 `BOOT_SAFE → READY → RUNNING`，只有 SafetyTask ARM，
OLED 任务不拥有电机写权限。

- [ ] **Step 5: Commit**

```powershell
git add tests/test_freertos_contract.py tests/test_oled_contract.py MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c MSPM0G3507_LineFollowing_Car/Makefile
git commit -m "feat: restore reset auto-start and rolling OLED diagnostics"
```

### Task 5: 完成构建、文档和 UniFlash-TXT 检查点

**Files:**
- Modify: `README.md`
- Modify: `MSPM0G3507_LineFollowing_Car/README.md`
- Modify: `docs/setup/SETUP_GUIDE.md`
- Create: `tests/test_runtime_log_integration.py`

**Interfaces:**
- 文档中的启动日志统一使用：

```text
0000 BOOT
0012 OLED OK
0020 AUTO START
0022 MOTOR CFG
0525 CFG OK
0526 MOTOR ARM
0626 TX L030 R030
0726 TX L060 R060
```

- [ ] **Step 1: 写失败文档/构建契约**

在 `tests/test_runtime_log_integration.py` 断言三个文档包含 `RESET`、`AUTO START`、
`MOTOR CFG`、`CFG OK`、`MOTOR ARM`、`TX L030 R030`，并包含架空驱动轮警告和
`rebuild` 命令；断言 `Makefile` 的 runtime log 源文件只出现一次。

- [ ] **Step 2: 运行失败测试**

```powershell
python -m unittest tests.test_runtime_log_integration -v
```

Expected: FAIL，文档仍描述旧的 K1 启动门或缺少完整日志。

- [ ] **Step 3: 更新文档**

明确 K1 不参与启动；将首次测试步骤写成“断开/架空驱动轮 → 接通 12.6 V →
RESET → 观察 OLED 最后一行”；保留串口 115200 和 PA10/PA11 OLED 引脚说明。
说明任何 `UART TIMEOUT`、`WATCHDOG`、`DIR WAIT`、`LINE LOST` 都要先停电再反馈，
不得带轮落地重复按 RESET。

- [ ] **Step 4: 运行全量验证**

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C MSPM0G3507_LineFollowing_Car rebuild
Get-Item 'dist\firmware\MSPM0G3507_LineFollowing_Car.txt' | Select-Object FullName,Length,LastWriteTime
Get-FileHash 'dist\firmware\MSPM0G3507_LineFollowing_Car.txt' -Algorithm SHA256
```

Expected: 全量主机测试通过；`rebuild` 先清理 `build/cli/.../obj`，再生成 `.out/.hex/.txt`；
最终只把 `dist/firmware/MSPM0G3507_LineFollowing_Car.txt` 提供给 UniFlash。

- [ ] **Step 5: Commit**

```powershell
git add README.md MSPM0G3507_LineFollowing_Car/README.md docs/setup/SETUP_GUIDE.md tests/test_runtime_log_integration.py
git commit -m "docs: document reset startup log and UniFlash checkpoint"
```

## Checkpoints

1. Task 1 后：主机环形缓冲通过，尚未接入电机。
2. Task 2 后：电机配置不会无限阻塞，失败可停在 `MOTOR CFG`/`UART TIMEOUT`。
3. Task 3 后：安全层能区分实际输出、方向等待和看门狗。
4. Task 4 后：RESET 自动 ARM，OLED 只读且 100 ms 滚动刷新。
5. Task 5 后：全量测试、clean build 和 UniFlash TI-TXT 均有可复制命令。

