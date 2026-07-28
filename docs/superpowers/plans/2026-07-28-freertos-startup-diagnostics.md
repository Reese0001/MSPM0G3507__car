# FreeRTOS Startup Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用临时 OLED 与 D1/D2 阶段码精确定位 `SCHED START` 到四任务入口之间的中断点，并在定位完成后无残留地撤销诊断代码。

**Architecture:** 应用层新增一个不依赖 FreeRTOS 的 `boot_trace` 小模块，普通阶段保存阶段码并设置两位 LED 状态，致命路径只使用 GPIO 闪码。FreeRTOS 使用既有 `traceSTARTING_SCHEDULER` 宏记录端口入口；两个极小汇编跳板记录 SVC 与首次上下文恢复，然后跳转到改名后的官方 TI CM0+ 端口实现。四任务用位图登记上线，SafetyTask 只在位图为 `0x0F` 后临时允许电机 Arm。

**Tech Stack:** MSPM0G3507、TI Arm Clang 4.0.4、MSPM0 SDK 2.10.00.04 FreeRTOS CM0+ 端口、DriverLib、Python `unittest`、TI-TXT/UniFlash。

## Global Constraints

- 诊断完成后删除本计划引入的运行时检查点，恢复 D1/D2 原用途和原任务速度。
- 不修改循迹算法、PID、电机协议、引脚配置或任务周期。
- 四任务全部上线前电机保持 Disarm；任何致命路径不得调用 OLED、格式化、UART 或 FreeRTOS API。
- 不修改 `C:\ti\mspm0_sdk_2_10_00_04` SDK 安装目录。
- 每次目标构建前同时清理 `freertos_kernel/Debug` 与应用 `build/cli` 缓存。

---

### Task 1: 锁定诊断契约

**Files:**
- Create: `tests/test_freertos_startup_diagnostics.py`

**Interfaces:**
- Consumes: 当前 `FreeRTOSConfig.h`、`empty.c`、`app/tasks/app_tasks.c` 与两个 Makefile。
- Produces: 阶段码、端口跳板、异常安全和四任务门控的静态契约。

- [ ] **Step 1: Write the failing contract test**

```python
from pathlib import Path
import unittest

REPO = Path(__file__).resolve().parents[1]
ROOT = REPO / "MSPM0G3507_LineFollowing_Car"


class FreeRtosStartupDiagnostics(unittest.TestCase):
    def test_all_startup_boundaries_are_named(self):
        header = (ROOT / "modules/diagnostics/boot_trace.h").read_text("utf-8")
        for name in (
            "BOOT_TRACE_MAIN", "BOOT_TRACE_TASKS_CREATED",
            "BOOT_TRACE_SCHED_START", "BOOT_TRACE_PORT_START",
            "BOOT_TRACE_SVC", "BOOT_TRACE_FIRST_RESTORE",
            "BOOT_TRACE_SAFETY_TASK", "BOOT_TRACE_SENSOR_TASK",
            "BOOT_TRACE_CONTROL_TASK", "BOOT_TRACE_DISPLAY_TASK",
            "BOOT_TRACE_ALL_TASKS",
        ):
            self.assertIn(name, header)

    def test_port_wrappers_route_to_official_handlers(self):
        wrapper = (ROOT / "modules/diagnostics/freertos_startup_wrappers.S").read_text("utf-8")
        kernel = (REPO / "freertos_kernel/Makefile").read_text("utf-8")
        self.assertIn("FreeRTOS_SVC_Handler", wrapper)
        self.assertIn("FreeRTOS_vRestoreContextOfFirstTask", wrapper)
        self.assertIn("-DSVC_Handler=FreeRTOS_SVC_Handler", kernel)
        self.assertIn("-DvRestoreContextOfFirstTask=FreeRTOS_vRestoreContextOfFirstTask", kernel)

    def test_fatal_path_is_gpio_only(self):
        source = (ROOT / "modules/diagnostics/boot_trace.c").read_text("utf-8")
        fatal = source[source.index("BootTrace_Fatal"):]
        for forbidden in ("RuntimeLog", "Ssd1306", "Motor_", "printf", "vTask"):
            self.assertNotIn(forbidden, fatal)

    def test_all_tasks_register_and_motor_waits_for_all(self):
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text("utf-8")
        for bit in (
            "BOOT_TASK_SAFETY", "BOOT_TASK_SENSOR",
            "BOOT_TASK_CONTROL", "BOOT_TASK_DISPLAY",
        ):
            self.assertIn(f"BootTrace_TaskOnline({bit})", tasks)
        arm = tasks.index("Motor_Safety_Arm();")
        self.assertIn("BootTrace_AllTasksOnline()", tasks[arm - 240:arm])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Verify RED**

Run: `python -m unittest tests.test_freertos_startup_diagnostics -v`

Expected: FAIL because `modules/diagnostics/boot_trace.h` and the port wrapper do not exist.

- [ ] **Step 3: Commit the failing test**

```powershell
git add -- tests/test_freertos_startup_diagnostics.py
git commit -m "test: specify freertos startup diagnostics"
```

---

### Task 2: 添加最小 BootTrace 状态机

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/modules/diagnostics/boot_trace.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/diagnostics/boot_trace.c`
- Modify: `MSPM0G3507_LineFollowing_Car/Makefile`

**Interfaces:**
- Produces: `BootTrace_Init(void)`、`BootTrace_Mark(BootTraceStage)`、`BootTrace_TaskOnline(uint8_t)`、`BootTrace_AllTasksOnline(void)`、`BootTrace_GetTaskMask(void)`、`BootTrace_Fatal(BootTraceFault)`。
- Consumes: `LED_PORT`、`LED_D1_PIN`、`LED_D2_PIN` DriverLib GPIO 定义。

- [ ] **Step 1: Add the public diagnostic codes**

```c
typedef enum {
    BOOT_TRACE_MAIN = 1,
    BOOT_TRACE_TASKS_CREATED = 2,
    BOOT_TRACE_SCHED_START = 3,
    BOOT_TRACE_PORT_START = 4,
    BOOT_TRACE_SVC = 5,
    BOOT_TRACE_FIRST_RESTORE = 6,
    BOOT_TRACE_SAFETY_TASK = 10,
    BOOT_TRACE_SENSOR_TASK = 11,
    BOOT_TRACE_CONTROL_TASK = 12,
    BOOT_TRACE_DISPLAY_TASK = 13,
    BOOT_TRACE_ALL_TASKS = 14
} BootTraceStage;

typedef enum {
    BOOT_FAULT_ASSERT = 1,
    BOOT_FAULT_HARDFAULT = 2,
    BOOT_FAULT_DEFAULT_IRQ = 3,
    BOOT_FAULT_STACK_OVERFLOW = 4,
    BOOT_FAULT_SCHED_RETURN = 5
} BootTraceFault;

#define BOOT_TASK_SAFETY  (1U << 0)
#define BOOT_TASK_SENSOR  (1U << 1)
#define BOOT_TASK_CONTROL (1U << 2)
#define BOOT_TASK_DISPLAY (1U << 3)
#define BOOT_TASK_ALL     (0x0FU)
```

- [ ] **Step 2: Implement static LED stage encoding and atomic task bits**

`BootTrace_Mark()` maps stages `03/04/05/06` to `D1:D2 = 00/01/10/11`. `BootTrace_TaskOnline()` updates the byte mask inside a saved-PRIMASK critical section and records `BOOT_TRACE_ALL_TASKS` when the mask becomes `0x0F`. `BootTrace_Fatal()` sets D1 and repeatedly blinks D2 `fault` times using a volatile CPU delay loop; it never returns.

```c
void BootTrace_TaskOnline(uint8_t bit)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    task_mask = (uint8_t)(task_mask | bit);
    if (task_mask == BOOT_TASK_ALL) {
        stage = BOOT_TRACE_ALL_TASKS;
    }
    __set_PRIMASK(primask);
}

bool BootTrace_AllTasksOnline(void)
{
    return task_mask == BOOT_TASK_ALL;
}
```

- [ ] **Step 3: Add only the C module to the application build**

Add `modules/diagnostics/boot_trace.c` to `SOURCES` and `-Imodules/diagnostics` to `CPPFLAGS` in `MSPM0G3507_LineFollowing_Car/Makefile`.

- [ ] **Step 4: Run the contract to confirm only wrapper assertions remain red**

Run: `python -m unittest tests.test_freertos_startup_diagnostics -v`

Expected: `test_all_startup_boundaries_are_named` and `test_fatal_path_is_gpio_only` PASS; wrapper/task integration tests still FAIL.

- [ ] **Step 5: Commit**

```powershell
git add -- MSPM0G3507_LineFollowing_Car/modules/diagnostics MSPM0G3507_LineFollowing_Car/Makefile
git commit -m "debug: add temporary boot trace state"
```

---

### Task 3: 记录 FreeRTOS 端口、SVC 与首次恢复

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/modules/diagnostics/freertos_startup_wrappers.S`
- Modify: `MSPM0G3507_LineFollowing_Car/FreeRTOSConfig.h`
- Modify: `freertos_kernel/Makefile`
- Modify: `MSPM0G3507_LineFollowing_Car/Makefile`
- Modify: `MSPM0G3507_LineFollowing_Car/empty.c`

**Interfaces:**
- Consumes: `BootTrace_Mark(BootTraceStage)` and official TI functions renamed only in `portasm.o`.
- Produces: public vector `SVC_Handler` and wrapper `vRestoreContextOfFirstTask`; both tail-branch to official implementations.

- [ ] **Step 1: Wire the existing scheduler trace hook**

In `FreeRTOSConfig.h` declare a no-argument bridge and add:

```c
void BootTrace_PortStart(void);
#define traceSTARTING_SCHEDULER(xIdleTaskHandles) \
    do { (void)(xIdleTaskHandles); BootTrace_PortStart(); } while (0)
```

`BootTrace_PortStart()` calls `BootTrace_Mark(BOOT_TRACE_PORT_START)`.

- [ ] **Step 2: Rename only the two official portasm symbols**

Add to `freertos_kernel/Makefile`:

```make
PORTASM_DIAG_FLAGS := -DSVC_Handler=FreeRTOS_SVC_Handler \
    -DvRestoreContextOfFirstTask=FreeRTOS_vRestoreContextOfFirstTask

$(BUILD_DIR)/portasm.o: $(PORT_DIR)/portasm.c $(CONFIG_DIR)/FreeRTOSConfig.h | $(BUILD_DIR)
	$(TIARMCLANG) -c $(CPPFLAGS) $(CFLAGS) $(PORTASM_DIAG_FLAGS) -o $@ $<
```

Do not add these flags to `port.c`; its call must continue resolving to the application wrapper.

- [ ] **Step 3: Add two register-preserving assembly trampolines**

```asm
.syntax unified
.thumb
.extern BootTrace_Mark
.extern FreeRTOS_SVC_Handler
.extern FreeRTOS_vRestoreContextOfFirstTask

.global SVC_Handler
.type SVC_Handler,%function
.thumb_func
SVC_Handler:
    push {r4, r5}
    mov  r5, lr
    push {r4, r5}
    movs r0, #5
    bl   BootTrace_Mark
    pop  {r4, r5}
    mov  lr, r5
    pop  {r4, r5}
    b    FreeRTOS_SVC_Handler

.global vRestoreContextOfFirstTask
.type vRestoreContextOfFirstTask,%function
.thumb_func
vRestoreContextOfFirstTask:
    push {r4, r5}
    mov  r5, lr
    push {r4, r5}
    movs r0, #6
    bl   BootTrace_Mark
    pop  {r4, r5}
    mov  lr, r5
    pop  {r4, r5}
    b    FreeRTOS_vRestoreContextOfFirstTask
```

- [ ] **Step 4: Add the assembly object without treating it as C**

Add an explicit object rule in the application Makefile:

```make
STARTUP_TRACE_ASM := modules/diagnostics/freertos_startup_wrappers.S
STARTUP_TRACE_OBJ := $(OBJ_DIR)/modules_diagnostics_freertos_startup_wrappers.o
OBJECTS += $(STARTUP_TRACE_OBJ)

$(STARTUP_TRACE_OBJ): $(STARTUP_TRACE_ASM) $(GEN_CONFIG) | $(OBJ_DIR)
	$(TIARMCLANG) -c $(CPPFLAGS) $(CFLAGS) -o $@ $<
```

- [ ] **Step 5: Centralize all fatal handlers in `empty.c`**

```c
void App_FreeRTOS_Assert(void) { BootTrace_Fatal(BOOT_FAULT_ASSERT); }
void HardFault_Handler(void) { BootTrace_Fatal(BOOT_FAULT_HARDFAULT); }
void Default_Handler(void) { BootTrace_Fatal(BOOT_FAULT_DEFAULT_IRQ); }
```

Mark `BOOT_TRACE_MAIN`, `BOOT_TRACE_TASKS_CREATED`, and `BOOT_TRACE_SCHED_START` at their existing boundaries. If `vTaskStartScheduler()` returns, call `BootTrace_Fatal(BOOT_FAULT_SCHED_RETURN)`.

- [ ] **Step 6: Verify the linker output**

Run clean builds, then:

```powershell
& 'D:\DevTools\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmobjdump.exe' -s -j .intvecs 'build\cli\MSPM0G3507_LineFollowing_Car\MSPM0G3507_LineFollowing_Car.out'
Select-String -Path 'build\cli\MSPM0G3507_LineFollowing_Car\MSPM0G3507_LineFollowing_Car.map' -Pattern 'SVC_Handler|FreeRTOS_SVC_Handler|vRestoreContextOfFirstTask|FreeRTOS_vRestoreContextOfFirstTask|Default_Handler|HardFault_Handler'
```

Expected: vector 11 points to the application `SVC_Handler`; both renamed official symbols and both wrappers appear exactly once.

- [ ] **Step 7: Commit**

```powershell
git add -- MSPM0G3507_LineFollowing_Car/FreeRTOSConfig.h MSPM0G3507_LineFollowing_Car/empty.c MSPM0G3507_LineFollowing_Car/Makefile MSPM0G3507_LineFollowing_Car/modules/diagnostics/freertos_startup_wrappers.S freertos_kernel/Makefile
git commit -m "debug: trace freertos first context switch"
```

---

### Task 4: 登记四任务上线并临时门控电机

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.c`

**Interfaces:**
- Consumes: `BootTrace_TaskOnline()`、`BootTrace_AllTasksOnline()`、`BootTrace_GetTaskMask()`。
- Produces: OLED `TASK MASK 0F` 记录和四任务上线后的唯一 `Motor_Safety_Arm()`。

- [ ] **Step 1: Mark each task before its first wait or peripheral operation**

Add exactly once at the start of each task body:

```c
BootTrace_TaskOnline(BOOT_TASK_SENSOR);
BootTrace_TaskOnline(BOOT_TASK_CONTROL);
BootTrace_TaskOnline(BOOT_TASK_SAFETY);
BootTrace_TaskOnline(BOOT_TASK_DISPLAY);
```

- [ ] **Step 2: Move the temporary Arm gate into the SafetyTask loop**

```c
bool motor_armed = false;

if (!motor_armed && BootTrace_AllTasksOnline() &&
    AppBoot_IsMotorConfigured() &&
    Motor_Safety_IsFaultLatched() == 0U) {
    Motor_Safety_Arm();
    motor_armed = true;
}
```

Remove the current pre-loop Arm and the one-shot `LED2_Toggle()` marker. Keep the single production `Motor_Safety_Arm()` call inside SafetyTask.

- [ ] **Step 3: Add a bounded task-mask log formatter**

```c
bool RuntimeLog_PushTaskMask(uint32_t now_ms, uint8_t mask)
{
    static const char hex[] = "0123456789ABCDEF";
    char payload[] = "TASK MASK 00";
    payload[10] = hex[(mask >> 4) & 0x0F];
    payload[11] = hex[mask & 0x0F];
    return RuntimeLog_Push(now_ms, payload);
}
```

DisplayTask logs a changed mask and keeps the existing `SAFETY TASK`、`SENSOR FRAME`、`CONTROL REQ` runtime events.

- [ ] **Step 4: Route stack overflow to the GPIO-only fatal code**

Replace the current infinite stack-overflow hook body with:

```c
(void)task;
(void)task_name;
BootTrace_Fatal(BOOT_FAULT_STACK_OVERFLOW);
```

- [ ] **Step 5: Verify GREEN**

Run:

```powershell
python -m unittest tests.test_freertos_startup_diagnostics tests.test_freertos_contract tests.test_freertos_schedule -v
```

Expected: all tests PASS.

- [ ] **Step 6: Commit**

```powershell
git add -- MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.c MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.h
git commit -m "debug: report freertos task startup mask"
```

---

### Task 5: 清缓存、构建并交付现场诊断固件

**Files:**
- Generated: `dist/firmware/MSPM0G3507_LineFollowing_Car.txt`
- Verify: `build/cli/MSPM0G3507_LineFollowing_Car/MSPM0G3507_LineFollowing_Car.map`

**Interfaces:**
- Consumes: Tasks 1-4.
- Produces: 可由 UniFlash 烧录的诊断 TI-TXT 及明确判读表。

- [ ] **Step 1: Clean and rebuild both kernel and application**

```powershell
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C freertos_kernel clean all
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C MSPM0G3507_LineFollowing_Car rebuild
```

Expected: exit code 0 and a new TI-TXT timestamp.

- [ ] **Step 2: Run the full host suite**

Run: `python -m unittest discover -s tests -v`

Expected: all tests PASS.

- [ ] **Step 3: Record artifact identity**

```powershell
Get-Item 'dist\firmware\MSPM0G3507_LineFollowing_Car.txt' | Select-Object FullName,Length,LastWriteTime
Get-FileHash 'dist\firmware\MSPM0G3507_LineFollowing_Car.txt' -Algorithm SHA256
```

- [ ] **Step 4: Field interpretation checkpoint**

- OLED `SCHED START`, LEDs `00`: scheduler trace hook not reached or immediate assert before stage 04.
- OLED `SCHED START`, LEDs `01`: entered FreeRTOS port but SVC not entered.
- OLED `SCHED START`, LEDs `10`: entered SVC but first context restore not called.
- OLED `SCHED START`, LEDs `11`: restore began but first task C entry not reached.
- D1 ON with D2 repeating 1-5 pulses: `E1-E5` fatal code.
- D1 OFF and D2 250 ms heartbeat, OLED task mask `0F`: all tasks running; continue into motor/control diagnostics.

Do not claim the motor issue fixed until the field result reaches task mask `0F` and a nonzero motor request is observed.

---

### Task 6: 根因修复后撤销临时诊断

**Files:**
- Delete: `MSPM0G3507_LineFollowing_Car/modules/diagnostics/boot_trace.c`
- Delete: `MSPM0G3507_LineFollowing_Car/modules/diagnostics/boot_trace.h`
- Delete: `MSPM0G3507_LineFollowing_Car/modules/diagnostics/freertos_startup_wrappers.S`
- Modify: `MSPM0G3507_LineFollowing_Car/FreeRTOSConfig.h`
- Modify: `freertos_kernel/Makefile`
- Modify: `MSPM0G3507_LineFollowing_Car/Makefile`
- Modify: `MSPM0G3507_LineFollowing_Car/empty.c`
- Modify: `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.c`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.h`
- Delete: `tests/test_freertos_startup_diagnostics.py`

**Interfaces:**
- Consumes: 已由现场阶段码确认并验证的根因修复。
- Produces: 只保留根因修复、原 D2 250 ms 心跳和正常任务周期的发布固件。

- [ ] **Step 1: Remove only the temporary diagnostics listed above**

Restore direct official `SVC_Handler`/`vRestoreContextOfFirstTask` symbols, remove `traceSTARTING_SCHEDULER`, restore the SafetyTask normal Arm location confirmed by the root-cause fix, and remove task-mask OLED output.

- [ ] **Step 2: Clean rebuild and run the full suite**

Use the exact Task 5 commands. Expected: no diagnostic symbols in the map, all tests PASS, new TI-TXT generated.

- [ ] **Step 3: Commit the cleanup separately**

```powershell
git add -A -- MSPM0G3507_LineFollowing_Car/modules/diagnostics MSPM0G3507_LineFollowing_Car/FreeRTOSConfig.h MSPM0G3507_LineFollowing_Car/Makefile MSPM0G3507_LineFollowing_Car/empty.c MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.c MSPM0G3507_LineFollowing_Car/modules/display/runtime_log.h freertos_kernel/Makefile tests/test_freertos_startup_diagnostics.py
git commit -m "chore: remove temporary freertos startup tracing"
```
