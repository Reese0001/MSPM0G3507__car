# FreeRTOS OLED Lookup Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current polling/PID-style line controller with a FreeRTOS-based, 15-position lookup controller, recover from corners without a permanent D1 latch, and show diagnostics on a PA10/PA11 SSD1306 OLED.

**Architecture:** Keep hardware safety in the existing 1 ms timer ISR, while four statically allocated FreeRTOS tasks handle safety, 2 ms sensing, lookup control, and low-priority display. The eight-channel line sensor remains the sole steering source; MPU6050 only limits excessive corner yaw. Motor output continues through the existing UART protocol and is rate-limited to one latest command every 5 ms.

**Tech Stack:** MSPM0G3507, TI Arm Clang 4.0.4, MSPM0 SDK 2.10.00.04 FreeRTOS 11.2.0 CM0+ port, DriverLib, SSD1306 software I2C, Python `unittest`, host C harnesses.

## Global Constraints

- Work directly on `main`; after every completed task, commit and immediately push `origin main`.
- Preserve motor UART PB6/PB7 and 1 ms `Motor_Safety_Tick1ms()`.
- Preserve the 0 to 30% soft-start ramp and command limit ±450; never emit an immediate 100% command.
- PA10 is OLED SCL, PA11 is OLED SDA; UART0 debug output is disabled.
- PA12/PA13 remain MPU6050 software I2C.
- PA15/PA16/PA17/PA18 remain line sensor AD0/AD1/AD2/OUT.
- Sensor 1 is leftmost and sensor 8 is rightmost when looking forward.
- The course has one continuous black line; do not implement parallel-line selection.
- Normal control accepts only one active sensor or two adjacent active sensors.
- Use static FreeRTOS tasks and static mailboxes only; no application heap allocation.
- C source encoding must remain unchanged; new documentation is UTF-8.

---

## File Structure

- `MSPM0G3507_LineFollowing_Car/FreeRTOSConfig.h`: application-specific 1 kHz, static-only kernel configuration.
- `MSPM0G3507_LineFollowing_Car/application/freertos/app_tasks.[ch]`: create and run four static tasks.
- `MSPM0G3507_LineFollowing_Car/application/freertos/app_mailbox.[ch]`: latest-value snapshots guarded by short critical sections.
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_position.[ch]`: decode and debounce the 15 legal positions.
- `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_lookup_control.[ch]`: convert stable position to bounded left/right motor commands.
- `MSPM0G3507_LineFollowing_Car/application/line_recovery.[ch]`: bounded blind-forward and same-direction search without permanent control fault.
- `MSPM0G3507_LineFollowing_Car/modules/display/ssd1306.[ch]`: minimal 128x64 text/bitmap driver.
- `MSPM0G3507_LineFollowing_Car/bsp/bsp_oled_i2c.[ch]`: PA10/PA11 open-drain software I2C.
- `MSPM0G3507_LineFollowing_Car/application/diagnostics/dashboard.[ch]`: render only the fixed diagnostic page.
- `MSPM0G3507_LineFollowing_Car/application/config/line_lookup_config.h`: all 15 table entries, speed limits, debounce, and recovery constants.
- `tests/*_harness.c`, `tests/test_*.py`: host behavior and source-contract checks.

---

### Task 1: Add a motor-safe FreeRTOS build skeleton

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/FreeRTOSConfig.h`
- Create: `MSPM0G3507_LineFollowing_Car/application/freertos/app_tasks.h`
- Create: `MSPM0G3507_LineFollowing_Car/application/freertos/app_tasks.c`
- Modify: `MSPM0G3507_LineFollowing_Car/empty.c`
- Modify: `MSPM0G3507_LineFollowing_Car/empty.syscfg`
- Modify: `MSPM0G3507_LineFollowing_Car/application/app_main.c`
- Modify: `MSPM0G3507_LineFollowing_Car/bsp/delay.c`
- Modify: `MSPM0G3507_LineFollowing_Car/.project`
- Modify: `MSPM0G3507_LineFollowing_Car/.cproject`
- Test: `tests/test_freertos_contract.py`

**Interfaces:**
- Consumes: `SYSCFG_DL_init()`, `Motor_Safety_Init()`, `Motor_Safety_Disarm()`, `Timer_Init()`.
- Produces: `bool AppTasks_Create(void)` and static task handles for later tasks.

- [x] **Step 1: Write the failing source-contract test**

```python
from pathlib import Path
import unittest

ROOT = Path(__file__).parents[1] / "MSPM0G3507_LineFollowing_Car"

class FreeRtosContract(unittest.TestCase):
    def test_static_only_1khz_kernel_and_safe_start(self):
        config = (ROOT / "FreeRTOSConfig.h").read_text(encoding="utf-8")
        tasks = (ROOT / "application/freertos/app_tasks.c").read_text(encoding="utf-8")
        main = (ROOT / "empty.c").read_text(encoding="utf-8")
        self.assertIn("#define configTICK_RATE_HZ 1000", config)
        self.assertIn("#define configSUPPORT_STATIC_ALLOCATION 1", config)
        self.assertIn("#define configSUPPORT_DYNAMIC_ALLOCATION 0", config)
        self.assertIn("xTaskCreateStatic", tasks)
        self.assertIn("Motor_Safety_Disarm()", tasks)
        self.assertIn("vTaskStartScheduler()", main)
```

- [x] **Step 2: Run the test and verify RED**

Run: `python -m unittest tests.test_freertos_contract -v`

Expected: FAIL because `FreeRTOSConfig.h` and `application/freertos/app_tasks.c` do not exist.

- [x] **Step 3: Add the minimal static kernel configuration**

Use the SDK file `C:\ti\mspm0_sdk_2_10_00_04\kernel\freertos\builds\LP_MSPM0G3507\release\FreeRTOSConfig.h` as the port reference, with these application settings:

```c
#define configCPU_CLOCK_HZ 80000000UL
#define configTICK_RATE_HZ 1000
#define configMAX_PRIORITIES 5
#define configMINIMAL_STACK_SIZE 96U
#define configUSE_PREEMPTION 1
#define configUSE_TIME_SLICING 0
#define configUSE_TIMERS 0
#define configUSE_MUTEXES 0
#define configUSE_TASK_NOTIFICATIONS 1
#define configSUPPORT_STATIC_ALLOCATION 1
#define configSUPPORT_DYNAMIC_ALLOCATION 0
#define configCHECK_FOR_STACK_OVERFLOW 2
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
```

`AppTasks_Create()` initially creates one static bootstrap task whose body calls `Motor_Safety_Disarm()` and blocks forever:

```c
static void BootstrapTask(void *argument)
{
    (void)argument;
    Motor_Safety_Disarm();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
```

Provide the static idle memory required by `configSUPPORT_STATIC_ALLOCATION`:

```c
void vApplicationGetIdleTaskMemory(StaticTask_t **tcb,
                                   StackType_t **stack,
                                   uint32_t *depth)
{
    static StaticTask_t idle_tcb;
    static StackType_t idle_stack[configMINIMAL_STACK_SIZE];
    *tcb = &idle_tcb;
    *stack = idle_stack;
    *depth = configMINIMAL_STACK_SIZE;
}
```

Change `empty.c` to start the scheduler only after successful task creation:

```c
int main(void)
{
    SYSCFG_DL_init();
    App_Main_Init();
    if (!AppTasks_Create()) {
        Motor_Safety_Disarm();
        for (;;) {}
    }
    vTaskStartScheduler();
    Motor_Safety_Disarm();
    for (;;) {}
}
```

Remove the SysConfig `SYSTICK` instance because the FreeRTOS port owns SysTick. Change `delay_us()` to compare `BSP_Time_GetUs()` from TIMG12, so no application code reads `SysTick->VAL`.

Import the official dependency project once from:

```text
C:\ti\mspm0_sdk_2_10_00_04\kernel\freertos\builds\LP_MSPM0G3507\release\ticlang\freertos_builds_LP_MSPM0G3507_release_ticlang.projectspec
```

Add `freertos_builds_LP_MSPM0G3507_release_ticlang` to `.project` project references. Add these exact SDK include directories to `.cproject`:

```text
${COM_TI_MSPM0_SDK_INSTALL_DIR}/kernel/freertos/Source/include
${COM_TI_MSPM0_SDK_INSTALL_DIR}/kernel/freertos/Source/portable/TI_ARM_CLANG/ARM_CM0
${WORKSPACE_LOC}/freertos_builds_LP_MSPM0G3507_release_ticlang
```

Add the dependency library search path and library:

```text
${WORKSPACE_LOC}/freertos_builds_LP_MSPM0G3507_release_ticlang/Debug
freertos_builds_LP_MSPM0G3507_release_ticlang.lib
```

Do not copy or fork the kernel.

- [x] **Step 4: Verify host contract and TI build**

Run:

```powershell
python -m unittest tests.test_freertos_contract -v
gmake -C MSPM0G3507_LineFollowing_Car/Debug -j4 all
```

Expected: contract PASS; TI link succeeds with no undefined `vTaskStartScheduler`, PendSV, SVC, or SysTick symbols. On a USB-only bench flash, D2 may heartbeat but motors remain stopped.

- [x] **Step 5: Commit and push**

```powershell
git add MSPM0G3507_LineFollowing_Car/FreeRTOSConfig.h MSPM0G3507_LineFollowing_Car/application/freertos MSPM0G3507_LineFollowing_Car/empty.c MSPM0G3507_LineFollowing_Car/empty.syscfg MSPM0G3507_LineFollowing_Car/application/app_main.c MSPM0G3507_LineFollowing_Car/bsp/delay.c MSPM0G3507_LineFollowing_Car/.project MSPM0G3507_LineFollowing_Car/.cproject tests/test_freertos_contract.py
git commit -m "feat: add motor-safe FreeRTOS skeleton"
git push origin main
```

---

### Task 2: Decode and debounce the 15 legal line positions

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_position.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_position.c`
- Create: `MSPM0G3507_LineFollowing_Car/application/config/line_lookup_config.h`
- Create: `tests/line_position_harness.c`
- Create: `tests/test_line_position.py`

**Interfaces:**
- Consumes: raw `uint8_t black_bits`, bit 0 = sensor 1 leftmost.
- Produces:

```c
typedef enum {
    LINE_PATTERN_LOST = 0,
    LINE_PATTERN_POSITION,
    LINE_PATTERN_WIDE,
    LINE_PATTERN_NOISE
} LinePatternType;

typedef struct {
    LinePatternType type;
    int8_t stable_position;
    int8_t candidate_position;
    uint8_t candidate_frames;
    uint8_t black_bits;
} LinePositionResult;

void LinePosition_Reset(void);
LinePositionResult LinePosition_Update(uint8_t black_bits);
```

- [x] **Step 1: Write the failing C harness**

The harness must assert:

```c
CHECK(LinePosition_Update(0x01U).stable_position == -7);
CHECK(LinePosition_Update(0x03U).stable_position == -6);
CHECK(LinePosition_Update(0x18U).stable_position == 0);
CHECK(LinePosition_Update(0x80U).stable_position == 7);
CHECK(LinePosition_Update(0x07U).type == LINE_PATTERN_WIDE);
CHECK(LinePosition_Update(0x11U).type == LINE_PATTERN_NOISE);
CHECK(LinePosition_Update(0x00U).type == LINE_PATTERN_LOST);
```

It must also prove that `-7 -> +7` is rejected on the first frame and accepted on the second identical frame, while `-7 -> -6` is accepted immediately.

- [x] **Step 2: Run the harness and verify RED**

Run: `python -m unittest tests.test_line_position -v`

Expected: FAIL because `line_position.c` is missing.

- [x] **Step 3: Implement the fixed decoder**

Use one 256-entry signed lookup initialized only for the 15 legal patterns:

```c
static const int8_t position_by_bits[256] = {
    [0x01] = -7, [0x03] = -6, [0x02] = -5, [0x06] = -4,
    [0x04] = -3, [0x0C] = -2, [0x08] = -1, [0x18] = 0,
    [0x10] = 1,  [0x30] = 2,  [0x20] = 3,  [0x60] = 4,
    [0x40] = 5,  [0xC0] = 6,  [0x80] = 7
};
```

Because zero-initialized entries collide with the valid center value, pair it with an explicit legal-pattern predicate. A contiguous run of three or more bits is `LINE_PATTERN_WIDE`; separated runs are `LINE_PATTERN_NOISE`.

Acceptance rule:

```c
if (abs(candidate - stable) <= 1) accept_now();
else if (candidate == previous_candidate && candidate_frames >= 2U) accept_now();
else hold_stable();
```

- [x] **Step 4: Run focused and full tests**

Run:

```powershell
python -m unittest tests.test_line_position -v
python -m unittest discover -s tests -v
```

Expected: new harness PASS and existing suite has zero failures.

- [x] **Step 5: Commit and push**

```powershell
git add MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_position.* MSPM0G3507_LineFollowing_Car/application/config/line_lookup_config.h tests/line_position_harness.c tests/test_line_position.py
git commit -m "feat: decode stable 15-position line states"
git push origin main
```

---

### Task 3: Add the open-loop speed and differential lookup table

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_lookup_control.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_lookup_control.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/config/line_lookup_config.h`
- Create: `tests/line_lookup_control_harness.c`
- Create: `tests/test_line_lookup_control.py`

**Interfaces:**
- Consumes: stable position `-7..7`, module state, optional fresh yaw rate.
- Produces:

```c
typedef struct {
    int16_t left;
    int16_t right;
    int16_t base;
    int16_t diff;
    bool valid;
} LineLookupCommand;

LineLookupCommand LineLookupControl_Step(int8_t position,
                                        float yaw_rate_dps,
                                        bool yaw_fresh);
```

- [x] **Step 1: Write the failing table tests**

Assert symmetry and bounds:

```c
center = LineLookupControl_Step(0, 0.0f, false);
left_edge = LineLookupControl_Step(-7, 0.0f, false);
right_edge = LineLookupControl_Step(7, 0.0f, false);
CHECK(center.left == center.right);
CHECK(center.base >= 400 && center.base <= 430);
CHECK(left_edge.left == right_edge.right);
CHECK(left_edge.right == right_edge.left);
CHECK(abs(left_edge.left) <= 450 && abs(left_edge.right) <= 450);
```

Also assert that high correct-direction yaw reduces the magnitude of the edge command but never reverses the requested turn.
Assert steering direction explicitly: at `-7`, left wheel is slower than right; at `+7`, right wheel is slower than left.

- [x] **Step 2: Run and verify RED**

Run: `python -m unittest tests.test_line_lookup_control -v`

Expected: FAIL because the lookup controller is missing.

- [x] **Step 3: Implement one symmetric table**

Store only the eight non-negative entries and mirror negative positions:

```c
typedef struct { int16_t base; int16_t diff; } LineLookupEntry;

static const LineLookupEntry table[8] = {
    {420, 0}, {400, 30}, {370, 65}, {330, 100},
    {285, 135}, {235, 165}, {175, 195}, {120, 220}
};
```

The differential sign is opposite to the position sign because negative position means the line is left:

```c
diff = position < 0 ? magnitude :
       position > 0 ? (int16_t)-magnitude : 0;
```

Apply MPU only as a corner limiter:

```c
if (yaw_fresh && abs(position) >= 5 &&
    fabsf(yaw_rate_dps) >= LINE_LOOKUP_HIGH_YAW_DPS) {
    diff = diff * 3 / 4;
}
```

Calculate `left = clamp(base - diff, -450, 450)` and `right = clamp(base + diff, -450, 450)`. Do not add PID state, derivative filters, or integral terms.

- [x] **Step 4: Run focused/full tests**

Run:

```powershell
python -m unittest tests.test_line_lookup_control -v
python -m unittest discover -s tests -v
```

Expected: all tests PASS.

- [x] **Step 5: Commit and push**

```powershell
git add MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_lookup_control.* MSPM0G3507_LineFollowing_Car/application/config/line_lookup_config.h tests/line_lookup_control_harness.c tests/test_line_lookup_control.py
git commit -m "feat: add open-loop line speed table"
git push origin main
```

---

### Task 4: Make corner and lost-line recovery non-latching

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/application/line_recovery.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/line_recovery.h`
- Modify: `MSPM0G3507_LineFollowing_Car/application/corner_maneuver.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/app_scheduler.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/config/line_recovery_config.h`
- Test: `tests/line_recovery_harness.c`
- Test: `tests/corner_maneuver_harness.c`
- Modify: `tests/test_app_scheduler.py`

**Interfaces:**
- Consumes: last stable position and new `LinePositionResult`.
- Produces: `LINE_RECOVERY_STOPPED` for an exhausted search; never reports a permanent control fault for ordinary line loss.

- [x] **Step 1: Change tests first**

Replace timeout expectations:

```c
CHECK(LineRecovery_GetState() == LINE_RECOVERY_STOPPED);
CHECK(request.left_speed == 0);
CHECK(request.right_speed == 0);
CHECK(!request.valid);
```

Add a regression sequence: stable `-5`, lost frames, forward-left blind search, same-direction rotate, then two stable `-2` frames; expect recovery to enter align and return to follow without D1.

Add a scheduler assertion that only `Motor_Safety_IsFaultLatched()` causes `control_fault_latched`.

- [x] **Step 2: Run and verify RED**

Run:

```powershell
python -m unittest tests.test_line_recovery tests.test_corner_maneuver tests.test_app_scheduler -v
```

Expected: FAIL on old permanent fault states.

- [x] **Step 3: Implement recoverable timeout semantics**

Add:

```c
typedef enum {
    LINE_RECOVERY_FOLLOW = 0,
    LINE_RECOVERY_LOSS_CONFIRM,
    LINE_RECOVERY_FORWARD_SEARCH,
    LINE_RECOVERY_ROTATE_SEARCH,
    LINE_RECOVERY_ALIGN,
    LINE_RECOVERY_STOPPED
} LineRecoveryState;
```

Use the last stable sign for both blind-forward and rotation. No reverse-search state is allowed. Corner seek timeout hands ownership to recovery:

```c
if (seek_expired) {
    CornerManeuver_Reset();
    out->owns_motion = false;
    out->fault = false;
    return true;
}
```

Remove permanent latching for `CornerManeuver`/`LineRecovery` timeouts from `AppScheduler_RunLineControl`. Keep D1 latching for motor UART and task-heartbeat failures only.

- [x] **Step 4: Run focused/full tests**

Run:

```powershell
python -m unittest tests.test_line_recovery tests.test_corner_maneuver tests.test_app_scheduler -v
python -m unittest discover -s tests -v
```

Expected: all tests PASS; old tests expecting ordinary search to become `FAULT` are updated to `STOPPED`.

- [x] **Step 5: Commit and push**

```powershell
git add MSPM0G3507_LineFollowing_Car/application tests/line_recovery_harness.c tests/corner_maneuver_harness.c tests/test_app_scheduler.py
git commit -m "fix: keep line recovery out of permanent fault"
git push origin main
```

---

### Task 5: Run sensing, control, safety, and motor output as static tasks

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/application/freertos/app_mailbox.h`
- Create: `MSPM0G3507_LineFollowing_Car/application/freertos/app_mailbox.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/freertos/app_tasks.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/app_main.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/app_scheduler.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/app_scheduler.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_scanner.c`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_scanner.h`
- Create: `tests/test_freertos_schedule.py`

**Interfaces:**
- `SensorTask`: publishes `LinePositionResult` every 2 ms and MPU snapshot every 10 ms.
- `ControlTask`: wakes from direct notification and publishes latest `MotionRequest`.
- `SafetyTask`: runs every 1 ms, applies request and checks heartbeats.
- `AppMailbox_*`: copies fixed structs inside `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()`.

- [x] **Step 1: Write failing schedule-contract tests**

Assert:

```python
self.assertIn("pdMS_TO_TICKS(2U)", tasks)
self.assertIn("pdMS_TO_TICKS(1U)", tasks)
self.assertIn("ulTaskNotifyTake", tasks)
self.assertIn("xTaskNotifyGive", tasks)
self.assertIn("MOTOR_UART_MIN_PERIOD_MS (5U)", config)
self.assertNotIn("AppScheduler_Run(now_ms)", main)
```

- [x] **Step 2: Run and verify RED**

Run: `python -m unittest tests.test_freertos_schedule -v`

Expected: FAIL because cooperative polling is still active.

- [x] **Step 3: Implement the four task loops**

Use fixed priorities so freshly published sensor data immediately preempts the sensor task:

```c
#define APP_TASK_PRIORITY_DISPLAY 1U
#define APP_TASK_PRIORITY_SENSOR  2U
#define APP_TASK_PRIORITY_CONTROL 3U
#define APP_TASK_PRIORITY_SAFETY  4U
```

Use `vTaskDelayUntil()` in sensor and safety. `ControlTask` blocks on `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)`. Publish only the newest request. Safety sends to the motor at most once every 5 ms unless an immediate zero command is required.

Add a bounded full-frame scanner:

```c
bool LineScanner_ReadFrame(uint32_t now_ms, LineSensorSnapshot *out);
```

It selects each of the eight channels, waits `LINE_MUX_SETTLE_US` using `BSP_Time_GetUs()`, samples OUT, and returns within 600 us. Every fifth SensorTask cycle, complete the existing nonblocking MPU transaction in a bounded service loop:

```c
while (BSP_I2C_GetStatus() == BSP_I2C_STATUS_BUSY &&
       (uint32_t)(BSP_Time_GetUs() - started_us) < 1000U) {
    BSP_I2C_Service(BSP_Time_GetUs());
}
```

The loop is bounded to 1 ms. SafetyTask can preempt it; after the line snapshot notification, higher-priority ControlTask preempts SensorTask. This preserves the 5 us software-I2C state machine without creating another timer or driver.

The hardware TIMER_0 ISR continues to call `Motor_Safety_Tick1ms()` and must not call any FreeRTOS API.

- [x] **Step 4: Verify tests and build**

Run:

```powershell
python -m unittest tests.test_freertos_schedule tests.test_motor_safety_contract tests.test_application_schedule -v
python -m unittest discover -s tests -v
gmake -C MSPM0G3507_LineFollowing_Car/Debug -j4 all
```

Expected: all host tests PASS; TI build succeeds; map file contains four static task stacks and no application heap symbol.

- [x] **Step 5: Commit and push**

```powershell
git add MSPM0G3507_LineFollowing_Car/application/freertos MSPM0G3507_LineFollowing_Car/application/app_main.c MSPM0G3507_LineFollowing_Car/application/app_scheduler.* MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_scanner.* tests/test_freertos_schedule.py
git commit -m "feat: schedule line control with FreeRTOS"
git push origin main
```

---

### Task 6: Add the PA10/PA11 SSD1306 diagnostic display

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/empty.syscfg`
- Create: `MSPM0G3507_LineFollowing_Car/bsp/bsp_oled_i2c.h`
- Create: `MSPM0G3507_LineFollowing_Car/bsp/bsp_oled_i2c.c`
- Create: `MSPM0G3507_LineFollowing_Car/modules/display/ssd1306.h`
- Create: `MSPM0G3507_LineFollowing_Car/modules/display/ssd1306.c`
- Create: `MSPM0G3507_LineFollowing_Car/application/diagnostics/dashboard.h`
- Create: `MSPM0G3507_LineFollowing_Car/application/diagnostics/dashboard.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/freertos/app_tasks.c`
- Create: `tests/test_oled_contract.py`

**Interfaces:**
- Consumes: diagnostic mailbox snapshot.
- Produces: `bool Ssd1306_Init(void)`, `bool Ssd1306_WritePage(uint8_t page, const uint8_t data[128])`, and `void Dashboard_Render(const AppDiagnostics *data)`.

- [x] **Step 1: Write failing pin/address/display tests**

Assert PA10/PA11 are GPIO named `OLED_I2C_SCL`/`OLED_I2C_SDA`, UART0 is absent, address is `0x3C`, display period is 200 ms, and OLED failure does not call `Motor_Safety_Disarm()`.

- [x] **Step 2: Run and verify RED**

Run: `python -m unittest tests.test_oled_contract -v`

Expected: FAIL because the OLED modules are missing and PA10/PA11 are still UART0.

- [x] **Step 3: Implement the minimal display**

Change SysConfig only through `empty.syscfg`; regenerate `ti_msp_dl_config.*`. Configure PA10/PA11 released-high GPIO. Implement open-drain behavior by switching output-low and input/released-high.

SSD1306 requirements:

```c
#define SSD1306_ADDRESS_7BIT 0x3CU
#define SSD1306_WIDTH 128U
#define SSD1306_PAGES 8U
```

Keep one 1024-byte framebuffer plus an 8-bit dirty-page mask. DisplayTask wakes every 200 ms and writes only dirty pages. Render fixed fields: run state, eight bits, stable/candidate position, left/right command, MPU state, recovery state, and fault code. Do not add menus, animation, proportional fonts, or a second screen.

- [x] **Step 4: Verify tests, SysConfig, and build**

Run:

```powershell
python -m unittest tests.test_oled_contract tests.test_sysconfig_contract -v
python -m unittest discover -s tests -v
gmake -C MSPM0G3507_LineFollowing_Car/Debug -j4 all
```

Expected: all tests PASS; generated config contains PA10/PA11 OLED GPIO and no UART0; TI build succeeds.

- [x] **Step 5: USB-only hardware check**

With 12.6 V disconnected:

1. Connect OLED VCC to 3.3 V, GND to GND, SCL to PA10, SDA to PA11.
2. Flash firmware.
3. Confirm the dashboard appears and D2 continues blinking.
4. Disconnect OLED and confirm line/motor tasks continue without D1.

- [x] **Step 6: Commit and push**

```powershell
git add MSPM0G3507_LineFollowing_Car/empty.syscfg MSPM0G3507_LineFollowing_Car/bsp/bsp_oled_i2c.* MSPM0G3507_LineFollowing_Car/modules/display MSPM0G3507_LineFollowing_Car/application/diagnostics MSPM0G3507_LineFollowing_Car/application/freertos/app_tasks.c tests/test_oled_contract.py
git commit -m "feat: show car diagnostics on SSD1306"
git push origin main
```

---

### Task 7: Add MPU corner limiting and actionable fault diagnostics

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_lookup_control.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/freertos/app_tasks.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/diagnostics/dashboard.c`
- Modify: `MSPM0G3507_LineFollowing_Car/application/config/line_lookup_config.h`
- Test: `tests/line_lookup_control_harness.c`
- Create: `tests/test_fault_diagnostics.py`

**Interfaces:**
- Consumes: fresh MPU yaw rate and per-task heartbeat counters.
- Produces: non-latching `C-SEARCH`, `L-LOST`, `OLED-I2C`; latching `M-UART`, `CTRL-HB`, `SENS-HB`.

- [x] **Step 1: Add failing tests**

Test that high same-direction yaw reduces `diff`, opposite yaw does not reverse direction, stale MPU caps speed at 280, and display-only failure never stops motors. Test that control/sensor heartbeat expiry does stop and latch D1.

- [x] **Step 2: Run and verify RED**

Run:

```powershell
python -m unittest tests.test_line_lookup_control tests.test_fault_diagnostics -v
```

Expected: FAIL for missing diagnostic classification and degraded speed cap.

- [x] **Step 3: Implement minimal supervision**

Use constants:

```c
#define LINE_LOOKUP_HIGH_YAW_DPS 95.0f
#define LINE_LOOKUP_IMU_DEGRADED_LIMIT 280
#define APP_CONTROL_HEARTBEAT_TIMEOUT_MS 30U
#define APP_SENSOR_HEARTBEAT_TIMEOUT_MS 20U
```

Do not create a yaw PID. Limit only the lookup result and publish one enum fault code to the dashboard.

- [x] **Step 4: Run full verification**

Run:

```powershell
python -m unittest discover -s tests -v
gmake -C MSPM0G3507_LineFollowing_Car/Debug -j4 all
```

Expected: zero host-test failures and successful TI link.

- [x] **Step 5: Commit and push**

```powershell
git add MSPM0G3507_LineFollowing_Car/modules/line_tracking/line_lookup_control.c MSPM0G3507_LineFollowing_Car/application/freertos/app_tasks.c MSPM0G3507_LineFollowing_Car/application/diagnostics/dashboard.c MSPM0G3507_LineFollowing_Car/application/config/line_lookup_config.h tests/line_lookup_control_harness.c tests/test_fault_diagnostics.py
git commit -m "feat: supervise lookup turns with MPU6050"
git push origin main
```

---

### Task 8: Build burnable firmware and perform staged car acceptance

**Files:**
- Modify: `docs/hardware/final-wiring.md`
- Modify: `docs/verification/sensor-platform-test-record.md`
- Generate locally only: `firmware/MSPM0G3507_LineFollowing_Car.hex`
- Generate locally only: `firmware/MSPM0G3507_LineFollowing_Car.txt`

**Interfaces:**
- Consumes: verified TI `.out`.
- Produces: matching Intel HEX and TI-TXT images for UniFlash; firmware remains untracked.

- [x] **Step 1: Run software verification**

```powershell
python -m unittest discover -s tests -v
gmake -C MSPM0G3507_LineFollowing_Car/Debug -j4 clean all
git diff --check
```

Expected: zero test failures, successful clean TI build, no whitespace errors.

- [x] **Step 2: Generate and compare UniFlash files**

Use TI Arm Objcopy/Hex utility from TI Arm Clang 4.0.4 to generate both formats. Parse both outputs and assert identical address/data bytes. Record SHA-256 hashes; do not add `firmware/` to git.

- [ ] **Step 3: Perform the safety bench sequence**

1. USB only, 12.6 V disconnected: verify OLED, D2 heartbeat, line bits, MPU state.
2. Wheels suspended, 12.6 V connected: verify 0→30% soft-start and immediate zero command.
3. Low-speed floor test: verify all 15 positions steer in the expected direction.
4. Lost-line test: verify blind travel follows the last stable direction and recovery does not latch D1.
5. Sharp/90-degree turn test: verify two stable reacquisition frames prevent post-turn oscillation.
6. Full route: raise only the center/near-center `base` table values if straight sections are still slow; never exceed 450.

- [ ] **Step 4: Record exact accepted values**

Update the verification record with final 15 table entries, yaw threshold, observed maximum task latency, firmware hashes, and pass/fail for each bench stage.

- [x] **Step 5: Commit and push documentation**

```powershell
git add docs/hardware/final-wiring.md docs/verification/sensor-platform-test-record.md
git commit -m "docs: record FreeRTOS car acceptance"
git push origin main
```
