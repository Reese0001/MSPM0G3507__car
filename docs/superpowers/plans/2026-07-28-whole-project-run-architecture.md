# Whole Project Run Architecture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the firmware application layer so RESET/K1, line control, safety arbitration, motor output, and OLED observation are separated and the car can run through a clear path.

**Architecture:** Keep the current single CCS project and existing `app/`, `modules/`, and `shared/` roots. First split the 525-line `app/tasks/app_tasks.c` into focused app modules while preserving the current FreeRTOS task model and motor safety layer. Later phases clean line/motor boundaries and optional modules without changing hardware protocols.

**Tech Stack:** MSPM0G3507 firmware in C, FreeRTOS static tasks, TI Arm Clang 4.0.4, MSPM0 SDK 2.10.00.04, Python `unittest`, TI-TXT/UniFlash.

## Global Constraints

- Keep one CCS project: `MSPM0G3507_LineFollowing_Car`.
- Keep UniFlash output at `dist/firmware/MSPM0G3507_LineFollowing_Car.txt`.
- Clean generated object/build caches before every firmware build.
- Keep `Set_Motor(5)` for the confirmed L-type 520 motor.
- Do not change the motor UART protocol unless a separate motor checklist is reviewed.
- No direct PWM and no 100% motor command.
- All motor motion must go through `Motor_Safety_RequestSpeed`.
- OLED is a debug log only; OLED failure must not stop the motor.
- X1 is the right sensor side and X8 is the left sensor side.

---

## File Structure

- Create `MSPM0G3507_LineFollowing_Car/app/run/run_controller.h`
- Create `MSPM0G3507_LineFollowing_Car/app/run/run_controller.c`
- Create `MSPM0G3507_LineFollowing_Car/app/line/line_motion.h`
- Create `MSPM0G3507_LineFollowing_Car/app/line/line_motion.c`
- Create `MSPM0G3507_LineFollowing_Car/app/log/runtime_observer.h`
- Create `MSPM0G3507_LineFollowing_Car/app/log/runtime_observer.c`
- Modify `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c`
- Modify `MSPM0G3507_LineFollowing_Car/Makefile`
- Add tests:
  - `tests/test_app_task_decomposition.py`
  - `tests/test_run_controller_contract.py`
  - `tests/test_line_motion_contract.py`
  - `tests/test_runtime_observer_contract.py`

`app/tasks/app_tasks.c` remains the only file that creates FreeRTOS tasks. It may call app modules, but must not contain bring-up policy, line-to-motion algorithm details, or OLED event formatting.

---

### Task 1: Decomposition Contract

**Files:**
- Create: `tests/test_app_task_decomposition.py`

**Interfaces:**
- Consumes: current source tree.
- Produces: failing tests that require the app runtime split.

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"
TASKS = ROOT / "app/tasks/app_tasks.c"


class AppTaskDecompositionContract(unittest.TestCase):
    def setUp(self):
        self.tasks = TASKS.read_text(encoding="utf-8")

    def test_app_tasks_is_only_a_task_shell(self):
        line_count = len(self.tasks.splitlines())
        self.assertLessEqual(line_count, 260)
        self.assertNotIn("BuildBringupRunRequest", self.tasks)
        self.assertNotIn("BuildMotionRequest", self.tasks)
        self.assertNotIn("RuntimeLog_Push(now_ms", self.tasks)

    def test_new_app_modules_exist(self):
        for path in (
            "app/run/run_controller.c",
            "app/run/run_controller.h",
            "app/line/line_motion.c",
            "app/line/line_motion.h",
            "app/log/runtime_observer.c",
            "app/log/runtime_observer.h",
        ):
            self.assertTrue((ROOT / path).is_file(), path)

    def test_makefile_builds_new_app_modules(self):
        makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
        for source in (
            "app/run/run_controller.c",
            "app/line/line_motion.c",
            "app/log/runtime_observer.c",
        ):
            self.assertIn(source, makefile)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify RED**

Run: `python -m unittest tests.test_app_task_decomposition -v`
Expected: FAIL because the new modules do not exist and `app_tasks.c` is still over 260 lines.

- [ ] **Step 3: Commit the RED test**

```bash
git add tests/test_app_task_decomposition.py
git commit -m "test: specify app task decomposition"
```

---

### Task 2: Run Controller Module

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/app/run/run_controller.h`
- Create: `MSPM0G3507_LineFollowing_Car/app/run/run_controller.c`
- Test: `tests/test_run_controller_contract.py`

**Interfaces:**
- Consumes: `MotionRequest` from `shared/motion_request.h`; `KeyEvent` from `modules/key/key.h`.
- Produces:
  - `void RunController_Init(void);`
  - `void RunController_OnKeyEvent(KeyEvent event);`
  - `bool RunController_BuildRequest(uint32_t now_ms, MotionRequest *request);`

- [ ] **Step 1: Write failing tests**

```python
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class RunControllerContract(unittest.TestCase):
    def test_public_api_and_speed_are_bounded(self):
        header = (ROOT / "app/run/run_controller.h").read_text(encoding="utf-8")
        source = (ROOT / "app/run/run_controller.c").read_text(encoding="utf-8")
        for token in (
            "RunController_Init",
            "RunController_OnKeyEvent",
            "RunController_BuildRequest",
            "MotionRequest",
            "KeyEvent",
        ):
            self.assertIn(token, header + source)
        self.assertIn("RUN_CONTROLLER_BRINGUP_SPEED (120)", source)
        self.assertNotIn("Motor_Safety_RequestSpeed", source)
        self.assertNotIn("Motor_SendSpeedFrame", source)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify RED**

Run: `python -m unittest tests.test_run_controller_contract -v`
Expected: FAIL because `app/run/run_controller.*` does not exist.

- [ ] **Step 3: Implement the module**

`run_controller.h`:

```c
#ifndef APP_RUN_RUN_CONTROLLER_H
#define APP_RUN_RUN_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "../../modules/key/key.h"
#include "../../shared/motion_request.h"

void RunController_Init(void);
void RunController_OnKeyEvent(KeyEvent event);
bool RunController_BuildRequest(uint32_t now_ms, MotionRequest *request);

#endif
```

`run_controller.c`:

```c
#include "run_controller.h"

#define RUN_CONTROLLER_BRINGUP_SPEED (120)

static bool run_requested;

void RunController_Init(void)
{
    run_requested = true;
}

void RunController_OnKeyEvent(KeyEvent event)
{
    if (event == KEY_EVENT_SHORT) {
        run_requested = true;
    }
}

bool RunController_BuildRequest(uint32_t now_ms, MotionRequest *request)
{
    if (request == 0 || !run_requested) {
        return false;
    }
    request->left_speed = RUN_CONTROLLER_BRINGUP_SPEED;
    request->right_speed = RUN_CONTROLLER_BRINGUP_SPEED;
    request->timestamp_ms = now_ms;
    request->valid = true;
    return true;
}
```

- [ ] **Step 4: Run test to verify GREEN**

Run: `python -m unittest tests.test_run_controller_contract -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add MSPM0G3507_LineFollowing_Car/app/run tests/test_run_controller_contract.py
git commit -m "feat: add run controller module"
```

---

### Task 3: Line Motion Module

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/app/line/line_motion.h`
- Create: `MSPM0G3507_LineFollowing_Car/app/line/line_motion.c`
- Modify: `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c`
- Test: `tests/test_line_motion_contract.py`

**Interfaces:**
- Consumes: `AppLineSample`, `MotionRequest`, `LineLookupControl_Step`, `LineRecovery_Step`, `Mpu6050Snapshot`.
- Produces:
  - `void AppLineMotion_Init(uint32_t now_ms);`
  - `void AppLineMotion_ServiceImu(uint32_t now_ms);`
  - `bool AppLineMotion_BuildRequest(const AppLineSample *sample, uint32_t now_ms, MotionRequest *request);`

- [ ] **Step 1: Write failing tests**

```python
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class LineMotionContract(unittest.TestCase):
    def test_line_motion_owns_line_to_request_logic(self):
        header = (ROOT / "app/line/line_motion.h").read_text(encoding="utf-8")
        source = (ROOT / "app/line/line_motion.c").read_text(encoding="utf-8")
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        for token in (
            "AppLineMotion_Init",
            "AppLineMotion_ServiceImu",
            "AppLineMotion_BuildRequest",
            "LineLookupControl_Step",
            "LineRecovery_Step",
        ):
            self.assertIn(token, header + source)
        self.assertNotIn("LineLookupControl_Step", tasks)
        self.assertNotIn("LineRecovery_Step", tasks)
        self.assertNotIn("Mpu6050_Service", tasks)
        self.assertIn("AppLineMotion_BuildRequest", tasks)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify RED**

Run: `python -m unittest tests.test_line_motion_contract -v`
Expected: FAIL because `app/line/line_motion.*` does not exist.

- [ ] **Step 3: Move line/IMU helper logic from `app_tasks.c` into `line_motion.c`**

Move these static helpers out of `app_tasks.c` without changing their behavior:

- `ServiceImu`
- `PublishImuSnapshot`
- `ImuStartupHold`
- `ReadFreshYawRate`
- `PositionSign`
- `BuildMotionRequest`

Rename the externally used functions:

- `ServiceImu` -> `AppLineMotion_ServiceImu`
- `BuildMotionRequest` -> `AppLineMotion_BuildRequest`

Keep private helpers static inside `line_motion.c`.

- [ ] **Step 4: Update `app_tasks.c`**

Add `#include "../line/line_motion.h"`.
SensorTask calls `AppLineMotion_ServiceImu(now_ms)`.
ControlTask calls `AppLineMotion_BuildRequest(&sample, now_ms, &request)`.
AppTasks_Create calls `AppLineMotion_Init(now_ms)`.

- [ ] **Step 5: Run test to verify GREEN**

Run: `python -m unittest tests.test_line_motion_contract -v`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add MSPM0G3507_LineFollowing_Car/app/line MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c tests/test_line_motion_contract.py
git commit -m "refactor: extract line motion module"
```

---

### Task 4: Runtime Observer Module

**Files:**
- Create: `MSPM0G3507_LineFollowing_Car/app/log/runtime_observer.h`
- Create: `MSPM0G3507_LineFollowing_Car/app/log/runtime_observer.c`
- Modify: `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c`
- Test: `tests/test_runtime_observer_contract.py`

**Interfaces:**
- Consumes: task flags, `MotorSafetyDiagnostics`, `SafetySupervisorState`, `LineRecoveryState`.
- Produces:
  - `typedef struct RuntimeObserverInputs RuntimeObserverInputs;`
  - `void RuntimeObserver_Init(bool display_ready);`
  - `bool RuntimeObserver_Update(uint32_t now_ms, const RuntimeObserverInputs *inputs);`

- [ ] **Step 1: Write failing tests**

```python
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class RuntimeObserverContract(unittest.TestCase):
    def test_observer_owns_runtime_log_events(self):
        header = (ROOT / "app/log/runtime_observer.h").read_text(encoding="utf-8")
        source = (ROOT / "app/log/runtime_observer.c").read_text(encoding="utf-8")
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        for token in (
            "RuntimeObserverInputs",
            "RuntimeObserver_Init",
            "RuntimeObserver_Update",
            '"TEST RUN"',
            '"UART TIMEOUT"',
            '"WATCHDOG"',
        ):
            self.assertIn(token, header + source)
        self.assertNotIn("RuntimeLog_Push(now_ms", tasks)
        self.assertIn("RuntimeObserver_Update", tasks)
        self.assertNotIn('"TEST RUN"', tasks)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify RED**

Run: `python -m unittest tests.test_runtime_observer_contract -v`
Expected: FAIL because `app/log/runtime_observer.*` does not exist.

- [ ] **Step 3: Move DisplayTask observation logic into `runtime_observer.c`**

Move the runtime-log state variables and event checks out of DisplayTask:

- previous safety/recovery state
- previous motor applied speeds
- previous fault/direction wait
- logged safety/sensor/control/test-run booleans
- task-mask log tracking
- display re-init and flush behavior

Keep DisplayTask as: delay 100 ms, gather snapshots, call `RuntimeObserver_Update`.

- [ ] **Step 4: Run test to verify GREEN**

Run: `python -m unittest tests.test_runtime_observer_contract -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add MSPM0G3507_LineFollowing_Car/app/log MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c tests/test_runtime_observer_contract.py
git commit -m "refactor: extract runtime observer"
```

---

### Task 5: Wire Build And Finish Phase 1

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/Makefile`
- Modify: `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c`
- Test: `tests/test_app_task_decomposition.py`

**Interfaces:**
- Consumes: `RunController_*`, `AppLineMotion_*`, `RuntimeObserver_*`.
- Produces: app tasks shell and buildable firmware.

- [ ] **Step 1: Update Makefile sources**

Add these to `SOURCES`:

```make
    app/run/run_controller.c \
    app/line/line_motion.c \
    app/log/runtime_observer.c \
```

Add these include paths to `CPPFLAGS`:

```make
    -Iapp/run \
    -Iapp/line \
    -Iapp/log \
```

- [ ] **Step 2: Finish slimming `app_tasks.c`**

Remove local implementations that now live in app modules:

- `APP_BRINGUP_RUN_SPEED`
- `BuildBringupRunRequest`
- line motion helpers
- runtime log formatting checks

SafetyTask calls:

```c
RunController_OnKeyEvent(Key_PollEvent());
if (RunController_BuildRequest(now_ms, &request)) {
    /* request starts as bring-up motion */
}
```

DisplayTask gathers inputs and calls:

```c
(void)RuntimeObserver_Update(now_ms, &observer_inputs);
```

- [ ] **Step 3: Run focused decomposition tests**

Run:

```powershell
python -m unittest tests.test_app_task_decomposition tests.test_run_controller_contract tests.test_line_motion_contract tests.test_runtime_observer_contract -v
```

Expected: PASS.

- [ ] **Step 4: Run full host tests**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py"
```

Expected: all tests PASS.

- [ ] **Step 5: Clean generated caches**

Remove only generated paths inside the workspace:

- `D:\DevProject\MSPM0G3507__car\build\cli`
- `D:\DevProject\MSPM0G3507__car\MSPM0G3507_LineFollowing_Car\Debug`
- `D:\DevProject\MSPM0G3507__car\freertos_kernel\Debug`
- `D:\DevProject\MSPM0G3507__car\dist\firmware`
- root `*.obj` files if any exist

- [ ] **Step 6: Rebuild and generate TI-TXT**

Run:

```powershell
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C freertos_kernel clean all
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -C MSPM0G3507_LineFollowing_Car rebuild
```

Expected: exit code 0 and fresh files under `dist/firmware`.

- [ ] **Step 7: Verify generated TXT**

Run:

```powershell
$txt='D:\DevProject\MSPM0G3507__car\dist\firmware\MSPM0G3507_LineFollowing_Car.txt'
$lines=Get-Content -LiteralPath $txt
Get-Item -LiteralPath $txt | Select-Object FullName,Length,LastWriteTime
$lines[0]
$lines[-1]
Get-FileHash -LiteralPath $txt -Algorithm SHA256
```

Expected: first line starts with `@`, last line is `q`, and SHA256 is reported.

- [ ] **Step 8: Commit phase 1**

```bash
git add MSPM0G3507_LineFollowing_Car/app MSPM0G3507_LineFollowing_Car/Makefile tests/test_app_task_decomposition.py tests/test_run_controller_contract.py tests/test_line_motion_contract.py tests/test_runtime_observer_contract.py
git commit -m "refactor: split application runtime core"
```

---

## Next Phase Preview

After phase 1 is green and a fresh TXT exists:

- Phase 2: tighten motor boundaries and fault reason reporting.
- Phase 3: tighten line sensor X1-right/X8-left algorithms and docs.
- Phase 4: finish optional module quarantine and README cleanup.
