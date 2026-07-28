# Run-First Bringup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the car request low-speed motor motion after RESET/K1 without waiting for a line sensor frame.

**Architecture:** Keep FreeRTOS and the existing safety/motor modules. Change only the app task layer so SafetyTask can publish a bounded bring-up motion request while line tracking is not yet producing control output. DisplayTask logs the run path and exact motor fault reason.

**Tech Stack:** C for MSPM0G3507 firmware, FreeRTOS static tasks, TI Arm Clang, Python unittest contract tests.

## Global Constraints

- Do not change the motor UART protocol.
- Keep `Set_Motor(5)`.
- Do not bypass `Motor_Safety`.
- Do not output direct PWM or 100% motor command.
- Keep all speed requests under the existing safety and adapter limits.
- Clean object/build caches before compiling and generating UniFlash TI-TXT.

---

## File Structure

- `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c`: owns task scheduling, bring-up request, K1 polling, OLED runtime observations.
- `tests/test_run_first_bringup.py`: static contract tests for the run-first behavior.
- `MSPM0G3507_LineFollowing_Car/README.md` and root `README.md`: mention the field behavior if the code change alters the visible startup log.

---

### Task 1: Contract Tests

**Files:**
- Create: `tests/test_run_first_bringup.py`

**Interfaces:**
- Consumes: `AppTasks_Create`, `SafetyTask`, `DisplayTask`, `Motor_Safety_RequestSpeed`.
- Produces: failing tests that require bring-up run behavior.

- [ ] **Step 1: Write the failing tests**

```python
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TASKS = ROOT / "MSPM0G3507_LineFollowing_Car" / "app" / "tasks" / "app_tasks.c"

def read_tasks():
    return TASKS.read_text(encoding="utf-8")

class RunFirstBringupContract(unittest.TestCase):
    def setUp(self):
        self.source = read_tasks()

    def test_bringup_constants_are_low_and_bounded(self):
        self.assertIn("APP_BRINGUP_RUN_SPEED", self.source)
        match = re.search(r"#define\s+APP_BRINGUP_RUN_SPEED\s+\((\d+)\)", self.source)
        self.assertIsNotNone(match)
        self.assertLessEqual(int(match.group(1)), 180)

    def test_safety_task_builds_default_run_without_line_frame(self):
        safety = self.source[self.source.index("static void SafetyTask"):]
        safety = safety[: self.source.index("static void DisplayTask")]
        self.assertIn("BuildBringupRunRequest", safety)
        self.assertLess(safety.index("BuildBringupRunRequest"),
                        safety.index("AppMailbox_ReadMotionRequest"))
        self.assertNotIn("APP_FAULT_CONTROL_HEARTBEAT", safety)

    def test_k1_can_request_run_after_boot(self):
        self.assertIn('#include "../../modules/key/key.h"', self.source)
        safety = self.source[self.source.index("static void SafetyTask"):]
        safety = safety[: self.source.index("static void DisplayTask")]
        self.assertIn("Key_PollEvent()", safety)
        self.assertIn("KEY_EVENT_SHORT", safety)
        self.assertIn("bringup_run_requested = true", safety)

    def test_motor_output_still_goes_through_safety_layer(self):
        self.assertIn("MotorAdapter_Apply(&decision);", self.source)
        self.assertNotIn("Motor_SendSpeedFrame(", self.source)
        self.assertNotIn("Send_Motor_ArrayU8(", self.source)

    def test_oled_logs_test_run(self):
        self.assertIn('"TEST RUN"', self.source)

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Verify RED**

Run: `python -m unittest tests.test_run_first_bringup -v`
Expected: FAIL because `APP_BRINGUP_RUN_SPEED` and `BuildBringupRunRequest` do not exist yet.

- [ ] **Step 3: Commit tests after RED is observed**

```bash
git add tests/test_run_first_bringup.py
git commit -m "test: specify run-first bringup"
```

---

### Task 2: Minimal Run-First Implementation

**Files:**
- Modify: `MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c`

**Interfaces:**
- Consumes: `MotorAdapter_Apply(const SafetyDecision *)`, `SafetySupervisor_Step`, `Key_PollEvent`.
- Produces: `BuildBringupRunRequest(uint32_t now_ms, MotionRequest *request)` inside `app_tasks.c`.

- [ ] **Step 1: Add low-speed constants and K1 include**

```c
#include "../../modules/key/key.h"

#define APP_BRINGUP_RUN_SPEED (120)
```

- [ ] **Step 2: Add the bring-up request helper**

```c
static void BuildBringupRunRequest(uint32_t now_ms, MotionRequest *request)
{
    request->left_speed = APP_BRINGUP_RUN_SPEED;
    request->right_speed = APP_BRINGUP_RUN_SPEED;
    request->timestamp_ms = now_ms;
    request->valid = true;
}
```

- [ ] **Step 3: Use the helper in SafetyTask**

Initialize `bringup_run_requested = true`. Each SafetyTask loop polls K1 and sets it true on `KEY_EVENT_SHORT`. Before reading the mailbox, build a bring-up request when requested. Then let a valid fresh mailbox request override it.

- [ ] **Step 4: Do not latch ControlTask heartbeat during bring-up**

Keep motor and sensor heartbeat fault handling, but remove `APP_FAULT_CONTROL_HEARTBEAT` from SafetyTask so a missing line-control notification cannot stop the bring-up run.

- [ ] **Step 5: Log `TEST RUN` once**

DisplayTask logs `TEST RUN` when the applied left or right speed first becomes non-zero.

- [ ] **Step 6: Verify GREEN**

Run: `python -m unittest tests.test_run_first_bringup -v`
Expected: PASS.

- [ ] **Step 7: Commit implementation**

```bash
git add MSPM0G3507_LineFollowing_Car/app/tasks/app_tasks.c tests/test_run_first_bringup.py
git commit -m "feat: add run-first bringup mode"
```

---

### Task 3: Full Verification And Firmware

**Files:**
- Generated: `dist/firmware/MSPM0G3507_LineFollowing_Car.txt`

**Interfaces:**
- Consumes: project Makefiles and existing firmware packaging.
- Produces: a fresh UniFlash TI-TXT.

- [ ] **Step 1: Clean caches**

Remove only generated build/cache outputs under the workspace: root `*.obj`, `MSPM0G3507_LineFollowing_Car/build`, `MSPM0G3507_LineFollowing_Car/Debug`, `freertos_kernel/Debug`, and `dist/firmware` outputs that will be regenerated.

- [ ] **Step 2: Run focused and full tests**

Run: `python -m unittest tests.test_run_first_bringup -v`
Run: `python -m unittest discover -s tests -p "test_*.py"`

- [ ] **Step 3: Rebuild firmware and generate TI-TXT**

Use the existing project build commands and conversion path already used in this repository.

- [ ] **Step 4: Verify TXT**

Confirm the `.txt` exists, starts with `@`, ends with `q`, and report timestamp, size, and SHA256.
