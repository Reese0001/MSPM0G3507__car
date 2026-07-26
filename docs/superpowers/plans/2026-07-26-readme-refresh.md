# README Current-State Refresh Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the root README accurately describe the current FreeRTOS firmware, UniFlash workflow, SSD1306 diagnostics, safety gates, and verification status.

**Architecture:** Keep one concise quick-start document at the repository root. Link to the existing detailed verification record instead of duplicating its full tables, and derive every hardware or firmware claim from the current source and generated artifacts.

**Tech Stack:** Markdown, TI MSPM0G3507, FreeRTOS 11.2.0, TI Arm Clang 4.0.4, UniFlash TI-TXT, SSD1306 software I2C.

## Global Constraints

- Modify only `README.md`; do not change firmware, SysConfig, control parameters, or wiring definitions.
- Preserve the motor safety requirements: all requests pass through `Motor_Safety_RequestSpeed()`, 0→30% soft-start, ±450 command bound, and 200 ms watchdog stop.
- State that hardware acceptance remains incomplete until the staged car tests are recorded.
- Keep the README valid UTF-8 and avoid adding dependencies or scripts.

---

### Task 1: Refresh the root README

**Files:**
- Modify: `README.md`
- Reference: `docs/verification/sensor-platform-test-record.md`
- Reference: `MSPM0G3507_LineFollowing_Car/application/diagnostics/dashboard.c`
- Reference: `MSPM0G3507_LineFollowing_Car/modules/display/ssd1306.h`

**Interfaces:**
- Consumes: current pin assignments, build artifact names, dashboard field labels, and motor safety limits.
- Produces: one accurate root-level quick-start for build, flash, wiring, display verification, and staged testing.

- [ ] **Step 1: Replace stale architecture and control statements**

Change the opening and control sections to state:

```text
FreeRTOS uses four statically allocated tasks for sensing, control, safety/motor output, and display.
The eight-channel sensor is decoded into 15 legal positions from -7 to +7.
Steering uses the bounded lookup table; MPU6050 yaw only limits excessive corner rotation.
```

Remove claims that the project has no RTOS, PA10/PA11 provide UART0 debugging, or the active controller is the old PD loop.

- [ ] **Step 2: Correct the wiring table**

Document these exact active assignments:

```text
PB6/PB7       motor driver UART1, 115200
PA10/PA11     SSD1306 SCL/SDA, software I2C, address 0x3C
PA12/PA13     MPU6050 SCL/SDA, software I2C, address 0x68
PA15..PA17    line sensor AD0..AD2
PA18          line sensor OUT, also BSL Invoke during UniFlash entry
```

State that X1 is leftmost when viewed in the car's forward direction and black is active low in the current firmware.

- [ ] **Step 3: Add CCS and UniFlash build/flash instructions**

Include the generated artifacts:

```text
MSPM0G3507_LineFollowing_Car/Build_LineFollowing/MSPM0G3507_LineFollowing_Car.out
firmware/MSPM0G3507_LineFollowing_Car.hex
firmware/MSPM0G3507_LineFollowing_Car.txt
```

For UniFlash, select the MSPM0G3507 serial BSL workflow and load the TI-TXT `.txt` file. Require 12.6 V to remain disconnected, close software holding the COM port, and temporarily disconnect the line sensor from PA18 if it prevents BSL entry.

- [ ] **Step 4: Document OLED output and minimal troubleshooting**

List the expected page fields exactly:

```text
LINE CAR DIAG
B:xxxxxxxx Tn
P:+n C:+n
L:+nnn R:+nnn
Sn Rn Mn
F:OK or an actionable fault label
```

Troubleshoot in this order: 3.3 V and common ground, PA10/PA11 wiring, `0x3C` module address, newly generated firmware, then `OLED-I2C`. State that OLED failure is non-latching and must not stop the motor tasks.

- [ ] **Step 5: Preserve the staged safety gate**

Keep the motor checklist and require this sequence:

```text
USB-only diagnostics → suspended wheels → low-speed floor → lost-line test → sharp-turn test → full route
```

Link to `docs/verification/sensor-platform-test-record.md` for recording results.

- [ ] **Step 6: Verify the document**

Run:

```powershell
python -m unittest tests.test_text_encoding tests.test_oled_contract tests.test_freertos_contract -v
git diff --check
```

Expected: all selected tests pass and `git diff --check` prints no errors.

Also check every relative Markdown link in `README.md` resolves to an existing repository path.

- [ ] **Step 7: Commit**

```powershell
git add README.md
git commit -m "docs: refresh UniFlash and OLED quick start"
```
