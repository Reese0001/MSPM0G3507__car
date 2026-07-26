# Task 1 report: motor-safe FreeRTOS skeleton

## Outcome

Implemented the Task 1 FreeRTOS skeleton.  The boot path initializes the
existing safety timer and motor UART, keeps the motor safety layer disarmed,
creates one statically allocated bootstrap task, and starts the scheduler.
The bootstrap task disarms again and only delays; it does not start line
control, MPU, OLED, display, or motor-output behavior.

## Changes

- Added `FreeRTOSConfig.h` for a 80 MHz, 1 kHz, static-only application
  configuration, including the MSPM0 CM0+ priority and exception-handler
  definitions from the SDK reference configuration.
- Added `application/freertos/app_tasks.[ch]` with one static bootstrap task,
  static idle-task memory, and a safety stack-overflow hook.
- Replaced the cooperative main loop with `AppTasks_Create()` followed by
  `vTaskStartScheduler()`.  Both the task-create failure path and the
  scheduler-return path disarm motors.
- Removed the old `AppScheduler_Init()` boot call, because it arms the motor
  safety layer.  The retained 1 ms hardware timer callback still calls
  `Motor_Safety_Tick1ms()`.
- Removed SysConfig SYSTICK ownership and changed `delay_us()` to use the
  TIMG12 `BSP_Time_GetUs()` timebase.
- Added the official FreeRTOS dependency project reference, include paths,
  library search path, and library name to the CCS metadata; no FreeRTOS
  kernel source was copied.
- Added the required source-contract test and updated affected pre-FreeRTOS
  contracts to assert disarmed scheduler boot rather than the removed polling
  loop.

## TDD evidence

The contract test was added before implementation and run RED:

```text
FileNotFoundError: .../MSPM0G3507_LineFollowing_Car/FreeRTOSConfig.h
```

After implementation it passed.

## Verification

| Check | Result |
| --- | --- |
| `python -m unittest tests.test_freertos_contract -v` | PASS (1 test) |
| `python -m unittest discover -s tests -v` | PASS (170 tests) |
| TI Clang syntax check: `application/freertos/app_tasks.c` | PASS |
| TI Clang syntax check: `empty.c` with generated SysConfig header | PASS |
| `gmake -C MSPM0G3507_LineFollowing_Car/Debug -j4 all` | BLOCKED by stale generated Debug makefiles |
| `git diff --check` | PASS |

The shell did not expose `gmake` on `PATH`; rerunning with the installed CCS
binary (`D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe`) reached the
compile phase.  It failed before linking because the existing generated
`Debug` makefiles still use the pre-migration include set and source layout:

```text
../empty.c:4:10: fatal error: 'FreeRTOS.h' file not found
../BSP/delay.c:2:10: fatal error: 'timer.h' file not found
```

Those generated files do not contain the new `.cproject` FreeRTOS include
paths and still reference the obsolete `BSP/Timer` layout.  Regenerating the
Debug project from CCS after importing the official
`freertos_builds_LP_MSPM0G3507_release_ticlang` dependency is required for a
representative final link.  This task intentionally does not copy or fork the
kernel to bypass that dependency.

## Safety review

- `empty.c` does not call `Motor_Safety_Arm()`.
- `App_Main_Init()` no longer initializes the legacy scheduler that arms the
  motor layer.
- `BootstrapTask()` calls `Motor_Safety_Disarm()` before blocking.
- The existing hardware 1 ms `Motor_Safety_Tick1ms()` callback remains
  registered.
- The existing soft-start implementation is untouched.
- No files under `firmware/` were modified or staged.

## Remaining concerns

The source and host tests are green, but a final TI link remains pending CCS
project regeneration/import of the official dependency.  The failure is
generated-build metadata, not an attempt to substitute a local FreeRTOS copy.

The referenced official dependency project is not present in this workspace
yet.  In addition, its SDK release configuration enables dynamic allocation
and software timers, whereas this task requires a static-only, no-timer
application configuration.  FreeRTOS task control-block layout is
configuration-dependent, so the imported dependency must be rebuilt with the
same `FreeRTOSConfig.h` before a target flash/link can be considered safe.
This task leaves that external project untouched rather than copying or
forking the kernel.
