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

## Approved review resolution: matching kernel build

Review identified that the SDK release archive used a different FreeRTOS
configuration from the application.  In particular, it enabled dynamic
allocation and software timers, which makes its `StaticTask_t` layout unsafe
for the application-owned static task buffers.  The approved resolution
replaces that archive reference with the repository-owned
`freertos_kernel/freertos_kernel_ticlang.projectspec` and matching
`freertos_kernel/Makefile`.

The project definition and Makefile link, rather than copy or modify, these
official SDK FreeRTOS 11.2.0 sources:

- `tasks.c`
- `list.c`
- `portable/TI_ARM_CLANG/ARM_CM0/port.c`
- `portable/TI_ARM_CLANG/ARM_CM0/portasm.c`

Both paths put `MSPM0G3507_LineFollowing_Car/FreeRTOSConfig.h` first on the
include path.  No `heap_*.c`, `timers.c`, queue, event-group, stream-buffer,
or POSIX support unit is compiled.  The car CCS metadata now references only
`freertos_kernel_ticlang.lib`; it no longer references the incompatible SDK
release project/library.

### Resolution TDD and verification evidence

The expanded kernel-agreement contract test was run RED before adding the
project files:

```text
FileNotFoundError: .../freertos_kernel/freertos_kernel_ticlang.projectspec
```

After implementation:

| Check | Result |
| --- | --- |
| `python -m unittest tests.test_freertos_contract -v` | PASS (2 tests) |
| `python -m unittest discover -s tests -v` | PASS (171 tests) |
| `gmake -C freertos_kernel -j4 all` | PASS; built `Debug/freertos_kernel_ticlang.lib` from the four selected SDK sources |
| `gmake -C MSPM0G3507_LineFollowing_Car/Debug -j4 all` | Still blocked by the pre-existing stale generated `Debug` makefiles |
| `git diff --check` | PASS |

The kernel build emitted one SDK `port.c` warning for an unused vector-table
local while `configASSERT` is not enabled.  It did not fail the build.

The car gmake failure occurs before it can consume the new library because
the checked-in generated Debug files still omit the FreeRTOS include path and
the application task source:

```text
../empty.c:4:10: fatal error: 'FreeRTOS.h' file not found
../BSP/delay.c:2:10: fatal error: 'timer.h' file not found
```

Per the approved scope, those generated files were not hand-edited.  Import
the repository kernel projectspec into CCS as `freertos_kernel_ticlang`, then
regenerate/rebuild the car Debug configuration so its generated makefiles
reflect the committed `.project` and `.cproject` metadata.

## Handler binding review fix

The CM0+ handler aliases were previously guarded by
`#ifndef __TI_COMPILER_VERSION__`, which prevented TI Arm Clang from binding
the FreeRTOS port symbols to the CMSIS startup handlers.  The guard was
removed so `xPortPendSVHandler`, `vPortSVCHandler`, and
`xPortSysTickHandler` are always mapped to `PendSV_Handler`, `SVC_Handler`,
and `SysTick_Handler` respectively.

The new handler contract was run RED first and failed because the TI compiler
guard was present.  After the minimal configuration change it passed:

| Check | Result |
| --- | --- |
| `python -m unittest tests.test_freertos_contract -v` | PASS (3 tests) |
| `python -m unittest discover -s tests -v` | PASS (172 tests) |
| `git diff --check` | PASS |
