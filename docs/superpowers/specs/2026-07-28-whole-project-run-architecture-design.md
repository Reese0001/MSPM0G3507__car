# Whole Project Run Architecture Redesign

## Goal

Refactor the firmware so the car can run from RESET/K1 through a clear,
auditable control path. The current field symptom is:

- OLED last visible line: `0910 SCHED STRAT`
- D1: solid on
- D2: 250 ms heartbeat
- motor: no movement

D2 heartbeat means the scheduler and SafetyTask are alive. The failure is the
runtime application logic: motor permission, line-control availability, safety
faults, and OLED observations are coupled inside one large task file. The
project must be reorganized so each layer has one job and the motor path can be
checked independently before line following takes over.

## Non-Negotiables

- Keep one CCS project: `MSPM0G3507_LineFollowing_Car`.
- Keep UniFlash output at `dist/firmware/MSPM0G3507_LineFollowing_Car.txt`.
- Clean generated object/build caches before every firmware build.
- Keep `Set_Motor(5)` for the confirmed L-type 520 motor.
- Do not change the motor UART protocol unless a separate motor checklist is
  reviewed.
- No direct PWM and no 100% motor command.
- All motor motion must go through `Motor_Safety_RequestSpeed`.
- OLED is a debug log only; OLED failure must not stop the motor.
- X1 is the right sensor side and X8 is the left sensor side.

## Current Architecture Problem

The repository is already half-refactored into `app/`, `modules/`, `shared/`,
and `modules/optional/`, but `app/tasks/app_tasks.c` still owns too much:

- FreeRTOS task creation and task bodies
- line scanner service
- line-to-motion request construction
- IMU service burst
- RESET/K1 run intent
- bring-up default speed
- safety fault latching
- motor arbitration
- OLED runtime observation

That makes a running scheduler look like a stalled car: one missing line frame
or stale control request can suppress motor output, while the OLED log does not
show the exact decision boundary.

## Target Structure

```text
MSPM0G3507_LineFollowing_Car/
  app/
    boot/                  board and module initialization
    run/                   RESET/K1 run state and bring-up motion source
    line/                  app-level line-to-motion orchestration
    log/                   OLED runtime observer
    tasks/                 FreeRTOS task shells and creation only
    mailbox/               latest-value task mailboxes
    safety/                safety supervisor policy

  modules/
    motor/                 configuration, protocol, UART, adapter, safety
    line_tracking/         scanner, decoder, controller, recovery
    mpu6050/               IMU driver/service
    display/               SSD1306 and low-level runtime log buffer
    key/                   K1 input
    led/                   D1/D2 output
    time/                  millisecond/microsecond timebase
    optional/              k230, ultrasonic, ybimu, legacy code

  shared/                  small cross-layer structs only
```

## Runtime Data Flow

```text
RESET/K1
  -> app/run/RunController
  -> MotionRequest
  -> app/safety/SafetySupervisor
  -> modules/motor/adapter
  -> modules/motor/safety
  -> modules/motor/uart
```

Line following is parallel and optional during bring-up:

```text
line scanner -> line position/control/recovery -> MotionRequest
```

If the line request is fresh and valid, SafetyTask may use it. If it is absent,
the bring-up request still proves the motor path. ControlTask heartbeat must
not be a run-start gate.

## Phase 1: Split The App Runtime Core

Create:

- `app/run/run_controller.c/.h`
- `app/line/line_motion.c/.h`
- `app/log/runtime_observer.c/.h`

Keep `app/tasks/app_tasks.c` as the FreeRTOS shell:

- SensorTask calls scanner and publishes line samples.
- ControlTask calls `AppLineMotion_BuildRequest`.
- SafetyTask calls `RunController_Update`, `SafetySupervisor_Step`, and
  `MotorAdapter_Apply`.
- DisplayTask calls `RuntimeObserver_Update`.

Expected result: `app_tasks.c` no longer constructs line motion, formats OLED
events, or owns bring-up policy.

## Phase 2: Stabilize Motor And Line Boundaries

Motor:

- Keep motor config, protocol, UART, adapter, and safety separated.
- Verify only the adapter calls `Motor_Safety_RequestSpeed`.
- Verify the safety layer owns soft-start, watchdog, fault reason, and D1.

Line:

- Keep scanner, position decoder, lookup controller, trend, and recovery
  separate.
- Verify line modules produce `MotionRequest` data and never call motor APIs.
- Preserve X1-right/X8-left meaning in tests/docs.

## Phase 3: Clean Project Surface

- Keep active sources in the Makefile only once.
- Keep unused modules under `modules/optional`.
- Remove stale root-level object files and old build products.
- Update README with the new runtime path and field checklist.
- Generate a fresh UniFlash TI-TXT after clean build.

## Field Success Criteria

On a wheel-lifted first test after burning the generated TI-TXT:

- OLED shows boot/config/run log, including `MOTOR ARM` and `TEST RUN`.
- D2 shows heartbeat.
- D1 is not solid unless a real latched fault occurs.
- The driven motors receive low-speed soft-start output.
- If the car still does not move, OLED/D1/D2 identify whether the failing
  boundary is motor UART, motor watchdog, safety rejection, or missing power.

## Verification

Each phase must include failing tests first, then implementation, then:

- focused tests for the changed boundary;
- full `python -m unittest discover -s tests -p "test_*.py"`;
- clean FreeRTOS kernel build;
- clean application rebuild;
- regenerated `dist/firmware/MSPM0G3507_LineFollowing_Car.txt`;
- TXT format check: first line starts with `@`, last line is `q`, plus SHA256.
