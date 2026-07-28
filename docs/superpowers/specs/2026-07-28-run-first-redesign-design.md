# Run-First Redesign Design

## Goal

Make the car move reliably before optimizing line following. RESET starts the
firmware automatically, and K1 can also request a start while the system is
running. OLED is a rolling debug log so field reports can identify whether the
motor path, safety path, or line sensor path is failing.

## Current Problem

The current FreeRTOS application only produces motor motion after several
runtime conditions line up: all tasks are online, the motor is configured, the
line scanner publishes a frame, ControlTask publishes a fresh motion request,
and SafetyTask approves it. This is correct for a mature race mode, but too
fragile for bring-up. A missing line frame or blocked control notification
turns into zero motor output, so the hardware cannot prove that the motor path
still works.

## Chosen Approach

Use a run-first application layer:

- Keep the existing motor UART protocol, `Set_Motor(5)`, timer, OLED log, and
  `Motor_Safety` soft-start/watchdog layer.
- Add one small bring-up run path that requests a low forward speed after boot
  when the motor is configured and armed.
- Treat line tracking as an input source, not as the gate that allows the motor
  to move.
- Keep OLED messages simple: boot, config result, arm, test run, line frame,
  control request, and exact motor fault reason.
- Keep D2 as heartbeat when the scheduler and safety loop are alive; D1 means a
  latched runtime fault.

## Behavior

After RESET:

1. Initialize SysConfig, timer, motor UART, safety, OLED, and runtime log.
2. Configure the confirmed L-type 520 motor using `Set_Motor(5)`.
3. Start the scheduler.
4. SafetyTask arms the motor only after the motor is configured and the task
   set is alive.
5. A bring-up run request commands a low forward speed through
   `Motor_Safety_RequestSpeed(0, left, 0, right)`.
6. K1 short press re-requests the same run command after boot.
7. If line tracking later produces valid control output, it may replace the
   bring-up request through the same motion request boundary.

## Safety Limits

This design does not change the motor protocol, does not change the confirmed
motor type, and does not bypass `Motor_Safety`. Speed remains below the
existing safety limit and still ramps from zero. No direct PWM or 100% command
is allowed.

The first field test must lift or disconnect the drive wheels, burn the new
UniFlash TXT, press RESET, and confirm OLED shows the run path before placing
the wheels on the ground.

## Testing

Write tests before implementation to prove:

- the application has a bring-up run mode that does not depend on a line frame;
- motor arming remains owned by SafetyTask, not boot code or display code;
- the run command goes through `Motor_Safety_RequestSpeed`, not direct UART;
- K1 is polled after boot and can request the same run path;
- OLED logs the chosen run and motor fault states.

After tests pass, clean old object/build caches, rebuild, and generate the
UniFlash TI-TXT.
