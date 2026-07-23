# Straight Boost and Early Corner Braking Design

## Goal

Raise average lap speed without sacrificing line-following accuracy. The car
may run faster only after the eight-channel sensor has confirmed a stable
straight. It must cancel the boost and slow down earlier when entering a
curve, addressing the observed failure mode of steering too late and running
outside the bend.

## Scope

- Keep the current eight-channel sensor, two-wheel drive, scheduler, trend
  detector, recovery state machine, safety supervisor, and motor protocol.
- Do not enable MPU6050, ultrasonic, or vision modules.
- Do not change lost-line backtracking or right-angle recovery behavior.
- Keep `LINE_CONTROL_KP` at 28.0.
- Keep every requested wheel command inside +/-450.
- Preserve soft-start, watchdog, request expiry, emergency stop, and the
  120 ms direction-change pause.

## Selected Approach

Use a confidence-gated straight boost inside the modular line controller.
Fixed speed increases alone are rejected because they also raise speed on
short or uncertain straights. A more complex curvature model is deferred
because the present sensor-only platform does not yet justify its tuning cost.

The controller maintains a saturating count of consecutive stable-straight
frames. A frame is stable only when all of the following are true:

- the estimate and trend are fresh;
- the trend type is `LINE_TREND_NORMAL`;
- the line event is `LINE_EVENT_NONE`;
- absolute predicted error is no greater than 1.0;
- confidence is at least 70.

Five consecutive stable frames unlock the straight target. Any frame that
fails these conditions resets the counter immediately.

## Initial Parameters

| Parameter | Current | New |
|---|---:|---:|
| Normal cruise command | 350 | 330 |
| Confirmed straight command | none | 400 |
| Straight confirmation | none | 5 frames |
| Straight error threshold | none | 1.0 |
| Curve error threshold | 2.0 | 1.5 |
| Hard-curve error threshold | 4.0 | 3.5 |
| Curve forward command | 270 | 240 |
| Hard-curve forward command | 180 | 150 |
| Tight-curve forward command | 140 | 120 |
| Hairpin forward command | 40 | 40 |
| Forward acceleration step | 15 | 15 |
| Forward deceleration step | 45 | 70 |
| Turn slew step | 20 | 25 |
| Proportional gain | 28.0 | 28.0 |

The confirmed straight target of 400 raises average speed on usable straight
sections. The lower cruise target prevents an immediate high-speed launch into
a short or uncertain segment. Earlier thresholds and faster deceleration
reduce overshoot before the turn command grows.

## Controller Data Flow

At each 5 ms control update:

1. Validate the fresh line estimate.
2. Validate the optional fresh trend result.
3. Update or reset the stable-straight frame counter.
4. Select the forward target:
   - confirmed straight: 400;
   - normal but unconfirmed: 330;
   - curve: 240;
   - hard curve: 150;
   - tight curve: 120;
   - hairpin: 40.
5. Apply the existing forward slew, with deceleration step 70.
6. Compute steering with KP 28 and the existing derivative term.
7. Apply turn slew step 25.
8. Clamp combined wheel commands through the existing controller, safety
   supervisor, and motor adapter limits.

Right-angle trend ownership remains in `line_recovery`; the controller does
not introduce another right-angle mode.

## Reset and Failure Behavior

- `LineController_Reset()` clears the straight-frame counter along with the
  previous forward and turn commands.
- Lost or stale estimates invalidate the controller output and clear all
  controller history.
- Missing, invalid, or stale trend data prevents straight boost but does not
  stop otherwise valid normal estimate control.
- Low confidence cancels straight boost and retains the existing low-confidence
  speed limit.
- Counter increments saturate and cannot wrap during multi-lap operation.

## Verification

Test-first implementation will cover:

- five qualifying frames are required before selecting 400;
- four qualifying frames remain at the 330 cruise target;
- one uncertain or curved frame immediately cancels boost;
- curve and hard-curve thresholds are 1.5 and 3.5;
- curve targets are 240, 150, 120, and 40;
- KP remains 28.0;
- turn slew is 25 and deceleration step is 70;
- left/right behavior stays mirror-symmetric;
- requested commands remain within +/-450;
- lost-line recovery timings and reversal pause remain unchanged.

After unit tests, affected sources will be compiled with TI Arm Clang 4.0.4,
the complete firmware will be relinked, and equivalent UniFlash HEX and TI-TXT
images will be generated and compared byte for byte.
