# Car platform and power acceptance contract

This document records the accepted baseline for the first car-control-platform
stage. It is a hardware acceptance boundary, not proof that the vehicle has
been run safely or that distance calculations are calibrated.

## Platform identity and motor assignment

- Main project path: `MSPM0G3507_LineFollowing_Car`.
- The purchased chassis uses two-drive L-shaped 520 Hall encoder motors. Its
  advertised overall dimensions are 228 mm x 148 mm x 102.15 mm and its wheels
  are 65 mm in diameter.
- M2 is left drive and M4 is right drive. M1/M3 remain zero at all times.
- The firmware motor selection is `MOTOR_TYPE=5`. The L520 motor is labelled
  12 V with an 11-line AB incremental encoder and a nominal reduction speed of
  about 300 rpm.

The 65 mm wheel diameter is an initial value taken from the purchase image.
Encoder pulses/rev, the exact pulse-to-revolution conversion, and the wheel
effective diameter remain to be measured on the assembled vehicle. Therefore
millimetre conversion is not validated and must not be presented as calibrated.

## Interfaces

| Interface | Accepted allocation |
|---|---|
| Debug UART | PA10/PA11 at 115200 |
| Motor UART1 | PB6/PB7 at 115200 |
| Eight-track select | PA15/PA16/PA17 select lines |
| Eight-track output | PA18 OUT; black line is low level |
| Eight-track address | 0x12 |

## Motor safety contract

All motion requests must go through Motor Safety.
`MOTOR_SAFETY_WATCHDOG_MS=200 ms` is the loss-of-control watchdog threshold:
after 200 ms without a new valid request, the safety layer must latch the
fault and command a fixed zero-speed frame. Starting motion must use the
existing 0-to-30% soft-start ramp; no request may bypass this safety layer.

## Power acceptance gate

The battery label states nominal 11.1 V, full-charge 12.6 V, and about 6 A
capability. The driver/expansion image is marked 5-12 V. Compatibility of a
12.6 V full-charge battery with the actual driver/expansion hardware is a hard
gate: do not connect the battery until its permissible supply range is verified
against the exact board and its documentation.

Stall-current risk must be considered before powering the motors. Provide a
suitable fuse and an accessible quick disconnect, then conduct the first
raised-wheel motor checks. Do not treat documentation
or offline tests as authorization to start a real motor.

## Vision boundary

The camera has not been purchased. This first stage does not change SysConfig
for vision and introduces no vision firmware interface. Camera and vision work
begin only after the required hardware is confirmed.
