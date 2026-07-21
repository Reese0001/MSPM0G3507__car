# Task 5 report — fail-closed car route state machine

## Implemented

- Added `BSP/CarControl/car_route.h/.c` with the exact route-state API.
- `CarRoute_Start` is explicit, checks the motor safety fault latch, arms the
  safety layer, and enters `CAR_ROUTE_LINE_FOLLOW` only from IDLE or STOPPED.
- Line-follow steps are nonblocking and use `CarMotion_Command` as the only
  motion boundary. Null, invalid, inconsistent, timestamp-zero, or feedback
  aged at least 200 ms immediately calls `CarMotion_Stop` and latches
  `CAR_ROUTE_FAULT`.
- Stop-marker candidates require two consecutive valid frames before entering
  STOPPED; the first candidate already requests zero motion while debouncing.
- Distance/angle states fail closed when `units_valid` is false and are not
  enabled even when a forced state supplies valid units; Task 3B calibration is
  still required before implementation.
- SEARCH_LINE and TARGET_APPROACH remain declared but disabled.
- No SysConfig, generated files, camera code, motor protocol, or `empty.c` was
  modified.

## Verification

```text
python -m unittest tests.test_car_route_contract -v
Ran 7 tests ... OK

python -m unittest discover -s tests -v
Ran 46 tests ... OK
```

CCS target build and hardware behavior were not claimed or run. Real motor
operation remains unverified and must only be tested after the safety and
hardware wiring checklist is confirmed.

`CarSensorFrame` currently has no timestamp. The caller must pass the frame
created by the current cycle's successful `CarSensor_ReadFrame` call; detecting
an old copied sensor frame requires a later public-interface change rather than
pretending the existing structure carries freshness metadata.
