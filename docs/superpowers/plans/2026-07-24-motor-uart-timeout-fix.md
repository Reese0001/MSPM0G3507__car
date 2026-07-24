# Motor UART Timeout Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate the timing-dependent UART transmit timeout that latches D1 and stops the motors while preserving every existing motor safety behavior.

**Architecture:** Keep the public motor UART API unchanged. Replace CPU-iteration waits inside `Motor_Usart_SendArrayBounded()` with the existing 1 MHz timer, wait for TX FIFO space per byte, and wait for UART idle only after the complete frame has been queued.

**Tech Stack:** C11, TI MSPM0 DriverLib, TI Arm Clang 4.0.4 LTS, Python `unittest`.

## Global Constraints

- Work only in `D:\DevProject\MSPM0G3507__car\.worktrees\line-following-burn`.
- Do not modify generated SysConfig files.
- Keep soft-start, the 200 ms watchdog, direction-change pause, fault latch, and zero-speed emergency stop unchanged.
- Do not raise PWM or requested motor speed.
- Keep the fix within the motor UART module and one focused regression test.
- Commit the completed UART fix separately.

---

### Task 1: Use hardware time and TX FIFO state for bounded transmission

**Files:**
- Create: `tests/test_motor_uart_timeout.py`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/bsp_motor_usart.h`
- Modify: `MSPM0G3507_LineFollowing_Car/modules/motor/bsp_motor_usart.c`

**Interfaces:**
- Consumes: `uint32_t BSP_Time_GetUs(void)`, `DL_UART_Main_isTXFIFOFull()`, `DL_UART_isBusy()`.
- Produces: unchanged `bool Motor_Usart_SendArrayBounded(const uint8_t *data, uint16_t length)`.

- [ ] **Step 1: Write the failing source-contract test**

```python
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class MotorUartTimeoutContract(unittest.TestCase):
    def test_bounded_send_uses_hardware_time_and_fifo_space(self):
        source = (ROOT / "modules/motor/bsp_motor_usart.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        header = (ROOT / "modules/motor/bsp_motor_usart.h").read_text(
            encoding="utf-8", errors="ignore"
        )
        body = source[source.index("bool Motor_Usart_SendArrayBounded"):]
        body = body[:body.index("bool Motor_EmergencyStop_FromISR")]
        self.assertIn('#include "../../bsp/time/timer.h"', source)
        self.assertIn("BSP_Time_GetUs()", body)
        self.assertIn("DL_UART_Main_isTXFIFOFull", body)
        self.assertIn("DL_UART_isBusy", body)
        self.assertIn("MOTOR_UART_TX_TIMEOUT_US (5000U)", header)
        self.assertNotIn("wait_count", body)
        self.assertNotIn("MOTOR_UART_TX_WAIT_LIMIT", header)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```powershell
python -m unittest tests.test_motor_uart_timeout -v
```

Expected: FAIL because the current code contains `wait_count`, lacks
`BSP_Time_GetUs()` and does not inspect TX FIFO fullness.

- [ ] **Step 3: Implement the minimum time-based sender**

In `bsp_motor_usart.h`, replace the iteration limit with:

```c
#define MOTOR_UART_TX_TIMEOUT_US (5000U)
```

In `bsp_motor_usart.c`, include the existing timer:

```c
#include "../../bsp/time/timer.h"
```

Replace only `Motor_Usart_SendArrayBounded()` with:

```c
bool Motor_Usart_SendArrayBounded(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    for (index = 0U; index < length; index++) {
        uint32_t started_us = BSP_Time_GetUs();

        while (DL_UART_Main_isTXFIFOFull(Motor_INST)) {
            if ((uint32_t)(BSP_Time_GetUs() - started_us) >=
                MOTOR_UART_TX_TIMEOUT_US) {
                DL_UART_Main_disable(Motor_INST);
                DL_UART_Main_enable(Motor_INST);
                return false;
            }
        }
        DL_UART_Main_transmitData(Motor_INST, data[index]);
    }

    {
        uint32_t started_us = BSP_Time_GetUs();

        while (DL_UART_isBusy(Motor_INST)) {
            if ((uint32_t)(BSP_Time_GetUs() - started_us) >=
                MOTOR_UART_TX_TIMEOUT_US) {
                DL_UART_Main_disable(Motor_INST);
                DL_UART_Main_enable(Motor_INST);
                return false;
            }
        }
    }
    return true;
}
```

- [ ] **Step 4: Run focused and full host tests**

Run:

```powershell
python -m unittest tests.test_motor_uart_timeout -v
python -m unittest discover -s tests -p "test_*.py"
```

Expected: the focused test passes and the full suite reports zero failures.

- [ ] **Step 5: Compile and link with TI Arm Clang**

Run the repository's existing full-build commands used for
`Build_LineFollowing/obj_minimal`, then link
`Build_LineFollowing/MSPM0G3507_LineFollowing_Car.out`.

Expected: all active C sources and generated/startup sources compile, and the
linker exits with code 0.

- [ ] **Step 6: Regenerate UniFlash artifacts**

Convert the newly linked OUT file into:

```text
firmware/MSPM0G3507_LineFollowing_Car.hex
firmware/MSPM0G3507_LineFollowing_Car.txt
```

Expected: both files contain identical address/data records and have fresh
SHA-256 hashes.

- [ ] **Step 7: Commit the completed fix**

```powershell
git add -- tests/test_motor_uart_timeout.py `
  MSPM0G3507_LineFollowing_Car/modules/motor/bsp_motor_usart.c `
  MSPM0G3507_LineFollowing_Car/modules/motor/bsp_motor_usart.h
git commit -m "fix: make motor uart timeout time based"
```
