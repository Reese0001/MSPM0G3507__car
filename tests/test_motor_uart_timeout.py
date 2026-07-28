from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class MotorUartTimeoutContract(unittest.TestCase):
    def test_bounded_send_uses_hardware_time_and_fifo_space(self):
        source = (ROOT / "modules/motor/uart/motor_uart.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        header = (ROOT / "modules/motor/uart/motor_uart.h").read_text(
            encoding="utf-8", errors="ignore"
        )
        body = source[source.index("bool Motor_Usart_SendArrayBounded"):]
        body = body[:body.index("bool Motor_EmergencyStop_FromISR")]
        self.assertIn('#include "../../time/timer.h"', source)
        self.assertIn("BSP_Time_GetUs()", body)
        self.assertIn("DL_UART_Main_isTXFIFOFull", body)
        self.assertIn("DL_UART_isBusy", body)
        self.assertIn("MOTOR_UART_TX_TIMEOUT_US (5000U)", header)
        self.assertNotIn("wait_count", body)
        self.assertNotIn("MOTOR_UART_TX_WAIT_LIMIT", header)


if __name__ == "__main__":
    unittest.main()
