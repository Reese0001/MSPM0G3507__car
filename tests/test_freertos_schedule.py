from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class FreeRtosScheduleContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tasks = (ROOT / "application/freertos/app_tasks.c").read_text(
            encoding="utf-8"
        )
        cls.mailbox = (ROOT / "application/freertos/app_mailbox.c").read_text(
            encoding="utf-8"
        )
        cls.config = (ROOT / "application/config/safety_config.h").read_text(
            encoding="utf-8"
        )
        cls.entry = (ROOT / "empty.c").read_text(encoding="utf-8")
        cls.scanner_h = (
            ROOT / "modules/line_tracking/line_scanner.h"
        ).read_text(encoding="utf-8")
        cls.scanner_c = (
            ROOT / "modules/line_tracking/line_scanner.c"
        ).read_text(encoding="utf-8")
        cls.timer = (ROOT / "bsp/time/timer.c").read_text(encoding="utf-8")

    def test_four_static_tasks_with_expected_timing(self):
        self.assertGreaterEqual(self.tasks.count("xTaskCreateStatic"), 4)
        self.assertIn("pdMS_TO_TICKS(2U)", self.tasks)
        self.assertIn("pdMS_TO_TICKS(1U)", self.tasks)
        self.assertIn("vTaskDelayUntil", self.tasks)
        self.assertIn("ulTaskNotifyTake", self.tasks)
        self.assertIn("xTaskNotifyGive", self.tasks)

    def test_priorities_put_safety_above_control_above_sensor(self):
        self.assertIn("#define APP_TASK_PRIORITY_DISPLAY 1U", self.tasks)
        self.assertIn("#define APP_TASK_PRIORITY_SENSOR  2U", self.tasks)
        self.assertIn("#define APP_TASK_PRIORITY_CONTROL 3U", self.tasks)
        self.assertIn("#define APP_TASK_PRIORITY_SAFETY  4U", self.tasks)

    def test_motor_uart_is_rate_limited_to_five_ms(self):
        self.assertIn("MOTOR_UART_MIN_PERIOD_MS (5U)", self.config)
        self.assertIn("MOTOR_UART_MIN_PERIOD_MS", self.tasks)

    def test_cooperative_polling_is_not_the_active_path(self):
        self.assertNotIn("AppScheduler_Run(now_ms)", self.entry)
        self.assertNotIn("AppScheduler_Run(", self.tasks)

    def test_sensor_task_uses_bounded_full_frame_scan(self):
        self.assertIn(
            "bool LineScanner_ReadFrame(uint32_t now_ms, "
            "LineSensorSnapshot *out);",
            self.scanner_h,
        )
        self.assertIn("LINE_SCAN_FRAME_BUDGET_US", self.scanner_c)
        self.assertIn("LineScanner_ReadFrame", self.tasks)
        self.assertIn("LinePosition_Update", self.tasks)

    def test_mpu_completion_loop_is_bounded_to_one_millisecond(self):
        self.assertIn("BSP_I2C_Service", self.tasks)
        self.assertIn("< 1000U", self.tasks)

    def test_mailboxes_copy_snapshots_inside_critical_sections(self):
        self.assertIn("taskENTER_CRITICAL()", self.mailbox)
        self.assertIn("taskEXIT_CRITICAL()", self.mailbox)
        self.assertNotIn("pvPortMalloc", self.mailbox + self.tasks)

    def test_hardware_tick_isr_stays_freertos_free(self):
        self.assertNotIn("FreeRTOS", self.timer)
        self.assertNotIn("FromISR", self.timer)
        self.assertIn("tick_callback()", self.timer)


if __name__ == "__main__":
    unittest.main()
