from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class NativeStartupDiagnostics(unittest.TestCase):
    def test_startup_boundaries_are_named_without_freertos_handlers(self):
        header = (ROOT / "modules/diagnostics/boot_trace.h").read_text("utf-8")
        for name in (
            "BOOT_TRACE_MAIN",
            "BOOT_TRACE_TASKS_CREATED",
            "BOOT_TRACE_SCHED_START",
            "BOOT_TRACE_ALL_TASKS",
            "BOOT_TASK_SENSOR",
            "BOOT_TASK_CONTROL",
            "BOOT_TASK_SAFETY",
            "BOOT_TASK_DISPLAY",
        ):
            self.assertIn(name, header)
        for name in ("SVC_Handler", "PendSV_Handler", "SysTick_Handler"):
            self.assertNotIn(name, header)

    def test_fatal_path_is_gpio_only(self):
        source = (ROOT / "modules/diagnostics/boot_trace.c").read_text("utf-8")
        fatal = source[source.index("BootTrace_Fatal"):]
        for forbidden in ("RuntimeLog", "Ssd1306", "Motor_", "printf", "vTask"):
            self.assertNotIn(forbidden, fatal)

    def test_fatal_masks_interrupts_and_initializes_leds(self):
        source = (ROOT / "modules/diagnostics/boot_trace.c").read_text("utf-8")
        fatal = source[source.index("BootTrace_Fatal"):]
        self.assertIn("__disable_irq();", fatal)
        self.assertIn("DL_GPIO_clearPins(LED_PORT, LED_D2_PIN);", fatal)
        disable = fatal.index("__disable_irq();")
        clear_d2 = fatal.index("DL_GPIO_clearPins(LED_PORT, LED_D2_PIN);")
        set_d1 = fatal.index("DL_GPIO_setPins(LED_PORT, LED_D1_PIN);")
        loop = fatal.index("for (;;)")
        self.assertLess(disable, clear_d2)
        self.assertLess(clear_d2, set_d1)
        self.assertLess(set_d1, loop)

    def test_partial_task_mask_uses_nonblocking_1ms_gpio_service(self):
        header = (ROOT / "modules/diagnostics/boot_trace.h").read_text("utf-8")
        source = (ROOT / "modules/diagnostics/boot_trace.c").read_text("utf-8")
        boot = (ROOT / "app/boot/app_boot.c").read_text("utf-8")
        self.assertIn("void BootTrace_Tick1ms(void);", header)
        self.assertIn("BootTrace_Tick1ms();", boot)
        tick = source[
            source.index("void BootTrace_Tick1ms"):
            source.index("BootTrace_Fatal")
        ]
        self.assertIn("BOOT_TRACE_PULSE_MS (100U)", source)
        self.assertIn("BOOT_TRACE_GROUP_GAP_MS", source)
        self.assertIn("BootTrace_Popcount", tick)
        self.assertNotIn("BootTrace_Delay", tick)

    def test_native_scheduler_registers_all_runtime_boundaries(self):
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text("utf-8")
        boot_trace = (ROOT / "modules/diagnostics/boot_trace.c").read_text("utf-8")
        for bit in (
            "BOOT_TASK_SAFETY",
            "BOOT_TASK_SENSOR",
            "BOOT_TASK_CONTROL",
            "BOOT_TASK_DISPLAY",
        ):
            self.assertIn(f"BootTrace_TaskOnline({bit})", tasks)
        self.assertIn("case BOOT_TRACE_ALL_TASKS:", boot_trace)


if __name__ == "__main__":
    unittest.main()
