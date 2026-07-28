from pathlib import Path
import unittest

REPO = Path(__file__).resolve().parents[1]
ROOT = REPO / "MSPM0G3507_LineFollowing_Car"


class FreeRtosStartupDiagnostics(unittest.TestCase):
    def test_all_startup_boundaries_are_named(self):
        path = ROOT / "modules/diagnostics/boot_trace.h"
        self.assertTrue(path.exists(), "boot trace header is missing")
        header = path.read_text("utf-8")
        for name in (
            "BOOT_TRACE_MAIN", "BOOT_TRACE_TASKS_CREATED",
            "BOOT_TRACE_SCHED_START", "BOOT_TRACE_PORT_START",
            "BOOT_TRACE_SVC", "BOOT_TRACE_FIRST_RESTORE",
            "BOOT_TRACE_SAFETY_TASK", "BOOT_TRACE_SENSOR_TASK",
            "BOOT_TRACE_CONTROL_TASK", "BOOT_TRACE_DISPLAY_TASK",
            "BOOT_TRACE_ALL_TASKS",
        ):
            self.assertIn(name, header)

    def test_port_wrappers_route_to_official_handlers(self):
        app_wrapper = (
            ROOT / "modules/diagnostics/freertos_startup_wrappers.S"
        ).read_text("utf-8")
        path = REPO / "freertos_kernel/portasm_diagnostic.c"
        self.assertTrue(path.exists(), "diagnostic portasm wrapper is missing")
        kernel_wrapper = path.read_text("utf-8")
        kernel = (REPO / "freertos_kernel/Makefile").read_text("utf-8")
        project = (
            REPO / "freertos_kernel/freertos_kernel_ticlang.projectspec"
        ).read_text("utf-8")

        self.assertIn("FreeRTOS_SVC_Handler", app_wrapper)
        self.assertIn("FreeRTOS_vRestoreContextOfFirstTask", app_wrapper)
        self.assertIn("#define SVC_Handler FreeRTOS_SVC_Handler", kernel_wrapper)
        self.assertIn(
            "#define vRestoreContextOfFirstTask "
            "FreeRTOS_vRestoreContextOfFirstTask",
            kernel_wrapper,
        )
        self.assertIn('#include "portasm.c"', kernel_wrapper)
        for build_description in (kernel, project):
            self.assertIn("portasm_diagnostic.c", build_description)
        self.assertNotIn("PORTASM_DIAG_FLAGS", kernel)
        self.assertNotIn("-DSVC_Handler=FreeRTOS_SVC_Handler", kernel)
        self.assertNotIn(
            "-DvRestoreContextOfFirstTask=FreeRTOS_vRestoreContextOfFirstTask",
            kernel,
        )

    def test_fatal_path_is_gpio_only(self):
        path = ROOT / "modules/diagnostics/boot_trace.c"
        self.assertTrue(path.exists(), "boot trace implementation is missing")
        source = path.read_text("utf-8")
        fatal = source[source.index("BootTrace_Fatal"):]
        for forbidden in ("RuntimeLog", "Ssd1306", "Motor_", "printf", "vTask"):
            self.assertNotIn(forbidden, fatal)

    def test_fatal_masks_interrupts_and_initializes_leds(self):
        source = (
            ROOT / "modules/diagnostics/boot_trace.c"
        ).read_text("utf-8")
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
        self.assertIn("BOOT_FATAL_GROUP_GAP_DELAYS (5U)", source)

    def test_partial_task_mask_uses_nonblocking_1ms_gpio_service(self):
        header = (
            ROOT / "modules/diagnostics/boot_trace.h"
        ).read_text("utf-8")
        source = (
            ROOT / "modules/diagnostics/boot_trace.c"
        ).read_text("utf-8")
        boot = (ROOT / "app/boot/app_boot.c").read_text("utf-8")

        self.assertIn("void BootTrace_Tick1ms(void);", header)
        self.assertIn("BootTrace_Tick1ms();", boot)
        self.assertGreater(
            boot.index("BootTrace_Tick1ms();"),
            boot.index("Motor_Safety_Tick1ms();"),
        )
        tick = source[
            source.index("void BootTrace_Tick1ms"):
            source.index("BootTrace_Fatal")
        ]
        self.assertIn("BOOT_TRACE_PULSE_MS (100U)", source)
        self.assertIn("BOOT_TRACE_GROUP_GAP_MS", source)
        self.assertIn("BootTrace_Popcount", tick)
        self.assertIn("DL_GPIO_clearPins(LED_PORT, LED_D1_PIN);", tick)
        self.assertIn("DL_GPIO_setPins(LED_PORT, LED_D2_PIN);", tick)
        self.assertIn("DL_GPIO_clearPins(LED_PORT, LED_D2_PIN);", tick)
        self.assertNotIn("BootTrace_Delay", tick)
        self.assertNotIn("/", tick)
        self.assertNotIn("%", tick)

    def test_all_tasks_register_and_motor_waits_for_all(self):
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text("utf-8")
        safety_runtime = (
            ROOT / "app/safety/safety_runtime.c"
        ).read_text("utf-8")
        for bit in (
            "BOOT_TASK_SAFETY", "BOOT_TASK_SENSOR",
            "BOOT_TASK_CONTROL", "BOOT_TASK_DISPLAY",
        ):
            self.assertIn(f"BootTrace_TaskOnline({bit})", tasks)
        arm = safety_runtime.index("Motor_Safety_Arm();")
        self.assertIn("BootTrace_AllTasksOnline()", safety_runtime[arm - 240:arm])

    def test_all_tasks_clear_startup_leds_before_heartbeat_handoff(self):
        source = (
            ROOT / "modules/diagnostics/boot_trace.c"
        ).read_text("utf-8")
        self.assertIn("case BOOT_TRACE_ALL_TASKS:", source)
        all_tasks_case = source[
            source.index("case BOOT_TRACE_ALL_TASKS:"):
            source.index("default:", source.index("case BOOT_TRACE_ALL_TASKS:"))
        ]
        task_online = source[
            source.index("void BootTrace_TaskOnline"):
            source.index("bool BootTrace_AllTasksOnline")
        ]

        self.assertIn(
            "DL_GPIO_clearPins(LED_PORT, LED_D1_PIN | LED_D2_PIN);",
            all_tasks_case,
        )
        self.assertIn(
            "BootTrace_SetStagePins(BOOT_TRACE_ALL_TASKS);",
            task_online,
        )


if __name__ == "__main__":
    unittest.main()
