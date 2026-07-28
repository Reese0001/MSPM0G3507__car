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
        path = ROOT / "modules/diagnostics/freertos_startup_wrappers.S"
        self.assertTrue(path.exists(), "FreeRTOS startup wrapper is missing")
        wrapper = path.read_text("utf-8")
        kernel = (REPO / "freertos_kernel/Makefile").read_text("utf-8")
        self.assertIn("FreeRTOS_SVC_Handler", wrapper)
        self.assertIn("FreeRTOS_vRestoreContextOfFirstTask", wrapper)
        self.assertIn("-DSVC_Handler=FreeRTOS_SVC_Handler", kernel)
        self.assertIn("-DvRestoreContextOfFirstTask=FreeRTOS_vRestoreContextOfFirstTask", kernel)

    def test_fatal_path_is_gpio_only(self):
        path = ROOT / "modules/diagnostics/boot_trace.c"
        self.assertTrue(path.exists(), "boot trace implementation is missing")
        source = path.read_text("utf-8")
        fatal = source[source.index("BootTrace_Fatal"):]
        for forbidden in ("RuntimeLog", "Ssd1306", "Motor_", "printf", "vTask"):
            self.assertNotIn(forbidden, fatal)

    def test_all_tasks_register_and_motor_waits_for_all(self):
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text("utf-8")
        for bit in (
            "BOOT_TASK_SAFETY", "BOOT_TASK_SENSOR",
            "BOOT_TASK_CONTROL", "BOOT_TASK_DISPLAY",
        ):
            self.assertIn(f"BootTrace_TaskOnline({bit})", tasks)
        arm = tasks.index("Motor_Safety_Arm();")
        self.assertIn("BootTrace_AllTasksOnline()", tasks[arm - 240:arm])


if __name__ == "__main__":
    unittest.main()
