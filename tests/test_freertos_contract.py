from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class NativeRuntimeContract(unittest.TestCase):
    def test_main_uses_native_poll_loop_not_freertos_scheduler(self):
        main = (ROOT / "empty.c").read_text(encoding="utf-8")
        self.assertIn("AppTasks_Init();", main)
        self.assertIn("AppTasks_Poll(Get_Time());", main)
        self.assertNotIn("vTaskStartScheduler", main)
        self.assertNotIn("#include \"FreeRTOS.h\"", main)
        self.assertNotIn("#include \"task.h\"", main)

    def test_car_image_does_not_link_freertos_kernel(self):
        makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
        cproject = (ROOT / ".cproject").read_text(encoding="utf-8")
        project = (ROOT / ".project").read_text(encoding="utf-8")
        self.assertNotIn("KERNEL_LIB", makefile)
        self.assertNotIn("freertos_kernel_ticlang.lib", makefile)
        self.assertNotIn("FREERTOS_INC", makefile)
        self.assertNotIn("FREERTOS_PORT", makefile)
        self.assertNotIn("freertos_startup_wrappers.S", makefile)
        self.assertNotIn("freertos_kernel_ticlang.lib", cproject)
        self.assertNotIn("kernel/freertos", cproject)
        self.assertNotIn("<project>freertos_kernel_ticlang</project>", project)
        self.assertIn("modules/diagnostics/freertos_startup_wrappers.S", cproject)

    def test_native_scheduler_keeps_small_periodic_boundaries(self):
        tasks = (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        for token in (
            "void AppTasks_Init(void)",
            "void AppTasks_Poll(uint32_t now_ms)",
            "FOUR_LINE_SAMPLE_PERIOD_MS",
            "DISPLAY_PERIOD_MS  (200U)",
            "sample_and_control(now_ms)",
            "Drive_Service(now_ms)",
            "Dashboard_Render(&diagnostics)",
        ):
            self.assertIn(token, tasks)
        for token in (
            "xTaskCreateStatic",
            "vTaskDelayUntil",
            "ulTaskNotifyTake",
            "taskENTER_CRITICAL",
        ):
            self.assertNotIn(token, tasks)

    def test_runtime_has_no_mailbox_layer(self):
        mailbox = ROOT / "app/mailbox"
        self.assertFalse(any(mailbox.glob("*.[ch]")))


if __name__ == "__main__":
    unittest.main()
