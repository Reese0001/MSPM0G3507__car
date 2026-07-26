from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"
REPOSITORY = ROOT.parent


class FreeRtosContract(unittest.TestCase):
    def test_static_only_1khz_kernel_and_safe_start(self):
        config = (ROOT / "FreeRTOSConfig.h").read_text(encoding="utf-8")
        tasks = (ROOT / "application/freertos/app_tasks.c").read_text(encoding="utf-8")
        main = (ROOT / "empty.c").read_text(encoding="utf-8")

        self.assertIn("#define configTICK_RATE_HZ 1000", config)
        self.assertIn("#define configSUPPORT_STATIC_ALLOCATION 1", config)
        self.assertIn("#define configSUPPORT_DYNAMIC_ALLOCATION 0", config)
        self.assertIn("xTaskCreateStatic", tasks)
        self.assertIn("Motor_Safety_Disarm()", tasks)
        self.assertIn("vTaskStartScheduler()", main)

    def test_kernel_build_shares_static_only_configuration(self):
        kernel = REPOSITORY / "freertos_kernel"
        project = (kernel / "freertos_kernel_ticlang.projectspec").read_text(
            encoding="utf-8"
        )
        makefile = (kernel / "Makefile").read_text(encoding="utf-8")
        car_project = (ROOT / ".project").read_text(encoding="utf-8")
        car_cproject = (ROOT / ".cproject").read_text(encoding="utf-8")

        self.assertEqual(
            list(REPOSITORY.rglob("FreeRTOSConfig.h")),
            [ROOT / "FreeRTOSConfig.h"],
        )
        self.assertIn("MSPM0G3507_LineFollowing_Car/FreeRTOSConfig.h", project)
        self.assertIn("CONFIG_DIR := ../MSPM0G3507_LineFollowing_Car", makefile)
        self.assertIn("$(CONFIG_DIR)/FreeRTOSConfig.h", makefile)
        for source in (project, makefile):
            self.assertIn("tasks.c", source)
            self.assertIn("list.c", source)
            self.assertIn("port.c", source)
            self.assertIn("portasm.c", source)
            self.assertNotIn("heap_", source)
            self.assertNotIn("timers.c", source)

        self.assertIn("freertos_kernel_ticlang", car_project)
        self.assertIn("freertos_kernel_ticlang.lib", car_cproject)
        self.assertIn("${WORKSPACE_LOC}/freertos_kernel_ticlang/Debug", car_cproject)
        self.assertNotIn("freertos_builds_LP_MSPM0G3507_release_ticlang", car_project)
        self.assertNotIn("freertos_builds_LP_MSPM0G3507_release_ticlang", car_cproject)


if __name__ == "__main__":
    unittest.main()
