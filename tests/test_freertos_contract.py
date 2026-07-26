from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


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


if __name__ == "__main__":
    unittest.main()
