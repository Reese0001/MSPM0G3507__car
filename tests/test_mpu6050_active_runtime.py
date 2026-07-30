from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class Mpu6050ActiveRuntime(unittest.TestCase):
    def test_active_runtime_uses_mpu6050_not_ybimu(self):
        tasks = (PROJECT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        makefile = (PROJECT / "Makefile").read_text(encoding="utf-8")
        source = (PROJECT / "modules/imu/mpu6050.c").read_text(encoding="utf-8")
        config = (PROJECT / "modules/imu/mpu6050.h").read_text(encoding="utf-8")

        self.assertIn("Mpu6050_Init", tasks)
        self.assertIn("Mpu6050_Service", tasks)
        self.assertIn("Mpu6050_GetSnapshot", tasks)
        self.assertNotIn("YbImu_", tasks)
        self.assertIn("modules/imu/mpu6050.c", makefile)
        self.assertNotIn("modules/optional/ybimu/ybimu.c", makefile)
        self.assertIn("MPU6050_ADDRESS (0x68U)", source)
        self.assertIn("MPU6050_GYRO_ZOUT_H (0x47U)", source)
        self.assertIn("MPU6050_SAMPLE_PERIOD_MS (10U)", config)

    def test_sysconfig_restores_original_mpu6050_pins(self):
        syscfg = (PROJECT / "empty.syscfg").read_text(encoding="utf-8")
        i2c = (PROJECT / "bsp/bsp_i2c.c").read_text(encoding="utf-8")

        for token in ('GPIO5.$name                         = "MPU6050_I2C"',
                      'GPIO5.associatedPins[0].pin.$assign = "PA12"',
                      'GPIO5.associatedPins[1].pin.$assign = "PA13"'):
            self.assertIn(token, syscfg)
        self.assertIn("MPU6050_I2C_PORT", i2c)
        self.assertNotIn("YBIMU_I2C_PORT", i2c)


if __name__ == "__main__":
    unittest.main()
