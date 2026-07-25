from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class Mpu6050Contract(unittest.TestCase):
    def test_sysconfig_assigns_mpu6050_bus(self):
        syscfg = (ROOT / "empty.syscfg").read_text(encoding="utf-8")
        self.assertIn("GPIO5.$name", syscfg)
        self.assertIn('"MPU6050_I2C"', syscfg)
        self.assertIn('"PA12"', syscfg)
        self.assertIn('"PA13"', syscfg)

    def test_nonblocking_bus_uses_mpu6050_gpio(self):
        source = (ROOT / "bsp/bsp_i2c.c").read_text(encoding="utf-8")
        self.assertIn("MPU6050_I2C_SCL_PIN", source)
        self.assertIn("MPU6050_I2C_SDA_PIN", source)
        self.assertNotIn("YBIMU_I2C_", source)
        self.assertNotIn("delay_us", source)
        self.assertNotRegex(source, r"while\s*\(")

    def test_i2c_bsp_is_in_active_build(self):
        cproject = (ROOT / ".cproject").read_text(encoding="utf-8")
        exclusion = cproject[
            cproject.index('excluding="') : cproject.index('" flags=')
        ]
        self.assertNotIn("bsp/bsp_i2c.c", exclusion)

    def test_driver_is_minimal_nonblocking_and_timestamped(self):
        header = (ROOT / "modules/mpu6050/mpu6050.h").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "modules/mpu6050/mpu6050.c").read_text(
            encoding="utf-8"
        )
        config = (ROOT / "modules/mpu6050/mpu6050_config.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "Mpu6050Snapshot",
            "ModuleStatus status",
            "yaw_rate_dps",
            "Mpu6050_Init",
            "Mpu6050_Service",
            "Mpu6050_GetState",
            "Mpu6050_GetSnapshot",
        ):
            self.assertIn(token, header)
        for token in ("0x75U", "0x6BU", "0x1AU", "0x1BU", "0x19U", "0x47U"):
            self.assertIn(token, source)
        for token in (
            "MPU6050_CALIBRATION_MS",
            "MPU6050_FILTER_ALPHA",
            "MPU6050_YAW_SIGN",
            "MPU6050_MAX_CONSECUTIVE_ERRORS",
        ):
            self.assertIn(token, config)
        self.assertNotIn("delay_", source)
        self.assertNotRegex(source, r"while\s*\(")


if __name__ == "__main__":
    unittest.main()
