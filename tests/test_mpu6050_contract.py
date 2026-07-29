from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class Mpu6050Contract(unittest.TestCase):
    def test_sysconfig_assigns_active_ybimu_bus(self):
        syscfg = (ROOT / "empty.syscfg").read_text(encoding="utf-8")
        self.assertIn("GPIO5.$name", syscfg)
        self.assertIn('"YBIMU_I2C"', syscfg)
        self.assertIn('"PA1"', syscfg)
        self.assertIn('"PA0"', syscfg)

    def test_nonblocking_bus_uses_active_ybimu_gpio(self):
        source = (ROOT / "bsp/bsp_i2c.c").read_text(encoding="utf-8")
        self.assertIn("YBIMU_I2C_SCL_PIN", source)
        self.assertIn("YBIMU_I2C_SDA_PIN", source)
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
        kalman = (ROOT / "modules/mpu6050/mpu6050_kalman.h").read_text(
            encoding="utf-8"
        )
        config = (ROOT / "modules/mpu6050/mpu6050_config.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "Mpu6050Snapshot",
            "ModuleStatus status",
            "yaw_rate_dps",
            "yaw_angle_deg",
            "Mpu6050Kalman",
            "Mpu6050_Init",
            "Mpu6050_Service",
            "Mpu6050_GetState",
            "Mpu6050_GetSnapshot",
        ):
            self.assertIn(token, header + kalman)
        for token in ("0x75U", "0x6BU", "0x1AU", "0x1BU", "0x19U", "0x47U"):
            self.assertIn(token, source)
        for token in (
            "MPU6050_CALIBRATION_MS",
            "MPU6050_FILTER_ALPHA",
            "MPU6050_YAW_SIGN",
            "MPU6050_MAX_CONSECUTIVE_ERRORS",
            "MPU6050_ANGLE_RESET",
            "MPU6050_KALMAN_MAX_DT_MS",
        ):
            self.assertIn(token, config)
        self.assertIn("last_processed_sample_ms", source)
        self.assertIn("now_ms - last_processed_sample_ms", source)
        begin_read = source[
            source.index("static bool begin_gyro_read"):
            source.index("void Mpu6050_Init")
        ]
        self.assertNotIn("last_processed_sample_ms =", begin_read)
        self.assertNotIn("delay_", source)
        self.assertNotRegex(source, r"while\s*\(")


if __name__ == "__main__":
    unittest.main()
