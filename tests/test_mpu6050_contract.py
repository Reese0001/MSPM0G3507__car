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


if __name__ == "__main__":
    unittest.main()
