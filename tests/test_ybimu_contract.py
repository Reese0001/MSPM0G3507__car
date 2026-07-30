from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class Mpu6050Contract(unittest.TestCase):
    def test_active_profile_builds_mpu6050_nonblocking_driver(self):
        active = (
            (ROOT / "modules/line_tracking/line_follower.c").read_text(encoding="utf-8")
            + (ROOT / "app/tasks/app_tasks.c").read_text(encoding="utf-8")
        )
        makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
        self.assertIn("Mpu6050_Service", active)
        self.assertIn("BSP_I2C_Service", active)
        self.assertNotIn("YbImu_", active)
        self.assertIn("modules/imu/mpu6050.c", makefile)
        self.assertNotIn("modules/optional/ybimu/ybimu.c", makefile)

    def test_driver_uses_original_mpu6050_address_and_gyro_registers(self):
        header = (ROOT / "modules/imu/mpu6050.h").read_text(encoding="utf-8")
        source = (ROOT / "modules/imu/mpu6050.c").read_text(encoding="utf-8")
        for token in (
            "MPU6050_ADDRESS (0x68U)",
            "MPU6050_PWR_MGMT_1 (0x6BU)",
            "MPU6050_GYRO_XOUT_H (0x43U)",
            "MPU6050_GYRO_ZOUT_H (0x47U)",
            "decode_i16_be",
            "Mpu6050_Init",
            "Mpu6050_Service",
            "Mpu6050_GetSnapshot",
        ):
            self.assertIn(token, source)
        self.assertIn("MPU6050_SAMPLE_PERIOD_MS (10U)", header)
        self.assertIn("MPU6050_STALE_TIMEOUT_MS (50U)", header)
        self.assertIn("MPU6050_GYRO_Z_SIGN (1.0f)", header)
        self.assertIn("MPU6050_GYRO_Y_SIGN (1.0f)", header)
        self.assertIn("MPU6050_GYRO_Z_SIGN", source)
        self.assertIn("MPU6050_GYRO_Y_SIGN", source)
        self.assertNotIn("delay_ms", source)
        self.assertNotRegex(source, r"while\s*\(")

    def test_bsp_i2c_is_nonblocking_open_drain_state_machine(self):
        combined = (
            (ROOT / "bsp/bsp_i2c.h").read_text(encoding="utf-8")
            + (ROOT / "bsp/bsp_i2c.c").read_text(encoding="utf-8")
        )
        for token in (
            "BSP_I2C_Init",
            "BSP_I2C_BeginRead",
            "BSP_I2C_BeginWrite",
            "BSP_I2C_Service",
            "BSP_I2C_GetStatus",
            "MPU6050_I2C_SCL_PIN",
            "MPU6050_I2C_SDA_PIN",
            "DL_GPIO_disableOutput",
            "BSP_I2C_TRANSACTION_TIMEOUT_US",
        ):
            self.assertIn(token, combined)
        self.assertNotIn("delay_us", combined)
        self.assertNotRegex(combined, r"while\s*\(")


if __name__ == "__main__":
    unittest.main()
