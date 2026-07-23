from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class YbImuContract(unittest.TestCase):
    def test_vendor_registers_and_address(self):
        header_path = ROOT / "modules/ybimu/ybimu_protocol.h"
        self.assertTrue(header_path.exists(), header_path)
        text = header_path.read_text(encoding="utf-8")
        for token in ("0x23U", "0x0AU", "0x10U", "0x16U", "0x26U"):
            self.assertIn(token, text)
        self.assertIn("YbImuProtocol_DecodeI16LE", text)
        self.assertIn("YbImuProtocol_DecodeFloatLE", text)

    def test_decoders_use_explicit_little_endian_assembly(self):
        source_path = ROOT / "modules/ybimu/ybimu_protocol.c"
        self.assertTrue(source_path.exists(), source_path)
        source = source_path.read_text(encoding="utf-8")
        for shift in ("<< 8", "<< 16", "<< 24"):
            self.assertIn(shift, source)
        self.assertIn("memcpy(&value, &raw, sizeof(value))", source)
        self.assertNotIn("(float *)", source)
        self.assertNotIn("*(float", source)

    def test_snapshot_is_timestamped_and_nonblocking(self):
        header_path = ROOT / "modules/ybimu/ybimu.h"
        source_path = ROOT / "modules/ybimu/ybimu.c"
        config_path = ROOT / "modules/ybimu/ybimu_config.h"
        self.assertTrue(header_path.exists(), header_path)
        self.assertTrue(source_path.exists(), source_path)
        self.assertTrue(config_path.exists(), config_path)
        header = header_path.read_text(encoding="utf-8")
        source = source_path.read_text(encoding="utf-8")
        config = config_path.read_text(encoding="utf-8")
        for token in (
            "YbImuSnapshot",
            "ModuleStatus status",
            "gyro_rad_s",
            "euler_deg",
            "quat",
            "mag_uT",
            "magnetic_heading_healthy",
            "YbImu_Init",
            "YbImu_Service",
            "YbImu_GetSnapshot",
        ):
            self.assertIn(token, header)
        self.assertIn("YBIMU_SAMPLE_PERIOD_MS", config)
        self.assertIn("10U", config)
        self.assertNotIn("delay_ms", source)
        self.assertNotRegex(source, r"while\s*\(")

    def test_service_advances_one_vendor_register_per_call(self):
        source_path = ROOT / "modules/ybimu/ybimu.c"
        self.assertTrue(source_path.exists(), source_path)
        source = source_path.read_text(encoding="utf-8")
        for token in (
            "YBIMU_REG_GYRO",
            "YBIMU_REG_MAG",
            "YBIMU_REG_QUAT",
            "YBIMU_REG_EULER",
            "publish_complete_group",
        ):
            self.assertIn(token, source)
        service = source[source.index("void YbImu_Service"):]
        self.assertEqual(1, service.count("BSP_I2C_BeginRead("))
        self.assertIn("read_index++", service)
        self.assertIn("working.status.timestamp_ms = now_ms", source)

    def test_bsp_i2c_is_nonblocking_open_drain_state_machine(self):
        header_path = ROOT / "bsp/bsp_i2c.h"
        source_path = ROOT / "bsp/bsp_i2c.c"
        self.assertTrue(header_path.exists(), header_path)
        self.assertTrue(source_path.exists(), source_path)
        combined = header_path.read_text(encoding="utf-8") + source_path.read_text(
            encoding="utf-8"
        )
        for token in (
            "BSP_I2C_Init",
            "BSP_I2C_BeginRead",
            "BSP_I2C_BeginWrite",
            "BSP_I2C_Service",
            "BSP_I2C_GetStatus",
            "BSP_I2C_MAX_TRANSFER",
        ):
            self.assertIn(token, combined)
        self.assertIn("length > BSP_I2C_MAX_TRANSFER", combined)
        for token in (
            "ti_msp_dl_config.h",
            "YBIMU_I2C_SCL_PIN",
            "YBIMU_I2C_SDA_PIN",
            "DL_GPIO_disableOutput",
            "BSP_I2C_STATUS_BUSY",
            "BSP_I2C_TRANSACTION_TIMEOUT_US",
        ):
            self.assertIn(token, combined)
        self.assertNotIn("delay_us", combined)
        self.assertNotRegex(combined, r"while\s*\(")

    def test_active_path_does_not_use_legacy_mpu(self):
        source_path = ROOT / "modules/ybimu/ybimu.c"
        self.assertTrue(source_path.exists(), source_path)
        source = source_path.read_text(encoding="utf-8")
        scheduler = (ROOT / "application/app_scheduler.c").read_text(encoding="utf-8")
        for forbidden in ("app_mpu6050", "Get_EulerAngles", "mpu_dmp"):
            self.assertNotIn(forbidden, source + scheduler)

    def test_calibration_has_timeout_states(self):
        header = (ROOT / "modules/ybimu/ybimu.h").read_text(encoding="utf-8")
        source = (ROOT / "modules/ybimu/ybimu.c").read_text(encoding="utf-8")
        config = (ROOT / "modules/ybimu/ybimu_config.h").read_text(
            encoding="utf-8"
        )
        protocol = (ROOT / "modules/ybimu/ybimu_protocol.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "YBIMU_CAL_IDLE",
            "YBIMU_CAL_RUNNING",
            "YBIMU_CAL_SUCCESS",
            "YBIMU_CAL_FAILED",
            "YbImu_RequestCalibration",
            "YbImu_CancelCalibration",
            "YbImu_GetCalibrationState",
        ):
            self.assertIn(token, header)
        for token in ("0x70U", "0x71U"):
            self.assertIn(token, protocol)
        for token in ("100U", "7000U", "60000U"):
            self.assertIn(token, config)
        self.assertNotIn("delay_ms", source)
        self.assertNotRegex(source, r"while\s*\(")
        self.assertNotIn("Motor_", source)

    def test_calibration_write_and_poll_are_bounded(self):
        source = (ROOT / "modules/ybimu/ybimu.c").read_text(encoding="utf-8")
        bsp = (ROOT / "bsp/bsp_i2c.h").read_text(encoding="utf-8") + (
            ROOT / "bsp/bsp_i2c.c"
        ).read_text(encoding="utf-8")
        self.assertIn("BSP_I2C_BeginWrite", source)
        self.assertIn("BSP_I2C_BeginWrite", bsp)
        self.assertIn("length > BSP_I2C_MAX_TRANSFER", bsp)
        self.assertIn("calibration_value = 0x01U", source)
        self.assertIn("service_calibration", source)

    def test_minimal_burn_profile_does_not_service_unfitted_imu(self):
        scheduler = (ROOT / "application/app_scheduler.c").read_text(
            encoding="utf-8"
        )
        profile = (ROOT / "application/config/line_following_profile.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("LINE_FOLLOWING_USE_IMU (0)", profile)
        self.assertNotIn("BSP_I2C_Service", scheduler)
        self.assertNotIn("YbImu_Service", scheduler)

    def test_magnetic_heading_has_plausibility_and_change_gates(self):
        source = (ROOT / "modules/ybimu/ybimu.c").read_text(encoding="utf-8")
        config = (ROOT / "modules/ybimu/ybimu_config.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "YBIMU_MAG_MIN_UT",
            "YBIMU_MAG_MAX_UT",
            "YBIMU_MAG_NORM_SQ_DELTA_MAX",
        ):
            self.assertIn(token, config)
        self.assertIn("update_magnetic_health", source)
        self.assertIn("magnetic_heading_healthy", source)

    def test_calibration_checklist_covers_bench_gates(self):
        path = ROOT.parents[0] / "docs/hardware/ybimu-calibration-checklist.md"
        self.assertTrue(path.exists(), path)
        text = path.read_text(encoding="utf-8")
        for token in (
            "PA12",
            "PA13",
            "0x23",
            "60s",
            "X/Y/Z",
            "yaw",
            "8 字",
            "100Hz",
            "电机断电",
        ):
            self.assertIn(token, text)


if __name__ == "__main__":
    unittest.main()
