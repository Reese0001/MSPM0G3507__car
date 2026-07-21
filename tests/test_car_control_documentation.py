"""Contract checks for operator-facing car-control documentation."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
DOCS = (
    ROOT / "README.md",
    ROOT / "MSPM0G3507_LineFollowing_Car" / "README.md",
    ROOT / "docs" / "setup" / "SETUP_GUIDE.md",
    ROOT / "docs" / "setup" / "CAR_CONTROL_TEST_MATRIX.md",
)


class CarControlDocumentationTests(unittest.TestCase):
    def test_operator_documents_are_clean_utf8_without_bom(self):
        mojibake_markers = ("\ufffd", "閿", "锟", "鈫", "绾", "鐢")

        for path in DOCS:
            with self.subTest(path=path.relative_to(ROOT)):
                raw = path.read_bytes()
                self.assertFalse(raw.startswith(b"\xef\xbb\xbf"))
                text = raw.decode("utf-8")
                for marker in mojibake_markers:
                    self.assertNotIn(marker, text)

    def test_entrypoint_and_route_integration_status_are_explicit(self):
        for path in DOCS[:3]:
            text = path.read_text(encoding="utf-8")
            with self.subTest(path=path.relative_to(ROOT)):
                self.assertIn("LineWalking()", text)
                self.assertIn("CarRoute", text)
                self.assertIn("尚未接入 empty.c", text)

    def test_hardware_and_data_freshness_contracts_are_documented(self):
        combined = "\n".join(path.read_text(encoding="utf-8") for path in DOCS)
        required = (
            "M2 左轮",
            "M4 右轮",
            "M1/M3",
            "PA15/PA16/PA17",
            "PA18",
            "PA12/PA13",
            "软件 I2C",
            "12.6 V",
            "5-12 V",
            "未购买摄像头",
            ">= 200 ms",
            "没有时间戳或序列号",
            "CarSensor_ReadFrame",
        )
        for value in required:
            with self.subTest(value=value):
                self.assertIn(value, combined)

    def test_matrix_records_stages_and_unverified_boundaries(self):
        text = (DOCS[3]).read_text(encoding="utf-8")
        required = (
            "Python 静态/单元测试",
            "TI Arm Clang 翻译单元编译",
            "CCS 完整构建",
            "架空轮",
            "低电压",
            "低速封闭赛道",
            "禁止接入 12.6 V 电机电源",
            "未验证",
            "待执行",
        )
        for value in required:
            with self.subTest(value=value):
                self.assertIn(value, text)


if __name__ == "__main__":
    unittest.main()
