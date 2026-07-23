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


if __name__ == "__main__":
    unittest.main()
