from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MCU = ROOT / "MSPM0G3507_LineFollowing_Car"


class K230ArchitectureContract(unittest.TestCase):
    def test_protocol_limits_exist(self):
        path = MCU / "modules/k230_link/k230_config.h"
        self.assertTrue(path.exists(), path)
        text = path.read_text(encoding="utf-8")
        self.assertIn("K230_FRAME_MAX_LEN", text)
        self.assertIn("128U", text)
        self.assertIn("K230_VISION_STALE_MS", text)
        self.assertIn("300U", text)
        self.assertIn("K230_ALLOWED_EVENT_ID", text)
        self.assertIn("16U", text)

    def test_link_cannot_include_motor_or_allocate(self):
        directory = MCU / "modules/k230_link"
        self.assertTrue(directory.is_dir(), directory)
        for path in directory.glob("*.[ch]"):
            text = path.read_text(encoding="utf-8").lower()
            self.assertNotIn("motor", text, path)
            self.assertNotIn("malloc", text, path)
            self.assertNotIn("calloc", text, path)
            self.assertNotIn("realloc", text, path)


if __name__ == "__main__":
    unittest.main()
