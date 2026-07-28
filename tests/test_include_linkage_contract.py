from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class IncludeLinkageContract(unittest.TestCase):
    def test_moved_motor_configuration_header_has_no_stale_include_name(self):
        stale = []
        for path in PROJECT.rglob("*"):
            if not path.is_file() or "Debug" in path.parts or "Build_LineFollowing" in path.parts:
                continue
            if path.suffix.lower() not in {".c", ".h"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            if '#include "app_motor.h"' in text:
                stale.append(path.relative_to(PROJECT).as_posix())

        self.assertEqual([], stale)


if __name__ == "__main__":
    unittest.main()
