from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class HostBuildCleanlinessContract(unittest.TestCase):
    def test_msvc_harness_builds_run_inside_temporary_directories(self):
        offenders = []
        for path in (ROOT / "tests").glob("test_*.py"):
            text = path.read_text(encoding="utf-8")
            if ("cl /nologo" in text or "cl.exe /nologo" in text) and "cwd=" not in text:
                offenders.append(path.name)

        self.assertEqual([], offenders)


if __name__ == "__main__":
    unittest.main()
