from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MSPM0G3507_LineFollowing_Car"


class RuntimeLogRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        vsdevcmd = (
            Path(os.environ["ProgramFiles"])
            / "Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
        )
        cls.assertTrue = unittest.TestCase.assertTrue
        cls.assertTrue(cls, vsdevcmd.exists(), "Visual Studio host toolchain missing")
        cls.temp_dir = tempfile.TemporaryDirectory()
        cls.exe = Path(cls.temp_dir.name) / "runtime_log_harness.exe"
        harness = ROOT / "tests/runtime_log_harness.c"
        source = PROJECT / "modules/display/runtime_log.c"
        command = (
            f'call "{vsdevcmd}" -arch=x64 >nul && '
            f'cl /nologo /std:c11 /utf-8 /W4 /WX /TC /I"{PROJECT}" '
            f'"{harness}" "{source}" /Fe"{cls.exe}"'
        )
        result = subprocess.run(
            command,
            cwd=cls.temp_dir.name,
            capture_output=True,
            text=True,
            errors="replace",
            check=False,
            shell=True,
            executable=os.environ["ComSpec"],
        )
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)

    @classmethod
    def tearDownClass(cls):
        cls.temp_dir.cleanup()

    def run_harness(self):
        result = subprocess.run([str(self.exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout

    def test_ring_order_and_capacity(self):
        result = subprocess.run([str(self.exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("0000 BOOT", result.stdout)
        self.assertIn("0007 E7", result.stdout)
        self.assertNotIn("0000 E0", result.stdout)

    def test_duplicate_payload_is_suppressed(self):
        self.assertIn("DEDUP_OK", self.run_harness())

    def test_long_duplicate_payload_is_suppressed(self):
        self.assertIn("LONG_DEDUP_OK", self.run_harness())

    def test_ascii_rejection_and_snapshot_bounds(self):
        output = self.run_harness()
        self.assertIn("ASCII_REJECT_OK", output)
        self.assertIn("TRUNCATE_AND_EMPTY_OK", output)


if __name__ == "__main__":
    unittest.main()
