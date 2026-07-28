"""运行日志文档与构建入口的集成契约测试。"""

from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
DOCUMENTS = (
    REPO_ROOT / "README.md",
    REPO_ROOT / "MSPM0G3507_LineFollowing_Car" / "README.md",
    REPO_ROOT / "docs" / "setup" / "SETUP_GUIDE.md",
)
STARTUP_LOG = """0000 BOOT
0012 OLED OK
0022 MOTOR CFG
0525 CFG OK
PRESS K1
.... MOTOR ARMED
.... SAFETY RUN
.... CONTROL REQ
.... TX Lxxx Rxxx"""
REQUIRED_DOCUMENTATION_MARKERS = (
    "RESET",
    "rebuild",
    "架空轮",
)


class RuntimeLogDocumentationIntegrationTests(unittest.TestCase):
    def test_all_user_documents_describe_reset_startup_log_and_wheel_lift(self):
        for document in DOCUMENTS:
            content = document.read_text(encoding="utf-8")
            for log_line in STARTUP_LOG.splitlines():
                with self.subTest(document=document, marker=log_line):
                    self.assertIn(log_line, content)
            for marker in REQUIRED_DOCUMENTATION_MARKERS:
                with self.subTest(document=document, marker=marker):
                    self.assertIn(marker, content)

    def test_makefile_includes_runtime_log_source_once(self):
        makefile = REPO_ROOT / "MSPM0G3507_LineFollowing_Car" / "Makefile"
        content = makefile.read_text(encoding="utf-8")
        self.assertEqual(content.count("modules/display/runtime_log.c"), 1)


if __name__ == "__main__":
    unittest.main()
