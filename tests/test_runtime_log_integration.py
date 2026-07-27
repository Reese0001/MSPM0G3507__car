"""运行日志文档与构建入口的集成契约测试。"""

from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
DOCUMENTS = (
    REPO_ROOT / "README.md",
    REPO_ROOT / "MSPM0G3507_LineFollowing_Car" / "README.md",
    REPO_ROOT / "docs" / "setup" / "SETUP_GUIDE.md",
)
REQUIRED_LOG_MARKERS = (
    "RESET",
    "AUTO START",
    "MOTOR CFG",
    "CFG OK",
    "MOTOR ARM",
    "TX L030 R030",
    "rebuild",
    "架空轮",
)


class RuntimeLogDocumentationIntegrationTests(unittest.TestCase):
    def test_all_user_documents_describe_reset_startup_log_and_wheel_lift(self):
        for document in DOCUMENTS:
            content = document.read_text(encoding="utf-8")
            for marker in REQUIRED_LOG_MARKERS:
                with self.subTest(document=document, marker=marker):
                    self.assertIn(marker, content)

    def test_makefile_includes_runtime_log_source_once(self):
        makefile = REPO_ROOT / "MSPM0G3507_LineFollowing_Car" / "Makefile"
        content = makefile.read_text(encoding="utf-8")
        self.assertEqual(content.count("modules/display/runtime_log.c"), 1)


if __name__ == "__main__":
    unittest.main()
