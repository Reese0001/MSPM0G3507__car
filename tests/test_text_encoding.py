from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
TEXT_SUFFIXES = {
    ".c", ".h", ".md", ".txt", ".syscfg", ".project", ".cproject",
    ".ccsproject", ".prefs", ".clangd", ".csv", ".cmd", ".mk", ".json",
    ".xml",
}
SPECIAL_NAMES = {".project", ".cproject", ".ccsproject", ".clangd"}


class TextEncodingTests(unittest.TestCase):
    def test_maintained_text_is_valid_utf8(self):
        roots = [ROOT / "MSPM0G3507_LineFollowing_Car", ROOT / "docs"]
        invalid = []
        damaged = []
        for base in roots:
            for path in base.rglob("*"):
                if not path.is_file():
                    continue
                if path.suffix.lower() not in TEXT_SUFFIXES and path.name not in SPECIAL_NAMES:
                    continue
                try:
                    text = path.read_text(encoding="utf-8")
                    if "\ufffd" in text:
                        damaged.append(str(path.relative_to(ROOT)))
                except UnicodeDecodeError:
                    invalid.append(str(path.relative_to(ROOT)))
        self.assertEqual([], invalid, "Non-UTF-8 maintained text:\n" + "\n".join(invalid))
        self.assertEqual([], damaged, "Damaged UTF-8 text:\n" + "\n".join(damaged))


if __name__ == "__main__":
    unittest.main()
