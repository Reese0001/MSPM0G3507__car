from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1] / "MSPM0G3507_LineFollowing_Car"


class ModularArchitectureContract(unittest.TestCase):
    def test_module_status_contract_exists(self):
        text = (ROOT / "modules/common/module_status.h").read_text(encoding="utf-8")
        for token in ("timestamp_ms", "sequence", "valid", "health", "ModuleStatus_IsFresh"):
            self.assertIn(token, text)

    def test_required_roots_exist(self):
        for name in ("application", "modules", "bsp"):
            self.assertTrue((ROOT / name).is_dir(), name)

    def test_legacy_bsp_is_removed(self):
        directory_names = {path.name for path in ROOT.iterdir() if path.is_dir()}
        self.assertNotIn("BSP", directory_names)

    def test_lower_layers_do_not_include_application(self):
        for base in (ROOT / "modules", ROOT / "bsp"):
            for path in base.rglob("*.[ch]"):
                text = path.read_text(encoding="utf-8", errors="ignore")
                self.assertNotIn('#include "application/', text, str(path))

    def test_lower_layers_do_not_include_application_only_headers(self):
        application = ROOT / "application"
        application_headers = {
            path.name for path in application.rglob("*.h")
        }
        for base in (ROOT / "modules", ROOT / "bsp"):
            for path in base.rglob("*.[ch]"):
                text = path.read_text(encoding="utf-8", errors="ignore")
                for header in re.findall(r'^\s*#include\s+"([^/"]+)"', text, re.MULTILINE):
                    if header not in application_headers:
                        continue
                    matches = list(ROOT.rglob(header))
                    self.assertTrue(matches, header)
                    self.assertFalse(
                        all(candidate.is_relative_to(application) for candidate in matches),
                        f"{path} includes application-only header {header}",
                    )

    def test_application_owns_legacy_key_event_policy(self):
        key_source = (ROOT / "modules/key/key.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        questions_source = (ROOT / "application/legacy_questions/questions.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        task_source = (ROOT / "application/legacy_task/task.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        self.assertIn("Key_PollEvent", key_source)
        self.assertNotIn("State_Machine", key_source)
        self.assertIn("Legacy_Questions_HandleKey", questions_source)
        self.assertIn("Key_PollEvent", questions_source)
        self.assertIn("Legacy_Questions_HandleKey", task_source)


if __name__ == "__main__":
    unittest.main()
