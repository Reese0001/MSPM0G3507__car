from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MCU = ROOT / "MSPM0G3507_LineFollowing_Car"


def make_frame(event_id: int, fields: list[object]) -> str:
    payload = f"{event_id:02d}," + ",".join(str(value) for value in fields)
    length = len(payload) + 5
    while True:
        frame = f"${length},{payload}#"
        if len(frame) == length:
            return frame + "\n"
        length = len(frame)


class K230ProtocolContract(unittest.TestCase):
    def test_frame_fixture_counts_dollar_through_hash(self):
        frame = make_frame(16, [10, 20, 30, 40, 87, "apple"])
        body = frame.rstrip("\n")
        declared = int(body[1 : body.index(",")])
        self.assertEqual(declared, len(body))
        self.assertTrue(body.startswith("$") and body.endswith("#"))

    def test_incremental_parser_states_and_validation_exist(self):
        header_path = MCU / "modules/k230_link/k230_protocol.h"
        source_path = MCU / "modules/k230_link/k230_protocol.c"
        self.assertTrue(header_path.exists(), header_path)
        self.assertTrue(source_path.exists(), source_path)
        header = header_path.read_text(encoding="utf-8")
        source = source_path.read_text(encoding="utf-8")
        for token in (
            "K230Frame",
            "field_count",
            "fields[6]",
            "text[48]",
            "K230Protocol_Init",
            "K230Protocol_ConsumeByte",
            "K230Protocol_TakeFrame",
        ):
            self.assertIn(token, header)
        for state in ("WAIT_DOLLAR", "READ_FRAME", "WAIT_LF", "DISCARD"):
            self.assertIn(state, source)
        for token in ("declared_length", "frame_length", "parse_frame"):
            self.assertIn(token, source)
        for forbidden in ("strtok", "sscanf", "strtol", "printf"):
            self.assertNotIn(forbidden, source)

    def test_parser_has_restart_overflow_and_id_gates(self):
        source_path = MCU / "modules/k230_link/k230_protocol.c"
        self.assertTrue(source_path.exists(), source_path)
        source = source_path.read_text(encoding="utf-8")
        self.assertIn("byte == '$'", source)
        self.assertIn("frame_length >= K230_FRAME_MAX_LEN", source)
        self.assertIn("K230_Config_IsIdAllowed", source)
        self.assertIn("id_tens", source)
        self.assertIn("id_ones", source)


if __name__ == "__main__":
    unittest.main()
