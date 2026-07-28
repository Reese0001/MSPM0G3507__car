from pathlib import Path
import tempfile
import unittest

from K230_Vision.app.event_log import EventLog
from K230_Vision.protocol.frame import encode_frame


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
        header_path = MCU / "modules/optional/k230/k230_protocol.h"
        source_path = MCU / "modules/optional/k230/k230_protocol.c"
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
        source_path = MCU / "modules/optional/k230/k230_protocol.c"
        self.assertTrue(source_path.exists(), source_path)
        source = source_path.read_text(encoding="utf-8")
        self.assertIn("byte == '$'", source)
        self.assertIn("frame_length >= K230_FRAME_MAX_LEN", source)
        self.assertIn("K230_Config_IsIdAllowed", source)
        self.assertIn("id_tens", source)
        self.assertIn("id_ones", source)

    def test_python_encoder_matches_mcu_length_rule(self):
        frame = encode_frame(16, [87, "apple"])
        body = frame.rstrip("\n")
        declared = int(body[1:body.index(",")])
        self.assertEqual(declared, len(body.encode("ascii")))
        self.assertTrue(body.startswith("$") and body.endswith("#"))
        self.assertEqual(frame, frame.encode("ascii").decode("ascii"))

    def test_python_encoder_rejects_unsafe_fields(self):
        for value in ("a,b", "bad#field", "line\nfeed", "中文"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    encode_frame(16, [80, value])
        with self.assertRaises(ValueError):
            encode_frame(18, [80])

    def test_canmv_entry_uses_vendor_uart_and_bounded_log(self):
        main = (ROOT / "K230_Vision/main.py").read_text(encoding="utf-8")
        config = (ROOT / "K230_Vision/config/vision_config.py").read_text(
            encoding="utf-8"
        )
        event_log = (ROOT / "K230_Vision/app/event_log.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("from ybUtils.YbUart import YbUart", main)
        self.assertIn("baudrate=UART_BAUDRATE", main)
        self.assertIn("encode_frame", main)
        self.assertIn("LOG_ENABLED = False", config)
        self.assertIn("LOG_MAX_BYTES = 1024 * 1024", config)
        self.assertIn("LOG_MAX_FILES = 20", config)

    def test_event_log_is_disabled_or_rotates_to_file_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "events.csv")
            EventLog(False, path, 1, 3).append(1, 16, 80, ["ignored"])
            self.assertFalse(Path(path).exists())

            logger = EventLog(True, path, 1, 3)
            for index in range(8):
                logger.append(index, 16, 80, ["apple"])
            files = list(Path(directory).glob("events.csv*"))
            self.assertLessEqual(len(files), 3)
            self.assertTrue(Path(path).exists())


if __name__ == "__main__":
    unittest.main()
