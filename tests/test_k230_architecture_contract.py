from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MCU = ROOT / "MSPM0G3507_LineFollowing_Car"


class K230ArchitectureContract(unittest.TestCase):
    def test_protocol_limits_exist(self):
        path = MCU / "modules/optional/k230/k230_config.h"
        self.assertTrue(path.exists(), path)
        text = path.read_text(encoding="utf-8")
        self.assertIn("K230_FRAME_MAX_LEN", text)
        self.assertIn("128U", text)
        self.assertIn("K230_VISION_STALE_MS", text)
        self.assertIn("300U", text)
        self.assertIn("K230_ALLOWED_EVENT_ID", text)
        self.assertIn("16U", text)

    def test_link_cannot_include_motor_or_allocate(self):
        directory = MCU / "modules/optional/k230"
        self.assertTrue(directory.is_dir(), directory)
        for path in directory.glob("*.[ch]"):
            text = path.read_text(encoding="utf-8").lower()
            self.assertNotIn("motor", text, path)
            self.assertNotIn("malloc", text, path)
            self.assertNotIn("calloc", text, path)
            self.assertNotIn("realloc", text, path)

    def test_snapshot_has_status_confidence_and_timestamp(self):
        header = (MCU / "modules/optional/k230/k230_link.h").read_text(
            encoding="utf-8"
        )
        for token in (
            "K230VisionSnapshot",
            "ModuleStatus status",
            "confidence",
            "event_id",
            "K230Link_Service",
            "K230Link_GetSnapshot",
            "rejected_frames",
        ):
            self.assertIn(token, header)

    def test_uart_isr_only_queues_bytes(self):
        source = (MCU / "modules/optional/k230/k230_link.c").read_text(
            encoding="utf-8"
        )
        start = source.index("void K230Link_OnRxByteFromISR")
        end = source.index("void K230Link_Service", start)
        isr_body = source[start:end]
        self.assertIn("rx_buffer", isr_body)
        for forbidden in (
            "K230Protocol_ConsumeByte",
            "K230Protocol_TakeFrame",
            "printf",
            "strtok",
            "sscanf",
        ):
            self.assertNotIn(forbidden, isr_body)

    def test_snapshot_is_age_checked(self):
        source = (MCU / "modules/optional/k230/k230_link.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("ModuleStatus_IsFresh", source)
        self.assertIn("K230_VISION_STALE_MS", source)

    def test_uart2_bsp_drains_rx_fifo_through_irq_bridge(self):
        source = (MCU / "bsp/bsp_k230_uart.c").read_text(encoding="utf-8")
        for token in (
            "ti_msp_dl_config.h",
            "DL_UART_Main_isRXFIFOEmpty",
            "DL_UART_Main_receiveData",
            "K230_INST_IRQHandler",
            "K230_UART_IRQHandler",
        ):
            self.assertIn(token, source)
        self.assertNotIn("receiveDataBlocking", source)


if __name__ == "__main__":
    unittest.main()
