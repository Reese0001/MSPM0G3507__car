"""Minimal fail-closed CanMV entry point for the K230 luxury module."""

import time

from K230_Vision.app.event_log import EventLog
from K230_Vision.config.vision_config import (
    ALLOWED_EVENT_IDS,
    LOG_ENABLED,
    LOG_MAX_BYTES,
    LOG_MAX_FILES,
    LOG_PATH,
    UART_BAUDRATE,
)
from K230_Vision.protocol.frame import encode_frame


def process_vision_frame():
    """Return (event_id, confidence, extra_fields), or None.

    The contest-specific detector is intentionally not guessed here. No result
    means no UART event, so a camera failure cannot create a valid MCU command.
    """
    return None


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000)


def run():
    from ybUtils.YbUart import YbUart

    uart = YbUart(baudrate=UART_BAUDRATE)
    event_log = EventLog(LOG_ENABLED, LOG_PATH, LOG_MAX_BYTES, LOG_MAX_FILES)
    while True:
        try:
            result = process_vision_frame()
            if result is None:
                time.sleep_ms(10) if hasattr(time, "sleep_ms") else time.sleep(0.01)
                continue
            event_id, confidence, extra_fields = result
            if event_id not in ALLOWED_EVENT_IDS or not 0 <= confidence <= 100:
                continue
            fields = [confidence] + list(extra_fields)
            uart.write(encode_frame(event_id, fields).encode("ascii"))
            event_log.append(_ticks_ms(), event_id, confidence, extra_fields)
        except Exception as exc:
            # Local diagnostics only: malformed results never become UART frames.
            print("K230 vision error:", exc)


if __name__ == "__main__":
    run()
