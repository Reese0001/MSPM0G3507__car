UART_BAUDRATE = 115200

# Only the agreed protocol self-test/classification event is enabled initially.
ALLOWED_EVENT_IDS = (16,)

# Logging is opt-in to protect the 64GB TF card from unnecessary writes.
LOG_ENABLED = False
LOG_PATH = "/sdcard/k230_events.csv"
LOG_MAX_BYTES = 1024 * 1024
LOG_MAX_FILES = 20
