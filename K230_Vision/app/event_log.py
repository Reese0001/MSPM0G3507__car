"""Optional bounded CSV event log for the K230 TF card."""

try:
    import uos as os
except ImportError:
    import os


class EventLog:
    def __init__(self, enabled, path, max_bytes, max_files):
        self.enabled = bool(enabled)
        self.path = path
        self.max_bytes = int(max_bytes)
        self.max_files = int(max_files)

    def _size(self, path):
        try:
            return os.stat(path)[6]
        except OSError:
            return 0

    def _remove(self, path):
        try:
            os.remove(path)
        except OSError:
            pass

    def _rotate(self):
        if self.max_files <= 1:
            self._remove(self.path)
            return
        self._remove("%s.%d" % (self.path, self.max_files - 1))
        for index in range(self.max_files - 2, 0, -1):
            old_path = "%s.%d" % (self.path, index)
            new_path = "%s.%d" % (self.path, index + 1)
            try:
                os.rename(old_path, new_path)
            except OSError:
                pass
        try:
            os.rename(self.path, self.path + ".1")
        except OSError:
            pass

    @staticmethod
    def _safe(value):
        return str(value).replace(",", ";").replace("\r", " ").replace("\n", " ")

    def append(self, timestamp_ms, event_id, confidence, fields):
        if not self.enabled:
            return
        if self._size(self.path) >= self.max_bytes:
            self._rotate()
        new_file = self._size(self.path) == 0
        with open(self.path, "a") as output:
            if new_file:
                output.write("timestamp_ms,event_id,confidence,fields\n")
            output.write("%d,%d,%d,%s\n" % (
                timestamp_ms,
                event_id,
                confidence,
                self._safe("|".join(str(item) for item in fields)),
            ))
