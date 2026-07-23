"""Canonical encoder for the MSPM0 K230 event protocol."""

VENDOR_ID_MIN = 1
VENDOR_ID_MAX = 17
MAX_FRAME_BYTES = 128
MAX_NUMERIC_FIELDS = 6
MAX_TEXT_BYTES = 47


def _encode_field(value):
    if isinstance(value, bool):
        raise ValueError("boolean fields are not supported")
    if isinstance(value, int):
        if value < -2147483648 or value > 2147483647:
            raise ValueError("integer field outside int32")
        return str(value), True
    if not isinstance(value, str) or not value:
        raise ValueError("field must be a non-empty int or string")
    try:
        encoded = value.encode("ascii")
    except UnicodeError as exc:
        raise ValueError("text field must be ASCII") from exc
    if len(encoded) > MAX_TEXT_BYTES:
        raise ValueError("text field too long")
    if any(ord(char) < 32 or ord(char) > 126 or char in "$#," for char in value):
        raise ValueError("unsafe text field")
    return value, False


def encode_frame(event_id, fields):
    """Return ``$length,ID,payload#\n`` using MCU-compatible limits."""
    if isinstance(event_id, bool) or not isinstance(event_id, int):
        raise ValueError("event_id")
    if not VENDOR_ID_MIN <= event_id <= VENDOR_ID_MAX:
        raise ValueError("event_id")
    if not isinstance(fields, (list, tuple)) or not fields:
        raise ValueError("fields")

    encoded_fields = []
    numeric_count = 0
    text_count = 0
    for value in fields:
        encoded, is_numeric = _encode_field(value)
        if is_numeric:
            numeric_count += 1
        else:
            text_count += 1
        encoded_fields.append(encoded)
    if numeric_count > MAX_NUMERIC_FIELDS or text_count > 1:
        raise ValueError("field count")

    payload = "%02d,%s" % (event_id, ",".join(encoded_fields))
    length = len(payload) + 5
    while True:
        body = "$%d,%s#" % (length, payload)
        new_length = len(body)
        if new_length == length:
            break
        length = new_length
    if len(body) > MAX_FRAME_BYTES:
        raise ValueError("frame too long")
    return body + "\n"
