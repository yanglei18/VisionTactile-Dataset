"""One UTF-8 JSON object per LF-delimited frame, bounded to 2 MiB including LF.

decode_message validates one already-framed message, not an arbitrary stream.
Socket readers must stop buffering at MAX_MESSAGE_BYTES. No control commands
are supported in v1; capability flags never grant a command implicitly.
"""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
import json

from .validation import CONTROL_PROTOCOL_VERSION, DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION, MAX_MESSAGE_BYTES, PROTOCOL_VERSION, ProtocolError, validate_message


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _constant(value):
    raise ProtocolError(f"non-finite JSON constant: {value}")


def encode_message(message) -> bytes:
    """Validate and encode a dict or dataclass; return exactly one trailing LF."""
    try:
        if is_dataclass(message) and not isinstance(message, type):
            converter = getattr(message, "to_dict", None)
            message = converter() if converter is not None else asdict(message)
        validate_message(message)
        payload = json.dumps(message, ensure_ascii=False, allow_nan=False,
                             separators=(",", ":")).encode("utf-8") + b"\n"
    except (TypeError, ValueError, OverflowError, RecursionError) as error:
        if isinstance(error, ProtocolError):
            raise
        raise ProtocolError(f"cannot encode protocol message: {error}") from error
    if len(payload) > MAX_MESSAGE_BYTES:
        raise ProtocolError(f"message exceeds {MAX_MESSAGE_BYTES} bytes")
    return payload


def decode_message(payload: bytes | str) -> dict:
    """Accept a single frame with optional LF/CRLF, rejecting concatenated frames."""
    if type(payload) not in (bytes, str):
        raise ProtocolError("payload must be bytes or str")
    try:
        raw = payload.encode("utf-8", errors="strict") if isinstance(payload, str) else payload
        if len(raw) > MAX_MESSAGE_BYTES:
            raise ProtocolError(f"message exceeds {MAX_MESSAGE_BYTES} bytes")
        text = raw.decode("utf-8", errors="strict")
        if text.endswith("\n"):
            text = text[:-1]
            if text.endswith("\r"):
                text = text[:-1]
        if "\n" in text or "\r" in text:
            raise ProtocolError("expected one JSON line, not multiple lines or frames")
        message = json.loads(text, parse_constant=_constant, object_pairs_hook=_pairs)
        return validate_message(message)
    except (UnicodeError, TypeError, ValueError, OverflowError, RecursionError) as error:
        if isinstance(error, ProtocolError):
            raise
        raise ProtocolError(f"cannot decode protocol message: {error}") from error


__all__ = ["PROTOCOL_VERSION", "CONTROL_PROTOCOL_VERSION", "DETAIL_PROTOCOL_VERSION", "DEVICE_PROTOCOL_VERSION", "MAX_MESSAGE_BYTES", "ProtocolError", "encode_message", "decode_message"]
