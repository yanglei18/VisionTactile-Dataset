"""Bounded JSON transport for the local hand service; standard library only.

Each connection carries one length-prefixed UTF-8 JSON request and response.
The caller owns retries: a transport timeout never submits another command.
"""
from __future__ import annotations

import json
import math
import socket
import struct
import time


SCHEMA_VERSION = 1
MAX_REQUEST_BYTES = 64 * 1024
MAX_RESPONSE_BYTES = 4 * 1024 * 1024
_PREFIX = struct.Struct("!I")


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON field: {key}")
        result[key] = value
    return result


def _invalid_constant(value):
    raise ValueError(f"Nonfinite JSON value: {value}")


def encode(document, limit):
    payload = json.dumps(document, ensure_ascii=False, allow_nan=False,
                         separators=(",", ":")).encode("utf-8")
    if not 0 < len(payload) <= limit:
        raise ValueError("Hand service message exceeds its size limit")
    return _PREFIX.pack(len(payload)) + payload


def _read_exact(connection, count, deadline):
    parts = bytearray()
    while len(parts) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Hand service message deadline exceeded")
        connection.settimeout(remaining)
        chunk = connection.recv(count - len(parts))
        if not chunk:
            raise ConnectionError("Hand service closed an incomplete message")
        parts.extend(chunk)
    return bytes(parts)


def receive(connection, limit):
    deadline = time.monotonic() + (connection.gettimeout() or 1.)
    length, = _PREFIX.unpack(_read_exact(connection, _PREFIX.size, deadline))
    if not 0 < length <= limit:
        raise ValueError("Hand service message exceeds its size limit")
    document = json.loads(_read_exact(connection, length, deadline).decode("utf-8"),
                          object_pairs_hook=_unique_pairs, parse_constant=_invalid_constant)
    if not isinstance(document, dict):
        raise ValueError("Hand service message must be an object")
    return document


def request(socket_path, document, *, timeout=1.0):
    """Perform exactly one request, with a bounded socket wait and no retries."""
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 0 < timeout <= 30:
        raise ValueError("Hand service timeout must be in (0, 30] seconds")
    payload = encode(document, MAX_REQUEST_BYTES)
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(str(socket_path))
        connection.sendall(payload)
        result = receive(connection, MAX_RESPONSE_BYTES)
    if result.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Unsupported hand service response version")
    if result.get("hand") != document.get("hand"):
        raise ValueError("Hand service response belongs to another hand")
    return result
