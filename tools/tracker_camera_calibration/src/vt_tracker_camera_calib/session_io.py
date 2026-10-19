"""Bounded, durable JSON exchange for calibration subprocesses."""
from __future__ import annotations
import json
import os
from pathlib import Path
import tempfile

MAX_DOCUMENT_BYTES = 1024 * 1024


def read_document(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | getattr(os, 'O_NOFOLLOW', 0))
    with os.fdopen(descriptor, 'rb') as stream:
        payload = stream.read(MAX_DOCUMENT_BYTES + 1)
    if len(payload) > MAX_DOCUMENT_BYTES:
        raise ValueError('document exceeds size limit')
    return json.loads(payload)


def atomic_document(path, document):
    payload = (json.dumps(document, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n').encode()
    if len(payload) > MAX_DOCUMENT_BYTES:
        raise ValueError('document exceeds size limit')
    path = Path(path)
    descriptor, temporary = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        parent = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(parent)
        finally:
            os.close(parent)
    finally:
        Path(temporary).unlink(missing_ok=True)
