"""Bounded private documents and atomic, durable file replacement."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import stat
import tempfile

import yaml

MAX_DOCUMENT_BYTES = 1024 * 1024


def checked_path(path: str | Path, *, within: Path | None = None) -> Path:
    path = Path(path).absolute()
    for part in (path, *path.parents):
        if part.is_symlink():
            raise ValueError('calibration paths must not contain symbolic links')
    path = path.resolve()
    if within is not None and not path.is_relative_to(within.resolve()):
        raise ValueError('calibration artifact is outside its session directory')
    return path


def private_directory(path: str | Path) -> Path:
    result = checked_path(path)
    result.mkdir(mode=0o700, parents=True, exist_ok=True)
    if not result.is_dir():
        raise ValueError('calibration storage must be a directory')
    return result


class _UniqueLoader(yaml.SafeLoader):
    def compose_node(self, parent, index):
        if self.check_event(yaml.AliasEvent):
            raise ValueError('calibration document aliases are unsupported')
        return super().compose_node(parent, index)

    def construct_mapping(self, node, deep=False):
        seen = set()
        for key_node, _ in node.value:
            key = self.construct_object(key_node, deep=deep)
            if not isinstance(key, str) or key in seen:
                raise ValueError('document keys must be unique strings')
            seen.add(key)
        return super().construct_mapping(node, deep=deep)


def read_document(path: str | Path, *, within: Path | None = None) -> dict:
    path = checked_path(path, within=within)
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode):
            raise ValueError('calibration document must be a regular file')
        if info.st_size > MAX_DOCUMENT_BYTES:
            raise ValueError('calibration document is too large')
        with os.fdopen(fd, 'rb', closefd=False) as source:
            raw = source.read(MAX_DOCUMENT_BYTES + 1)
        if len(raw) > MAX_DOCUMENT_BYTES:
            raise ValueError('calibration document is too large')
    finally:
        os.close(fd)
    try:
        document = yaml.load(raw.decode('utf-8'), Loader=_UniqueLoader)
        encoded = json.dumps(document, allow_nan=False)
    except (yaml.YAMLError, UnicodeError, RecursionError, TypeError) as exc:
        raise ValueError('invalid calibration document') from exc
    if not isinstance(document, dict) or len(encoded.encode()) > MAX_DOCUMENT_BYTES:
        raise ValueError('calibration document must be a bounded mapping')
    return document


def canonical_bytes(document: dict) -> bytes:
    return json.dumps(document, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest_document(document: dict) -> str:
    return hashlib.sha256(canonical_bytes(document)).hexdigest()


def atomic_json(path: str | Path, document: dict) -> None:
    path = checked_path(path)
    private_directory(path.parent)
    data = canonical_bytes(document)
    if len(data) > MAX_DOCUMENT_BYTES:
        raise ValueError('calibration document is too large')
    fd, temporary = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as output:
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_DIRECTORY | os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        Path(temporary).unlink(missing_ok=True)
