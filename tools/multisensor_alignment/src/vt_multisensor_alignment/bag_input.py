"""Shared, bounded file validation for local ROS bag readers.

Directory traversal and file reads use no-follow descriptors, including every
ancestor. Native ROS storage still opens paths itself: callers must keep bags
closed and immutable while reading; this is not an OS sandbox.
"""
from contextlib import contextmanager
import os
from pathlib import Path
import stat

import yaml


MAX_METADATA_BYTES = 4 * 1024 * 1024
MAX_STORAGE_FILES = 4096


def absolute_path(path):
    # Do not resolve symlinks before the no-follow checks can see them.
    return Path(path).expanduser().absolute()


@contextmanager
def _regular_file(path):
    path = absolute_path(path)
    if '..' in path.parts:
        raise ValueError(f'Bag input path cannot contain parent traversal: {path}')
    directory = descriptor = None
    try:
        flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
        directory = os.open(path.anchor, flags)
        for component in path.parts[1:-1]:
            child = os.open(component, flags, dir_fd=directory)
            os.close(directory)
            directory = child
        descriptor = os.open(path.name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC,
                             dir_fd=directory)
        if not stat.S_ISREG(os.fstat(descriptor).st_mode):
            raise ValueError(f'Bag input must be a regular file: {path}')
        yield descriptor
    except OSError as exc:
        raise ValueError(f'Bag input must exist without symlink components: {path}') from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if directory is not None:
            os.close(directory)


def _identity(info):
    return info.st_dev, info.st_ino, info.st_mode, info.st_size, info.st_mtime_ns, info.st_ctime_ns


def fingerprint(files):
    result = []
    for file in files:
        with _regular_file(file) as descriptor:
            result.append((str(file), *_identity(os.fstat(descriptor))))
    return tuple(result)


def read_metadata(path):
    metadata = absolute_path(path) / 'metadata.yaml'
    with _regular_file(metadata) as descriptor:
        before = _identity(os.fstat(descriptor))
        if os.fstat(descriptor).st_size > MAX_METADATA_BYTES:
            raise ValueError('Bag metadata.yaml exceeds 4 MiB')
        chunks, size = [], 0
        while size <= MAX_METADATA_BYTES:
            chunk = os.read(descriptor, min(65536, MAX_METADATA_BYTES + 1 - size))
            if not chunk:
                break
            chunks.append(chunk)
            size += len(chunk)
        if size > MAX_METADATA_BYTES or _identity(os.fstat(descriptor)) != before:
            raise ValueError('Bag metadata.yaml changed or exceeded 4 MiB while reading')
    try:
        document = yaml.safe_load(b''.join(chunks))
    except (yaml.YAMLError, UnicodeError, RecursionError) as exc:
        raise ValueError('Invalid bag metadata.yaml') from exc
    if not isinstance(document, dict) or not isinstance(document.get('rosbag2_bagfile_information'), dict):
        raise ValueError('Invalid rosbag2_bagfile_information in metadata.yaml')
    return document['rosbag2_bagfile_information'], (str(metadata), *before)


def inventory(path):
    path = absolute_path(path)
    info, metadata_identity = read_metadata(path)
    if any(path.glob('*.active')):
        raise ValueError('Replay requires a closed bag; active recording found')
    names = info.get('relative_file_paths', [])
    if not isinstance(names, list) or len(names) > MAX_STORAGE_FILES:
        raise ValueError('Invalid bag storage inventory')
    files = [path / 'metadata.yaml']
    seen = set()
    for name in names:
        if not isinstance(name, str) or not name or len(name) > 4096 or '\x00' in name:
            raise ValueError('Invalid bag storage path')
        relative = Path(name)
        if not relative.parts or relative.is_absolute() or '..' in relative.parts or relative in seen:
            raise ValueError('Invalid bag storage path')
        seen.add(relative)
        files.append(path / relative)
    identities = fingerprint(files)
    if identities[0] != metadata_identity:
        raise ValueError('Bag metadata.yaml changed during inventory')
    return info, identities
