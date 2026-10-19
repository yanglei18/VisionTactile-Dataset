"""Retention uses a verified existing output file, with no child processes."""
import os
import time

import pytest


def eventually(predicate, timeout=3.):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('log retention did not complete')


def append_file(path):
    return os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)


def test_retention_keeps_appending_and_does_not_follow_replaced_path(tmp_path):
    from vt_capture_core.process_log import ProcessLogRetention
    path = tmp_path / 'manager.log'
    fd = append_file(path)
    try:
        with ProcessLogRetention(path, output_fd=fd, limit_bytes=8192, keep_bytes=4096, interval_s=.01):
            os.write(fd, b'a' * 20000)
            eventually(lambda: path.stat().st_size == 4096)
            old = path.with_name('old.log')
            path.rename(old)
            path.write_bytes(b'unrelated file')
            os.write(fd, b'b' * 20000)
            eventually(lambda: old.stat().st_size == 4096)
            assert old.read_bytes() == b'b' * 4096
            assert path.read_bytes() == b'unrelated file'
        os.write(fd, b'after-close')
        assert old.read_bytes().endswith(b'after-close')
    finally:
        os.close(fd)


@pytest.mark.parametrize('unsafe', ['symlink', 'different_inode', 'public_file', 'fifo'])
def test_retention_rejects_unrelated_or_unsafe_files(tmp_path, unsafe):
    from vt_capture_core.process_log import ProcessLogRetention
    path, other = tmp_path / 'manager.log', tmp_path / 'other'
    fd = append_file(path)
    try:
        if unsafe == 'symlink':
            other.symlink_to(path)
            candidate = other
        elif unsafe == 'different_inode':
            other.touch(mode=0o600)
            candidate = other
        elif unsafe == 'public_file':
            path.chmod(0o644)
            candidate = path
        else:
            os.mkfifo(other, 0o600)
            candidate = other
        with pytest.raises((OSError, ValueError)):
            ProcessLogRetention(candidate, output_fd=fd)
    finally:
        os.close(fd)


def test_retention_rejects_output_without_append_semantics(tmp_path):
    from vt_capture_core.process_log import ProcessLogRetention
    path = tmp_path / 'manager.log'
    fd = os.open(path, os.O_WRONLY | os.O_CREAT, 0o600)
    try:
        with pytest.raises(ValueError):
            ProcessLogRetention(path, output_fd=fd)
    finally:
        os.close(fd)
