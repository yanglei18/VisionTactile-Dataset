"""Desktop activation tests use temporary IPC only; no Qt windows or hardware."""
import fcntl
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time

import pytest

from vt_capture_console import app
from vt_capture_console.single_instance import DesktopSingleInstance


@pytest.fixture
def instances(tmp_path):
    created = []

    def make(name="workstation"):
        instance = DesktopSingleInstance(tmp_path / name)
        created.append(instance)
        return instance

    yield make
    for instance in reversed(created):
        instance.close()


def test_second_launch_queues_activation_before_qt_is_ready(instances):
    first, second = instances(), instances()
    assert first.acquire_or_activate()
    assert not second.acquire_or_activate()
    assert first.consume_activation()
    assert not first.consume_activation()
    assert first._thread.is_alive()


def test_simultaneous_launches_choose_one_primary(instances):
    candidates = [instances() for _ in range(5)]
    barrier = threading.Barrier(len(candidates))
    results, errors = [], []

    def launch(candidate):
        try:
            barrier.wait(timeout=2.)
            results.append(candidate.acquire_or_activate())
        except Exception as exc:
            errors.append(exc)

    threads = [threading.Thread(target=launch, args=(candidate,)) for candidate in candidates]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=5.)
    assert not errors
    assert results.count(True) == 1
    assert results.count(False) == 4


def test_workstation_paths_have_independent_instances(instances):
    first, second = instances("one"), instances("two")
    assert first.acquire_or_activate()
    assert second.acquire_or_activate()
    assert first.address != second.address


def test_normal_exit_releases_lock_and_abstract_socket(instances):
    first = instances()
    assert first.acquire_or_activate()
    inode = first.lock_path.lstat().st_ino
    first.close()
    replacement = instances()
    assert replacement.acquire_or_activate()
    assert replacement.lock_path.lstat().st_ino == inode
    assert not any(path.suffix == ".sock" for path in first.directory.iterdir())


def test_crashed_test_owner_does_not_leave_stale_activation_socket(instances):
    candidate = instances()
    code = """
import sys, time
from vt_capture_console.single_instance import DesktopSingleInstance
instance = DesktopSingleInstance(sys.argv[1])
assert instance.acquire_or_activate()
print('ready', flush=True)
time.sleep(60)
"""
    child = subprocess.Popen([sys.executable, "-c", code, str(candidate.directory)],
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        import select
        assert select.select([child.stdout], [], [], 3.)[0]
        assert child.stdout.readline().strip() == "ready"
        child.kill()  # Only the exact temporary test child created above.
        child.wait(timeout=2.)
        assert candidate.acquire_or_activate()
    finally:
        if child.poll() is None:
            child.kill()
        child.communicate(timeout=2.)


def test_busy_owner_without_listener_has_bounded_failure(instances):
    candidate = instances()
    fd = os.open(candidate.lock_path, os.O_RDWR | os.O_CREAT, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        started = time.monotonic()
        with pytest.raises(RuntimeError, match="未重复打开"):
            candidate.acquire_or_activate(timeout=.1)
        assert time.monotonic() - started < .5
        assert candidate._listener is None
    finally:
        os.close(fd)


@pytest.mark.parametrize("kind", ["symlink", "public", "hardlink"])
def test_invalid_lock_file_is_preserved(instances, kind):
    candidate = instances()
    other = candidate.directory / "preserve"
    other.write_text("preserve me")
    other.chmod(0o600)
    if kind == "symlink":
        candidate.lock_path.symlink_to(other)
    elif kind == "hardlink":
        os.link(other, candidate.lock_path)
    else:
        candidate.lock_path.write_text("preserve lock")
        candidate.lock_path.chmod(0o644)
    with pytest.raises((OSError, RuntimeError)):
        candidate.acquire_or_activate()
    assert other.read_text() == "preserve me"
    assert candidate.lock_path.exists()


@pytest.mark.parametrize("payload", [b"start-recording\n", b"VT_CAPTURE_ACTIVATE/2\n", b"x" * 1024])
def test_unknown_or_oversized_messages_cannot_activate(instances, payload):
    first = instances()
    assert first.acquire_or_activate()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(1.)
        connection.connect(first.address)
        connection.sendall(payload)
        connection.shutdown(socket.SHUT_WR)
        try:
            assert connection.recv(128) == b""
        except ConnectionResetError:
            pass
    assert not first.consume_activation()


def test_partial_client_cannot_block_later_activation(instances):
    first, second = instances(), instances()
    assert first.acquire_or_activate()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stalled:
        stalled.connect(first.address)
        stalled.sendall(first.REQUEST[:1])
        before = time.monotonic()
        assert not second.acquire_or_activate(timeout=1.)
        assert time.monotonic() - before < 1.
    assert first.consume_activation()


def test_peer_uid_mismatch_rejects_activation(instances, monkeypatch):
    first = instances()
    monkeypatch.setattr(first, "_same_user", lambda connection: False)
    assert first.acquire_or_activate()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(1.)
        connection.connect(first.address)
        connection.sendall(first.REQUEST)
        try:
            assert connection.recv(128) == b""
        except ConnectionResetError:
            pass
    assert not first.consume_activation()


def test_duplicate_app_returns_before_application_or_ui_imports(monkeypatch):
    calls = []

    class Existing:
        def acquire_or_activate(self):
            calls.append("activate")
            return False

        def close(self):
            calls.append("close")

    monkeypatch.setattr("vt_capture_console.single_instance.DesktopSingleInstance", Existing)
    monkeypatch.setattr(app, "_run_application", lambda *args: pytest.fail("Qt application initialized"))
    assert app.main(["--desktop"]) == 0
    assert calls == ["activate", "close"]


@pytest.mark.parametrize("arguments", [["--demo"], ["--live"], ["--desktop", "--smoke-seconds", "1"]])
def test_demo_live_and_smoke_runs_skip_desktop_single_instance(monkeypatch, arguments):
    monkeypatch.setattr("vt_capture_console.single_instance.DesktopSingleInstance",
                        lambda: pytest.fail("independent run acquired desktop lock"))
    received = []
    monkeypatch.setattr(app, "_run_application", lambda args, width, height, instance: received.append(instance) or 17)
    assert app.main(arguments) == 17
    assert received == [None]


def test_importing_admission_module_does_not_import_qt():
    result = subprocess.run([sys.executable, "-c", "import sys; import vt_capture_console.single_instance; "
                             "assert not any(name.startswith(('PySide6', 'OpenGL', 'rclpy')) for name in sys.modules)"],
                            capture_output=True, text=True, timeout=3.)
    assert result.returncode == 0, result.stderr
