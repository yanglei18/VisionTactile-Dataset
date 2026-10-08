import os
from pathlib import Path
import socket
import tempfile

import pytest

from vt_glove_bridge.transport import LocalReceiver


@pytest.fixture
def endpoint():
    with tempfile.TemporaryDirectory(prefix="vtg-") as directory:
        yield str(Path(directory) / "g.sock")


def test_receive_empty_nonblocking_and_cleanup(endpoint):
    receiver = LocalReceiver(endpoint)
    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        assert receiver.receive() is None
        assert os.stat(endpoint).st_mode & 0o777 == 0o600
        sender.sendto(b"hello", endpoint)
        assert receiver.receive() == b"hello"
    finally:
        sender.close()
        receiver.close()
    assert not Path(endpoint).exists()


def test_does_not_steal_running_receiver(endpoint):
    receiver = LocalReceiver(endpoint)
    try:
        with pytest.raises((ValueError, BlockingIOError)):
            LocalReceiver(endpoint)
        assert Path(endpoint).is_socket()
    finally:
        receiver.close()


def test_does_not_steal_foreign_listener_without_lock(endpoint):
    original = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    original.bind(endpoint)
    try:
        with pytest.raises(ValueError, match="another receiver"):
            LocalReceiver(endpoint)
        assert Path(endpoint).is_socket()
    finally:
        original.close()


def test_stale_socket_can_restart(endpoint):
    original = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    original.bind(endpoint)
    original.close()
    receiver = LocalReceiver(endpoint)
    receiver.close()
    receiver = LocalReceiver(endpoint)
    receiver.close()


def test_regular_file_is_never_removed(endpoint):
    Path(endpoint).write_text("keep")
    with pytest.raises(ValueError, match="non-socket"):
        LocalReceiver(endpoint)
    assert Path(endpoint).read_text() == "keep"


def test_rejects_public_parent(endpoint):
    os.chmod(Path(endpoint).parent, 0o755)
    with pytest.raises(ValueError, match="private"):
        LocalReceiver(endpoint)


def test_oversize_datagram_rejected_and_next_packet_survives(endpoint):
    receiver = LocalReceiver(endpoint)
    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        sender.sendto(b"x" * 60001, endpoint)
        with pytest.raises(ValueError, match="exceeds"):
            receiver.receive()
        sender.sendto(b"good", endpoint)
        assert receiver.receive() == b"good"
    finally:
        sender.close()
        receiver.close()
