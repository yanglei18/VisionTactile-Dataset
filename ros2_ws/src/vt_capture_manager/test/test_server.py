import os
import socket
import stat
import time

import pytest

from vt_capture_core import decode_message, encode_message
from vt_capture_manager.config import load_config
from vt_capture_manager.model import SnapshotModel
from vt_capture_manager.server import SnapshotServer, default_socket_path


def call(path, payload=None):
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(2)
        client.connect(str(path))
        client.sendall(payload or encode_message(dict(version=1, request_id="one", command="snapshot")))
        chunks = bytearray()
        while not chunks.endswith(b"\n"):
            data = client.recv(65536)
            if not data:
                break
            chunks.extend(data)
        return decode_message(bytes(chunks))


def test_real_socket_permissions_and_read_only_roundtrip(tmp_path):
    path = tmp_path / "private" / "manager.sock"
    with SnapshotServer(SnapshotModel(load_config()), path):
        assert stat.S_IMODE(path.stat().st_mode) == 0o600
        assert stat.S_IMODE(path.parent.stat().st_mode) == 0o700
        response = call(path)
        assert response["ok"] and response["request_id"] == "one"
        assert response["snapshot"]["mode"] == "live"
        assert response["snapshot"]["capabilities"] == dict(record=False, device_control=False)
        error = call(path, b'{"version":1,"request_id":"bad","command":"record"}\n')
        assert not error["ok"] and "snapshot" in error
    assert not path.exists()


def test_existing_socket_file_and_symlink_are_never_replaced(tmp_path):
    model = SnapshotModel(load_config())
    path = tmp_path / "private" / "manager.sock"
    with SnapshotServer(model, path):
        with pytest.raises(FileExistsError):
            SnapshotServer(model, path).start()
        assert call(path)["ok"]
    path.write_text("user data")
    with pytest.raises(FileExistsError):
        SnapshotServer(model, path).start()
    assert path.read_text() == "user data"
    link = path.parent / "link.sock"
    link.symlink_to(path)
    with pytest.raises(FileExistsError):
        SnapshotServer(model, link).start()


def test_slow_partial_and_disconnected_clients_do_not_block(tmp_path):
    path = tmp_path / "private" / "manager.sock"
    with SnapshotServer(SnapshotModel(load_config()), path, client_timeout_s=.2):
        slow = socket.socket(socket.AF_UNIX)
        slow.connect(str(path))
        slow.sendall(b'{"version":')
        assert call(path)["ok"]
        broken = socket.socket(socket.AF_UNIX)
        broken.connect(str(path))
        broken.sendall(encode_message(dict(version=1, request_id="broken", command="snapshot")))
        broken.close()
        assert call(path)["ok"]
        time.sleep(.25)
        slow.settimeout(1)
        assert slow.recv(1) == b""
        slow.close()
        assert call(path)["ok"]


def test_oversized_input_closed_and_next_client_works(tmp_path):
    path = tmp_path / "private" / "manager.sock"
    with SnapshotServer(SnapshotModel(load_config()), path):
        with socket.socket(socket.AF_UNIX) as client:
            client.settimeout(2)
            client.connect(str(path))
            try:
                client.sendall(b"x"*(2*1024*1024+1))
                assert client.recv(1) == b""
            except (BrokenPipeError, ConnectionResetError):
                pass
        assert call(path)["ok"]


def test_default_private_paths(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    assert default_socket_path() == tmp_path / "visiontactile/manager-control.sock"
    monkeypatch.delenv("XDG_RUNTIME_DIR")
    monkeypatch.setenv("HOME", str(tmp_path))
    assert default_socket_path() == tmp_path / ".local/state/visiontactile/manager-control.sock"
