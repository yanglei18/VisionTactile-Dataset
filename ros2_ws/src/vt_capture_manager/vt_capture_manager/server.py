"""Bounded private IPC: read-only v1 snapshots and opt-in v2 management."""
from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
import selectors
import socket
import stat
import threading
import time

from vt_capture_core import encode_message, decode_message, MAX_MESSAGE_BYTES, ProtocolError


def default_socket_path():
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    if runtime:
        parent = Path(runtime)
        info = parent.lstat()
        if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise ValueError("XDG_RUNTIME_DIR must be an owned private directory")
        return parent / "visiontactile" / "manager-control.sock"
    return Path.home() / ".local" / "state" / "visiontactile" / "manager-control.sock"


def _prepare_parent(path):
    parent = path.parent
    # Only create the immediate private service directory. Never chmod or
    # replace existing paths, and refuse symlink directories.
    missing = []
    ancestor = parent
    while not ancestor.exists() and not ancestor.is_symlink():
        missing.append(ancestor)
        ancestor = ancestor.parent
    for directory in reversed(missing):
        try:
            directory.mkdir(mode=0o700)
        except FileExistsError:
            pass
    info = parent.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise ValueError("socket parent must be an owned 0700 directory, not a symlink")
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"Refusing to replace existing socket/file: {path}")


@dataclass
class _Client:
    sock: socket.socket
    deadline: float
    incoming: bytearray = field(default_factory=bytearray)
    outgoing: bytes = b""


class SnapshotServer:
    """A selector thread bounds clients/bytes/time; slow clients never run in ROS."""
    def __init__(self, model, path=None, *, management=None, max_clients=16, client_timeout_s=2., max_message_bytes=MAX_MESSAGE_BYTES):
        if not 1 <= max_clients <= 128 or not 0.05 <= client_timeout_s <= 30:
            raise ValueError("invalid client resource bounds")
        if not 1024 <= max_message_bytes <= MAX_MESSAGE_BYTES:
            raise ValueError("invalid message byte bound")
        self.max_message_bytes = max_message_bytes
        self.model = model
        self.management = management
        self.path = Path(path) if path is not None else default_socket_path()
        self.max_clients, self.client_timeout_s = max_clients, client_timeout_s
        self._stop = threading.Event()
        self._thread = None
        self._listener = None
        self._inode = None

    def start(self):
        if self._thread is not None:
            raise RuntimeError("server is single-use")
        _prepare_parent(self.path)
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            listener.bind(str(self.path))
            self._inode = self.path.lstat().st_ino
            os.chmod(self.path, 0o600)
            listener.listen(self.max_clients)
            listener.setblocking(False)
            self._listener = listener
            self._thread = threading.Thread(target=self._run, name="capture-snapshots", daemon=True)
            self._thread.start()
        except BaseException:
            listener.close()
            self._cleanup_path()
            raise
        return self

    def _reply(self, payload):
        request_id = "invalid-request"
        version = 1
        try:
            request = decode_message(payload)
            request_id = request["request_id"]
            version = request["version"]
            if self.management is not None:
                response = self.management.dispatch(request)
            elif request.get("command") in {"snapshot", "ping"}:
                response = dict(version=version, request_id=request_id, ok=True, snapshot=self._snapshot(version))
                if version >= 2:
                    response["operation"] = None
            else:
                from .management import receipt
                response = dict(version=version, request_id=request_id, ok=False, snapshot=self._snapshot(version),
                                operation=receipt(request_id, "rejected", "read_only", "Manager is read-only."), error="Manager is read-only.")
        except (ProtocolError, ValueError, KeyError, TypeError) as exc:
            response = dict(version=version, request_id=request_id, ok=False, snapshot=self._snapshot(version), error=str(exc)[:512])
            if version >= 2:
                response["operation"] = None
        return encode_message(response)

    def _snapshot(self, version):
        if self.management is not None:
            return self.management.snapshot(version)
        result = self.model.snapshot()
        if version >= 2:
            from .management import empty_management
            result["management"] = empty_management()
        if version >= 4:
            result["process_control"] = dict(enabled=False, groups=[])
        if version >= 5:
            result["capabilities"]["capture_plan"] = False
        if version >= 6:
            from vt_capture_core.calibration_protocol import empty_calibration
            result["capabilities"]["calibration"] = False
            result["calibration"] = empty_calibration()
        return result

    def _run(self):
        selector = selectors.DefaultSelector()
        clients = {}
        selector.register(self._listener, selectors.EVENT_READ, None)

        def drop(client):
            clients.pop(client.sock.fileno(), None)
            try:
                selector.unregister(client.sock)
            except KeyError:
                pass
            client.sock.close()

        try:
            while not self._stop.is_set():
                for key, mask in selector.select(.05):
                    if key.data is None:
                        connection, _ = self._listener.accept()
                        if len(clients) >= self.max_clients:
                            connection.close()
                            continue
                        connection.setblocking(False)
                        client = _Client(connection, time.monotonic()+self.client_timeout_s)
                        clients[connection.fileno()] = client
                        selector.register(connection, selectors.EVENT_READ, client)
                        continue
                    client = key.data
                    try:
                        if mask & selectors.EVENT_READ:
                            data = client.sock.recv(65536)
                            if not data:
                                drop(client)
                                continue
                            client.incoming.extend(data)
                            if len(client.incoming) > self.max_message_bytes:
                                drop(client)
                                continue
                            if b"\n" in client.incoming:
                                # One request per connection. A second frame is
                                # rejected by the common decoder, not executed.
                                client.outgoing = self._reply(bytes(client.incoming))
                                selector.modify(client.sock, selectors.EVENT_WRITE, client)
                        if mask & selectors.EVENT_WRITE:
                            sent = client.sock.send(client.outgoing[:65536])
                            client.outgoing = client.outgoing[sent:]
                            if not client.outgoing:
                                drop(client)
                    except (OSError, ValueError, ProtocolError):
                        drop(client)
                now = time.monotonic()
                for client in list(clients.values()):
                    if now >= client.deadline:
                        drop(client)
        finally:
            for client in list(clients.values()):
                drop(client)
            selector.close()
            self._listener.close()
            self._cleanup_path()

    def _cleanup_path(self):
        try:
            info = self.path.lstat()
            if self._inode is not None and info.st_ino == self._inode and stat.S_ISSOCK(info.st_mode):
                self.path.unlink()
        except FileNotFoundError:
            pass

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(3.)
            if self._thread.is_alive():
                raise RuntimeError("snapshot thread did not stop")

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.close()
