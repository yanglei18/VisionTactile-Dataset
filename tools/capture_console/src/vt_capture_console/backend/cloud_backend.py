"""One binary cloud receiver; bounded by camera count and strict frame identity."""
from __future__ import annotations

import os
import socket
import struct
import threading
import time

from vt_capture_core.spatial.cloud_protocol import PREFIX_BYTES, inspect_cloud_prefix, unpack_cloud_frame


class CloudBackend:
    def __init__(self, *, autostart=True):
        self._lock = threading.RLock()
        self._descriptor = None
        self._manager = None
        self._frames = {}
        self._interest = set()
        self._status = "点云后端尚未启用"
        self._stop, self._wake = threading.Event(), threading.Event()
        self._connection = None
        self._thread = None
        if autostart:
            self._thread = threading.Thread(target=self._run, name="capture-console-cloud", daemon=True)
            self._thread.start()

    def configure(self, descriptor, manager_instance_id):
        descriptor = dict(descriptor) if descriptor else None
        if descriptor and (not isinstance(descriptor.get("socket_path"), str)
                           or not isinstance(descriptor.get("worker_instance_id"), str)
                           or type(descriptor.get("worker_pid")) is not int):
            descriptor = None
        with self._lock:
            if (descriptor, manager_instance_id) != (self._descriptor, self._manager):
                self._descriptor, self._manager = descriptor, manager_instance_id
                self._frames.clear()
                self._disconnect()
            self._wake.set()

    def set_interest(self, viewer_id, enabled):
        with self._lock:
            self._interest.add(viewer_id) if enabled else self._interest.discard(viewer_id)
            if not self._interest:
                self._frames.clear()
                self._disconnect()
        self._wake.set()

    def status(self):
        with self._lock:
            return self._status

    def _disconnect(self):
        connection = self._connection
        self._connection = None
        if connection:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()

    def accept_frame(self, wire):
        import numpy as np
        frame = unpack_cloud_frame(wire)
        header = frame.header
        with self._lock:
            descriptor = self._descriptor
            if (not descriptor or header.get("manager_instance_id") != self._manager
                    or header["worker_instance_id"] != descriptor["worker_instance_id"]):
                return False
            key = header["camera_id"]
            if key not in self._frames and len(self._frames) >= 3:
                return False
            previous = self._frames.get(key)
            if previous and header["cloud_sequence"] <= previous["header"]["cloud_sequence"]:
                return False
            positions = np.frombuffer(frame.xyz_bytes, dtype="<f4").reshape(-1, 3)
            colors = np.frombuffer(frame.rgba_bytes, dtype="u1").reshape(-1, 4).astype("f4")/255.
            positions.setflags(write=False)
            colors.setflags(write=False)
            self._frames[key] = {"header": header, "positions": positions, "colors": colors,
                                 "received_monotonic": time.monotonic()}
            self._status = "点云已接收，等待同帧场景"
            return True

    def latest(self, scene):
        if not scene:
            return {}
        with self._lock:
            frames = dict(self._frames)
        matches = {}
        cameras = {c.get("id", c.get("camera_id")): c for c in scene.get("cameras", ())}
        for key, value in frames.items():
            header = value["header"]
            camera = cameras.get(key)
            if camera is None or time.monotonic()-value["received_monotonic"] > 1.:
                continue
            if any(field not in scene or field not in header or scene[field] != header[field]
                   for field in ("manager_instance_id", "scene_epoch", "time_epoch", "coordinate_epoch", "sync_epoch")):
                continue
            if any(field not in camera or camera[field] != header.get(field)
                   for field in ("frame_key", "geometry_revision", "geometry_sha256")):
                continue
            matches[key] = value
        return matches

    @staticmethod
    def _read_exact(connection, size, deadline):
        data = bytearray()
        while len(data) < size:
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError("点云帧接收超时")
            connection.settimeout(remaining)
            block = connection.recv(size-len(data))
            if not block:
                raise ConnectionError("点云流已断开")
            data.extend(block)
        return bytes(data)

    def _run(self):
        while not self._stop.is_set():
            with self._lock:
                descriptor = dict(self._descriptor) if self._descriptor and self._interest else None
            if descriptor is None:
                self._wake.wait(.25)
                self._wake.clear()
                continue
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                    connection.settimeout(.5)
                    connection.connect(descriptor["socket_path"])
                    pid, uid, _ = struct.unpack("3i", connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
                    if uid != os.getuid() or pid != descriptor["worker_pid"]:
                        raise ValueError("点云进程身份不匹配")
                    with self._lock:
                        if descriptor != self._descriptor or not self._interest:
                            continue
                        self._connection = connection
                    while not self._stop.is_set():
                        deadline = time.monotonic()+1.
                        prefix = self._read_exact(connection, PREFIX_BYTES, deadline)
                        header_bytes, payload_bytes = inspect_cloud_prefix(prefix)
                        frame = prefix+self._read_exact(connection, header_bytes+payload_bytes, deadline)
                        self.accept_frame(frame)
            except (OSError, ValueError, TypeError, ConnectionError) as exc:
                with self._lock:
                    self._status = "点云暂不可用："+str(exc)[:160]
                    self._frames.clear()
                self._stop.wait(.5)
            finally:
                with self._lock:
                    self._connection = None

    def close(self):
        self._stop.set()
        self._wake.set()
        with self._lock:
            self._disconnect()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=.7)
