"""One binary cloud receiver; bounded by camera count and strict frame identity."""
from __future__ import annotations

from collections import deque
import os
import socket
import struct
import threading
import time

from vt_capture_core.spatial.cloud_protocol import PREFIX_BYTES, inspect_cloud_prefix, unpack_cloud_frame

# Cover the supervisor's original 0.5 s complete-group lease at 15 Hz.
MAX_CAMERA_FRAMES = 8
MAX_CAMERA_BYTES = 128 * 1024 * 1024
FRAME_TTL_S = 1.
_GENERATION_FIELDS = ("manager_instance_id", "scene_epoch", "time_epoch", "coordinate_epoch", "sync_epoch")
_GEOMETRY_FIELDS = ("geometry_revision", "geometry_sha256")


class CloudBackend:
    def __init__(self, *, autostart=True):
        self._lock = threading.RLock()
        self._descriptor = None
        self._manager = None
        self._frames = {}
        self._last_headers = {}
        self._generation = None
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
                self._clear_frames()
                self._disconnect()
            self._wake.set()

    def set_interest(self, viewer_id, enabled):
        with self._lock:
            self._interest.add(viewer_id) if enabled else self._interest.discard(viewer_id)
            if not self._interest:
                self._clear_frames()
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

    def _clear_frames(self):
        self._frames.clear()
        self._last_headers.clear()
        self._generation = None

    def _expire_frames(self, now):
        # Keep only bounded header/sequence fences after expiry, not point arrays.
        for history in self._frames.values():
            while history and now-history[0]["received_monotonic"] > FRAME_TTL_S:
                history.popleft()

    def accept_frame(self, wire):
        import numpy as np
        frame = unpack_cloud_frame(wire, copy_payload=False)
        header = frame.header
        with self._lock:
            descriptor = self._descriptor
            if (self._stop.is_set() or not descriptor or header.get("manager_instance_id") != self._manager
                    or header["worker_instance_id"] != descriptor["worker_instance_id"]):
                return False
            key = header["camera_id"]
            generation = tuple(header.get(field) for field in _GENERATION_FIELDS)
            if self._generation is not None and generation != self._generation:
                # Worker sequences continue across epochs. A delayed old packet
                # must not roll the cache back to a retired generation.
                if header["cloud_sequence"] <= max(h["cloud_sequence"] for h in self._last_headers.values()):
                    return False
                self._clear_frames()
            if key not in self._frames and len(self._frames) >= 3:
                return False
            previous = self._last_headers.get(key)
            if previous and header["cloud_sequence"] <= previous["cloud_sequence"]:
                return False
            now = time.monotonic()
            self._expire_frames(now)
            history = self._frames.setdefault(key, deque(maxlen=MAX_CAMERA_FRAMES))
            if previous and any(previous[field] != header[field] for field in _GEOMETRY_FIELDS):
                history.clear()
            positions = np.frombuffer(frame.xyz_bytes, dtype="<f4").reshape(-1, 3)
            colors = np.frombuffer(frame.rgba_bytes, dtype="u1").reshape(-1, 4).astype("f4")/255.
            positions.setflags(write=False)
            colors.setflags(write=False)
            # Binary transport can lead the scene stream by a few frames. Retain
            # exact candidates instead of replacing the still-referenced frame.
            history.append({"header": header, "positions": positions, "colors": colors,
                            "received_monotonic": now,
                            "retained_bytes": len(wire)+colors.nbytes})
            while len(history)>1 and sum(v['retained_bytes'] for v in history)>MAX_CAMERA_BYTES:
                history.popleft()
            self._last_headers[key] = header
            self._generation = generation
            self._status = "点云已接收，等待同帧场景"
            return True

    def latest(self, scene):
        with self._lock:
            self._expire_frames(time.monotonic())
            if not scene:
                return {}
            matches = {}
            cameras = {c.get("id", c.get("camera_id")): c for c in scene.get("cameras", ())}
            for key, history in self._frames.items():
                camera = cameras.get(key)
                if camera is None:
                    continue
                for value in reversed(history):
                    header = value["header"]
                    if any(field not in scene or field not in header or scene[field] != header[field]
                           for field in _GENERATION_FIELDS):
                        continue
                    if any(field not in camera or camera[field] != header.get(field)
                           for field in _GEOMETRY_FIELDS):
                        continue
                    if scene.get('time_mode') == 'independent':
                        # Metadata can advance before a multi-MiB packet arrives.
                        # This packet carries BOTH source poses; validate their
                        # binding rather than requiring a newer scene frame key.
                        bound=header.get('tracker_pose',{})
                        tracker=next((t for t in scene.get('trackers',())
                                      if t.get('role')==camera.get('tracker_role')),None)
                        age=scene.get('generated_realtime_ns',0)-header['source_realtime_ns']
                        if (header.get('matching_mode')!='independent' or camera.get('registration_state')!='ready'
                                or tracker is None or bound.get('role')!=camera.get('tracker_role')
                                or bound.get('tracker_id')!=tracker.get('tracker_id')
                                or not 0<=age<1_000_000_000):
                            continue
                    elif camera.get('frame_key') != header.get('frame_key'):
                        continue
                    matches[key] = value
                    break
            return matches

    @staticmethod
    def _read_into_exact(connection, target, deadline):
        offset = 0
        while offset < len(target):
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError("点云帧接收超时")
            connection.settimeout(remaining)
            count = connection.recv_into(target[offset:])
            if not count:
                raise ConnectionError("点云流已断开")
            offset += count

    @staticmethod
    def _read_exact(connection, size, deadline):
        data = bytearray(size)
        CloudBackend._read_into_exact(connection, memoryview(data), deadline)
        return bytes(data)

    def _read_prefix(self, connection):
        # No frame is not a broken connection. Wait for the first bytes without
        # resetting the socket/sequence fence on every one-second input gap.
        # Once a frame begins, partial header/payload still have a hard deadline.
        idle_since=time.monotonic()
        while not self._stop.is_set():
            connection.settimeout(.25)
            try:
                first=connection.recv(PREFIX_BYTES)
            except socket.timeout:
                if time.monotonic()-idle_since>=1.:
                    with self._lock:
                        self._status='等待新点云（源端暂无输出）'
                        self._expire_frames(time.monotonic())
                continue
            if not first:
                raise ConnectionError('点云流已断开')
            deadline=time.monotonic()+1.
            return first+self._read_exact(connection,PREFIX_BYTES-len(first),deadline),deadline
        return None

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
                        received=self._read_prefix(connection)
                        if received is None:
                            break
                        prefix,deadline=received
                        header_bytes, payload_bytes = inspect_cloud_prefix(prefix)
                        frame = bytearray(PREFIX_BYTES+header_bytes+payload_bytes)
                        frame[:PREFIX_BYTES] = prefix
                        self._read_into_exact(connection, memoryview(frame)[PREFIX_BYTES:], deadline)
                        # Freeze once; all decoded XYZ views share this owner.
                        self.accept_frame(bytes(frame))
            except (OSError, ValueError, TypeError, ConnectionError) as exc:
                with self._lock:
                    self._status = "点云暂不可用："+str(exc)[:160]
                    self._clear_frames()
                self._stop.wait(.5)
            finally:
                with self._lock:
                    self._connection = None

    def close(self):
        self._stop.set()
        self._wake.set()
        with self._lock:
            self._disconnect()
            self._clear_frames()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=.7)
