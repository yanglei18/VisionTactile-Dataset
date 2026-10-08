"""Thread-safe pure-data projection of received ROS messages (duck typed).

Freshness is evidence about messages, not a claim of physical connectivity.
No serial, Qt, ROS, model inference, filesystem writes, or recorder control.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass, field
import math
import shutil
import threading
import time
import uuid

from .config import ManagerConfig


def _finite(values, count):
    result = [float(v) for v in values]
    if len(result) != count or not all(math.isfinite(v) for v in result):
        raise ValueError(f"expected {count} finite values")
    return result


def _stamp_ns(message):
    value = getattr(message, "host_realtime_ns", None)
    if value is None:
        stamp = message.header.stamp
        value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return int(value)


def _mano_identity(message):
    """Only bounded result provenance, never raw IMU or calibration contents."""
    result = {}
    for name, limit in (("session_id", 256), ("config_sha256", 128), ("model_rest_sha256", 128)):
        value = getattr(message, name)
        if not isinstance(value, str) or not value or len(value) > limit:
            raise ValueError(f"invalid MANO preview {name}")
        result[name] = value
    for name, bits in (("calibration_version", 32), ("frame_index", 64)):
        value = getattr(message, name)
        if type(value) is not int or not 0 <= value < (1 << bits):
            raise ValueError(f"invalid MANO preview {name}")
        result[name] = value
    return result


@dataclass
class _Device:
    arrivals: deque = field(default_factory=lambda: deque(maxlen=4096))
    valid_arrivals: deque = field(default_factory=lambda: deque(maxlen=4096))
    received_ns: int | None = None
    source_ns: int = 0
    source_order_ns: int = 0
    valid: bool = False
    explicitly_disconnected: bool = False
    detail: str = "No messages received; physical connection is unknown."
    preview: dict = field(default_factory=dict)
    identity: tuple | None = None
    last_sample_key: tuple | None = None
    cell_stamps: list = field(default_factory=list)
    cell_max_age_ns: int = 0
    preview_source_ns: int = 0


class SnapshotModel:
    def __init__(self, config: ManagerConfig, *, monotonic_ns=time.monotonic_ns,
                 realtime_ns=time.time_ns, disk_usage=shutil.disk_usage, instance_id=None):
        self.config = config
        self._mono, self._wall, self._disk = monotonic_ns, realtime_ns, disk_usage
        self.instance_id = instance_id or uuid.uuid4().hex
        self._lock = threading.RLock()
        self._configs = {d.id: d for d in config.devices}
        self._devices = {d.id: _Device() for d in config.devices}
        self._events = deque(maxlen=32)
        self._event_id = 0
        self._sequence = 0
        self._capture_received_ns = None
        self._capture_source_ns = 0
        self._capture = None
        self._recording_started_ns = None
        self._event_keys = {}

    def event(self, level, message, *, key=None):
        with self._lock:
            now = self._mono()
            if key is not None and now - self._event_keys.get(key, -10**18) < 5_000_000_000:
                return
            if key is not None:
                self._event_keys[key] = now
            self._event_id += 1
            self._events.append(dict(id=str(self._event_id), level=level, message=str(message)[:1024]))

    def observe(self, device_id, message):
        """Record one received payload; malformed messages never replace good geometry."""
        with self._lock:
            config, state = self._configs[device_id], self._devices[device_id]
            now = self._mono()
            state.received_ns = now
            state.arrivals.append(now)
            try:
                source_ns = _stamp_ns(message)
                wall = self._wall()
                timestamp_eligible = 0 < source_ns <= wall + 500_000_000
                fresh_source = timestamp_eligible and wall-source_ns <= int(self.config.stale_after_s * 1e9)
                preview, valid, detail, disconnected, identity = self._project(config, message)
                if timestamp_eligible and identity == state.identity and source_ns < state.source_order_ns:
                    self.event("warning", f"{device_id}: ignored out-of-order source timestamp", key=f"order:{device_id}")
                    return
                # An invalid/future clock is still displayed as an invalid
                # observation, but must not poison accepted ordering, session
                # identity or duplicate suppression for the next normal frame.
                if timestamp_eligible:
                    if identity is not None and identity != state.identity:
                        state.valid_arrivals.clear()
                        state.identity = identity
                    state.source_order_ns = source_ns
                state.source_ns = source_ns
                state.valid = bool(valid and fresh_source)
                if config.kind != "camera":
                    state.preview = preview
                state.explicitly_disconnected = disconnected
                state.detail = detail if fresh_source else f"Source timestamp unavailable, future or stale. {detail}"
                sample_key = (identity, source_ns, getattr(message, "frame_index", None),
                              getattr(message, "source_sequence", None), getattr(message, "packet_index", None))
                if state.valid and sample_key != state.last_sample_key:
                    state.valid_arrivals.append(now)
                if timestamp_eligible:
                    state.last_sample_key = sample_key
                if config.kind == "tactile":
                    state.cell_stamps = [int(message.block_realtime_ns[r*8+c%8]) for r in range(20) for c in range(20)]
                    state.cell_max_age_ns = min(int(message.max_age_ns), int(self.config.stale_after_s * 1e9))
            except (AttributeError, TypeError, ValueError, OverflowError) as exc:
                state.valid = False
                state.preview = {}
                state.source_ns = 0
                state.detail = f"Rejected malformed {config.kind} message: {exc}"
                self.event("warning", f"{device_id}: {state.detail}", key=device_id)

    def _project(self, config, message):
        kind = config.kind
        if kind == "camera":
            width, height, step = int(message.width), int(message.height), int(message.step)
            if width <= 0 or height <= 0 or step <= 0 or width > 16384 or height > 16384:
                raise ValueError("invalid image dimensions/stride")
            if len(message.data) < step * height:
                raise ValueError("truncated image data")
            if step * height > 32 * 1024 * 1024 or len(message.data) > 32 * 1024 * 1024:
                raise ValueError("image exceeds 32 MiB preview bound")
            return {}, True, f"{width}x{height} {message.encoding}; preview display only; depth units are not inferred.", False, None
        if kind == "tracker":
            if message.role != config.id or message.header.frame_id != "vive_map":
                raise ValueError("tracker role/frame mismatch")
            p, q = message.pose.position, message.pose.orientation
            position = _finite((p.x, p.y, p.z), 3)
            orientation = _finite((q.x, q.y, q.z, q.w), 4)
            norm = math.sqrt(sum(v*v for v in orientation))
            valid = bool(message.pose_valid) and .9 <= norm <= 1.1
            preview = dict(position=position, orientation=orientation, valid=valid)
            return preview, valid, f"Tracker {message.tracker_id}; tracking={message.tracking_status}; valid Hz counts pose_valid samples.", False, (message.tracker_id,)
        if message.hand != config.side:
            raise ValueError("hand does not match configured side")
        if kind == "glove":
            if message.header.frame_id != f"glove_{config.side}_reference":
                raise ValueError("glove reference frame mismatch")
            if int(message.schema_version) != 1:
                raise ValueError("unsupported glove schema")
            joints = []
            if len(message.joint_positions_m):
                flat = _finite(message.joint_positions_m, 63)
                joints = [flat[i:i+3] for i in range(0, 63, 3)]
            valid = bool(message.pose_valid and message.time_valid)
            preview = dict(joints=joints, valid=bool(valid and joints),
                           quality=str(message.quality)[:128], reason=str(message.reason)[:512])
            source_mode = getattr(message, "source_mode", "")
            if source_mode == "replay":
                # Legacy skeleton consumers also need to distinguish replay.
                preview["replay"] = True
            if valid:
                pose = _finite(message.mano_pose_rotvec, 48)
                shape = _finite(message.shape_params, 10)
                _finite(message.root_quat_xyzw, 4)
                source_ns, wall = _stamp_ns(message), self._wall()
                if (source_mode in ("live", "replay") and 0 < source_ns <= wall
                        and wall - source_ns <= int(self.config.stale_after_s * 1e9)):
                    # Source MANO already includes root rotation. Forward its
                    # exact parameters, without re-solving or rotating again.
                    preview["mano"] = dict(pose=pose, shape=shape, hand=message.hand,
                                           source_realtime_ns=source_ns, source_mode=source_mode,
                                           **_mano_identity(message))
                    # The optional 21-point geometry can be absent even when
                    # an actual MANO representation is available to render.
                    preview["valid"] = True
            detail = (f"{message.quality}: {message.reason}; calibration={message.calibration_version}; "
                      f"session={message.session_id}; local reference only; connection/calibration progress unavailable.")
            return preview, valid, detail[:1024], False, (message.session_id, message.calibration_version,
                                                          message.config_sha256, message.model_rest_sha256)
        rows, cols, schema = int(message.rows), int(message.cols), int(message.schema_version)
        if (schema, rows, cols) not in ((1, 80, 80), (2, 20, 20)):
            raise ValueError("unsupported tactile grid/schema")
        if len(message.adc) != rows * cols or len(message.valid_mask) != rows * cols:
            raise ValueError("tactile array length mismatch")
        values = _finite([message.adc[r * cols + c] for r in range(20) for c in range(20)], 400)
        masks = [bool(message.valid_mask[r * cols + c]) for r in range(20) for c in range(20)]
        if len(message.block_realtime_ns) != rows * 8:
            raise ValueError("tactile block clock length mismatch")
        # Each physical column is a separate column group. Never make a stale
        # rolling cell fresh merely because another block was updated.
        wall = self._wall()
        max_age = min(int(message.max_age_ns), int(self.config.stale_after_s * 1e9))
        for r in range(20):
            for c in range(20):
                stamp = int(message.block_realtime_ns[r * 8 + c % 8])
                masks[r*20+c] &= bool(0 < stamp <= wall + 500_000_000 and 0 <= wall - stamp <= max_age)
        connected, time_valid = bool(message.connected), bool(message.time_valid)
        if not connected or not time_valid:
            masks = [False] * 400
        valid = connected and time_valid and any(masks)
        preview = dict(values=values, valid_mask=masks, rows=20, cols=20)
        return preview, valid, f"{message.reason}; valid cells={sum(masks)}/400; rolling snapshot, not simultaneous scan.", not connected, (message.session_id, message.stream_epoch)

    def observe_capture(self, message):
        with self._lock:
            try:
                source = _stamp_ns(message)
                if not 0 < source <= self._wall() + 500_000_000:
                    self.event("warning", "Ignored CaptureStatus with unavailable/future source timestamp", key="capture-time")
                    return
                state = int(message.state)
                if state not in range(7):
                    raise ValueError("unknown CaptureStatus state")
                if 0 < source < self._capture_source_ns:
                    return
                session = str(message.session_id)
                now = self._mono()
                if state == 3 and (self._capture is None or self._capture[0:2] != (3, session)):
                    self._recording_started_ns = now
                elif self._capture is not None and session != self._capture[1]:
                    self._recording_started_ns = None
                self._capture = (state, session, str(message.detail)[:1024], bool(message.recorder_alive), int(message.disk_free_bytes))
                self._capture_source_ns, self._capture_received_ns = source, now
            except (AttributeError, TypeError, ValueError) as exc:
                self.event("warning", f"Malformed CaptureStatus: {exc}", key="capture")

    def set_camera_preview(self, device_id, source_ns, preview):
        with self._lock:
            state = self._devices[device_id]
            if (self._configs[device_id].kind != "camera" or not 0 < source_ns <= self._wall() + 500_000_000
                    or source_ns < state.preview_source_ns):
                return
            state.preview, state.preview_source_ns = preview, source_ns

    def provenance(self):
        """Bounded identities observed in messages, never hardware discovery."""
        with self._lock:
            now, wall = self._mono(), self._wall()
            result = []
            names = {"tracker": ("tracker_id",), "glove": ("session_id", "calibration_version", "config_sha256", "model_rest_sha256"),
                     "tactile": ("session_id", "stream_epoch"), "camera": ()}
            for identifier, state in self._devices.items():
                config = self._configs[identifier]
                age = None if state.received_ns is None else max(0., (now-state.received_ns)/1e9)
                identity = dict(zip(names[config.kind], state.identity)) if state.identity is not None else None
                fresh = (age is not None and age <= self.config.stale_after_s and state.source_order_ns > 0
                         and state.source_ns == state.source_order_ns and -.5 <= (wall-state.source_order_ns)/1e9 <= self.config.stale_after_s)
                result.append(dict(device_id=identifier, kind=config.kind, side=config.side, topic=config.topic,
                    identity=identity, identity_status="observed" if identity else "unknown", fresh=fresh,
                    receive_age_s=age, observed_source_realtime_ns=state.source_order_ns or None,
                    received_monotonic_ns=state.received_ns,
                    limitation="Message identity only; camera serial and unreported hardware identities remain unknown."))
            return result

    def topic_health(self):
        """Cheap latest-sample readiness; do not copy previews or query the disk.

        Camera valid_hz is intentionally absent in the UI, so it must not be
        used as a readiness boolean. Likewise a previous valid sample must not
        hide a newly invalid pose just because it remains in the rate window.
        """
        with self._lock:
            now, wall = self._mono(), self._wall()
            maximum = int(self.config.stale_after_s * 1e9)
            result = {}
            for identity, state in self._devices.items():
                result[self._configs[identity].topic] = bool(
                    state.valid and not state.explicitly_disconnected
                    and state.received_ns is not None and 0 <= now - state.received_ns <= maximum
                    and state.source_ns > 0 and -.5e9 <= wall - state.source_ns <= maximum)
            result[self.config.capture_status_topic] = bool(
                self._capture is not None and self._capture_received_ns is not None
                and 0 <= now - self._capture_received_ns <= maximum
                and self._capture_source_ns > 0 and -.5e9 <= wall - self._capture_source_ns <= maximum)
            return result

    def snapshot(self):
        # Disk queries must not hold the state lock used by ROS callbacks.
        try:
            free = int(self._disk(self.config.disk_path).free)
        except OSError:
            free = None
        with self._lock:
            now, wall = self._mono(), self._wall()
            devices = []
            camera_preview_budget = 512 * 1024
            for identity, state in self._devices.items():
                config = self._configs[identity]
                for queue in (state.arrivals, state.valid_arrivals):
                    while queue and queue[0] <= now - 1_000_000_000:
                        queue.popleft()
                age = None if state.received_ns is None else max(0., (now-state.received_ns)/1e9)
                source_age = (wall-state.source_ns)/1e9 if state.source_ns > 0 else None
                fresh_source = source_age is not None and -.5 <= source_age <= self.config.stale_after_s
                status = "unknown" if age is None else ("offline" if state.explicitly_disconnected or age > self.config.offline_after_s
                         else "stale" if age > self.config.stale_after_s or not fresh_source else "online")
                preview = deepcopy(state.preview)
                if config.kind == "camera" and (status != "online" or not 0 <= wall-state.preview_source_ns <= int(self.config.stale_after_s*1e9)):
                    preview = {}
                if config.kind == "camera" and preview:
                    size = len(preview.get("data_b64", ""))
                    if size > camera_preview_budget:
                        preview = {}
                    else:
                        camera_preview_budget -= size
                if "valid_mask" in preview:
                    preview["valid_mask"] = [bool(valid and 0 <= wall-stamp <= state.cell_max_age_ns)
                                             for valid, stamp in zip(preview["valid_mask"], state.cell_stamps)]
                if status != "online" or not state.valid:
                    if "valid" in preview:
                        preview["valid"] = False
                    preview.pop("mano", None)
                    if "valid_mask" in preview:
                        preview["valid_mask"] = [False] * 400
                if "mano" in preview and (source_age is None or source_age < 0):
                    # Keep the old snapshot clock tolerance for legacy views,
                    # but never export a future MANO pose after a wall-clock
                    # adjustment either.
                    preview.pop("mano")
                detail = state.detail
                if status == "offline" and not state.explicitly_disconnected:
                    detail = "Topic silent; physical connection unknown. " + detail
                devices.append(dict(id=identity, label=config.label, kind=config.kind, side=config.side,
                                    status=status, topic=config.topic, receive_hz=float(len(state.arrivals)),
                                    valid_hz=None if config.kind == "camera" else float(len(state.valid_arrivals)),
                                    age_s=age, detail=detail[:1024], preview=preview))
            recording = dict(state="UNAVAILABLE", session_id="", elapsed_s=0., duration_s=0.,
                             detail="Recording control not integrated. No fresh CaptureStatus received.")
            if (self._capture is not None and now-self._capture_received_ns <= int(self.config.offline_after_s*1e9)
                    and 0 < self._capture_source_ns <= wall + 500_000_000
                    and wall-self._capture_source_ns <= int(self.config.offline_after_s*1e9)):
                state, session, detail, alive, disk = self._capture
                mapped = {0: "IDLE", 3: "RECORDING", 5: "COMPLETE"}.get(state, "UNAVAILABLE")
                if state == 3 and not alive:
                    mapped = "UNAVAILABLE"
                recording.update(state=mapped, session_id=session,
                    detail=f"Read-only external CaptureStatus state={state}, recorder_alive={alive}. {detail} Recording control unavailable; duration unknown.")
                # The external message has no actual start/duration. Never
                # fabricate elapsed time from this manager's first receipt.
            self._sequence += 1
            return dict(mode="live", instance_id=self.instance_id, sequence=self._sequence,
                        recording=recording, devices=devices, events=deepcopy(list(reversed(self._events))),
                        resources=dict(disk_free_bytes=free), capabilities=dict(record=False, device_control=False))
