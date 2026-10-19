"""Bounded software-time grouping of complete camera frames.

Input timestamps share audited host realtime. Reception/lease clocks must use
the same local monotonic clock. This matcher does not claim exposure or hardware
synchronization. Manager owns generation changes; mismatched input is rejected
until ``reset``. Raw images remain opaque caller-owned payloads.
"""

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass, field
import json
from types import MappingProxyType

from .timeline import PoseBuffer
from .types import InsertResult, PoseSelection, timestamp_ns
from .scene_protocol import _bounded_json, _generation, _string


@dataclass(frozen=True)
class CameraFrameSample:
    camera_id: str
    frame_key: str | Mapping
    source_realtime_ns: int
    received_monotonic_ns: int
    time_epoch: str
    coordinate_epoch: str
    geometry_revision: str | int
    payload: object = field(default=None, compare=False, repr=False)
    _key: str = field(init=False, repr=False)

    def __post_init__(self):
        _string(self.camera_id, 'camera_id')
        timestamp_ns(self.source_realtime_ns, 'source_realtime_ns')
        timestamp_ns(self.received_monotonic_ns, 'received_monotonic_ns')
        for key in ('time_epoch', 'coordinate_epoch', 'geometry_revision'):
            _generation(getattr(self, key), key)
        if isinstance(self.frame_key, str):
            _string(self.frame_key, 'frame_key', 1024)
            key = self.frame_key
        elif isinstance(self.frame_key, Mapping) and self.frame_key:
            frame_key = dict(self.frame_key)
            _bounded_json(frame_key)
            key = json.dumps(frame_key, sort_keys=True, separators=(',', ':'), allow_nan=False)
            if len(key) > 4096:
                raise ValueError('frame key exceeds limit')
            object.__setattr__(self, 'frame_key', MappingProxyType(json.loads(key)))
        else:
            raise ValueError('frame_key must be a nonempty string or mapping')
        object.__setattr__(self, '_key', key)

    @property
    def generation(self):
        return self.time_epoch, self.coordinate_epoch, self.geometry_revision


@dataclass(frozen=True)
class FrameSelection:
    frames: tuple[CameraFrameSample, ...]
    scene_time_ns: int
    reference_camera: str
    sync_epoch: int
    time_epoch: str
    coordinate_epoch: str
    geometry_revision: str | int
    residuals_ns: Mapping[str, int]
    missing: tuple[str, ...]
    poses: Mapping[str, PoseSelection]
    unavailable_poses: tuple[str, ...]
    reason: str = ''

    @property
    def complete(self):
        return not self.missing and not self.unavailable_poses


class FrameSynchronizer:
    def __init__(self, camera_ids, *, reference_camera=None,
                 max_delta_ns=20_000_000, wait_ns=100_000_000,
                 failover_ns=250_000_000, restore_ns=1_000_000_000,
                 max_frames_per_camera=6, mode='live', tracker_buffers=None,
                 tracker_gap_ns=50_000_000):
        camera_ids = tuple(camera_ids)
        if not camera_ids or len(camera_ids) > 32 or len(set(camera_ids)) != len(camera_ids):
            raise ValueError('camera_ids must contain 1 to 32 unique names')
        for camera in camera_ids:
            _string(camera, 'camera_id')
        if reference_camera is None:
            reference_camera = camera_ids[0]
        if reference_camera not in camera_ids:
            raise ValueError('reference_camera must belong to camera_ids')
        if mode not in ('live', 'replay'):
            raise ValueError('mode must be live or replay')
        for key, value in (('max_delta_ns', max_delta_ns), ('wait_ns', wait_ns),
                           ('failover_ns', failover_ns), ('restore_ns', restore_ns),
                           ('tracker_gap_ns', tracker_gap_ns)):
            timestamp_ns(value, key)
        if failover_ns == 0 or restore_ns == 0:
            raise ValueError('failover and restoration intervals must be positive')
        if type(max_frames_per_camera) is not int or not 1 <= max_frames_per_camera <= 64:
            raise ValueError('max_frames_per_camera must be between 1 and 64')
        buffers = dict(tracker_buffers or {})
        if set(buffers)-set(camera_ids) or any(not isinstance(value, PoseBuffer) for value in buffers.values()):
            raise ValueError('tracker_buffers must map configured cameras to PoseBuffer')
        self.camera_ids = camera_ids
        self.preferred_reference_camera = reference_camera
        self.active_reference_camera = reference_camera
        self.mode = mode
        self.max_delta_ns, self.wait_ns = max_delta_ns, wait_ns
        self.failover_ns, self.restore_ns = failover_ns, restore_ns
        self.max_frames_per_camera = max_frames_per_camera
        self.tracker_buffers, self.tracker_gap_ns = buffers, tracker_gap_ns
        self.sync_epoch = 0
        self._generation = None
        self._initialize_history()

    def _initialize_history(self):
        self._frames = {camera: [] for camera in self.camera_ids}
        self._seen_keys = {camera: deque(maxlen=128) for camera in self.camera_ids}
        self._last_source = {}
        self._last_received = {}
        self._consumed = {}
        self._first_received = None
        self._preferred_stable_since = None
        self._pending = None
        self._last_scene_time = None
        self._last_poll_ns = None

    def reset(self, *, time_epoch, coordinate_epoch, geometry_revision):
        for name, value in (('time_epoch', time_epoch), ('coordinate_epoch', coordinate_epoch),
                            ('geometry_revision', geometry_revision)):
            _generation(value, name)
        self._generation = (time_epoch, coordinate_epoch, geometry_revision)
        self._initialize_history()
        self.active_reference_camera = self.preferred_reference_camera
        self.sync_epoch += 1

    def append(self, frame: CameraFrameSample) -> InsertResult:
        if not isinstance(frame, CameraFrameSample):
            raise TypeError('frame must be CameraFrameSample')
        camera = frame.camera_id
        if camera not in self._frames:
            return InsertResult(False, 'unknown_camera')
        if self._generation is None:
            self._generation = frame.generation
        if frame.generation != self._generation:
            return InsertResult(False, 'generation_mismatch')
        frames = self._frames[camera]
        if frame._key in self._seen_keys[camera] or any(item.source_realtime_ns == frame.source_realtime_ns for item in frames):
            return InsertResult(False, 'duplicate')
        if frame.source_realtime_ns <= self._consumed.get(camera, -1):
            return InsertResult(False, 'already_consumed')
        if self._last_scene_time is not None and frame.source_realtime_ns < self._last_scene_time-self.max_delta_ns:
            return InsertResult(False, 'too_old')
        if self._first_received is None:
            self._first_received = frame.received_monotonic_ns
        if frame.source_realtime_ns > self._last_source.get(camera, -1):
            previous_received = self._last_received.get(camera)
            # A late packet cannot rewrite the monotonic freshness clock.
            received = max(frame.received_monotonic_ns, previous_received or 0)
            self._last_received[camera] = received
            self._last_source[camera] = frame.source_realtime_ns
            if camera == self.preferred_reference_camera:
                if previous_received is None or received-previous_received >= self.failover_ns or self._preferred_stable_since is None:
                    self._preferred_stable_since = received
        index = bisect_left([item.source_realtime_ns for item in frames], frame.source_realtime_ns)
        frames.insert(index, frame)
        self._seen_keys[camera].append(frame._key)
        del frames[:max(0, len(frames)-self.max_frames_per_camera)]
        return InsertResult(True)

    def _fresh(self, camera, now):
        last = self._last_received.get(camera)
        return last is not None and 0 <= now-last < self.failover_ns

    def _switch_reference(self, camera):
        if camera == self.active_reference_camera:
            return
        self.active_reference_camera = camera
        self.sync_epoch += 1
        self._pending = None
        if camera != self.preferred_reference_camera:
            self._preferred_stable_since = None
        # Drop stale candidates, but retain usable complete frames as well as
        # the owner-held Tracker history across a reference-camera switch.
        if self._last_scene_time is not None:
            for frames in self._frames.values():
                frames[:] = [item for item in frames if item.source_realtime_ns > self._last_scene_time]

    def _update_reference(self, now):
        if self.mode == 'replay' or self._first_received is None:
            return
        preferred = self.preferred_reference_camera
        if self.active_reference_camera != preferred and self._fresh(preferred, now):
            stable = self._preferred_stable_since
            if stable is not None and now-stable >= self.restore_ns:
                self._switch_reference(preferred)
                return
        current = self.active_reference_camera
        last = self._last_received.get(current, self._first_received)
        if now-last < self.failover_ns:
            return
        for candidate in self.camera_ids:
            if self._fresh(candidate, now) and any(
                self._last_scene_time is None or item.source_realtime_ns > self._last_scene_time
                for item in self._frames[candidate]
            ):
                self._switch_reference(candidate)
                return

    def poll(self, now_monotonic_ns: int, *, scene_time_ns: int | None = None) -> FrameSelection | None:
        timestamp_ns(now_monotonic_ns, 'now_monotonic_ns')
        if self._last_poll_ns is not None and now_monotonic_ns < self._last_poll_ns:
            raise ValueError('poll monotonic clock moved backward; reset required')
        self._last_poll_ns = now_monotonic_ns
        if scene_time_ns is not None:
            timestamp_ns(scene_time_ns, 'scene_time_ns')
            if self.mode != 'replay':
                raise ValueError('explicit timeline is only available in replay')
        if self._generation is None:
            return None
        if self.mode == 'live':
            # Stop producing cloud frames when every input has gone stale.
            # Expire an old pending anchor as well, so recovery never paints
            # its old geometry before the first current frame.
            for frames in self._frames.values():
                frames[:] = [item for item in frames
                             if 0 <= now_monotonic_ns-item.received_monotonic_ns < self.failover_ns]
            if self._pending is not None and now_monotonic_ns-self._pending.received_monotonic_ns >= self.failover_ns:
                self._pending = None
            if not any(self._fresh(camera, now_monotonic_ns) for camera in self.camera_ids):
                return None
        self._update_reference(now_monotonic_ns)
        explicit = scene_time_ns is not None
        if explicit:
            anchor_time, received = scene_time_ns, now_monotonic_ns-self.wait_ns
        else:
            if self._pending is None:
                choices = [item for item in self._frames[self.active_reference_camera]
                           if self._last_scene_time is None or item.source_realtime_ns > self._last_scene_time]
                if not choices:
                    return None
                self._pending = choices[0]
            anchor_time = self._pending.source_realtime_ns
            received = self._pending.received_monotonic_ns
        if self._last_scene_time is not None and anchor_time <= self._last_scene_time:
            return None
        selected = []
        missing = []
        poses = {}
        unavailable_poses = []
        for camera in self.camera_ids:
            candidates = [item for item in self._frames[camera]
                          if abs(item.source_realtime_ns-anchor_time) <= self.max_delta_ns]
            if not explicit and camera == self.active_reference_camera:
                # Its queue may have evicted the pending anchor while waiting.
                candidates = [self._pending]
            if not candidates:
                missing.append(camera)
                continue
            chosen = min(candidates, key=lambda item: (abs(item.source_realtime_ns-anchor_time), item.source_realtime_ns))
            selected.append(chosen)
            if camera in self.tracker_buffers:
                buffer = self.tracker_buffers[camera]
                # Equal host timestamps do not prove equal coordinate/time epochs.
                if buffer.latest is None or (buffer.latest.time_epoch, buffer.latest.coordinate_epoch) != self._generation[:2]:
                    pose = PoseSelection(None, 'unavailable', 'generation_mismatch')
                else:
                    pose = buffer.select(chosen.source_realtime_ns, max_gap_ns=self.tracker_gap_ns)
                poses[camera] = pose
                if pose.pose is None:
                    unavailable_poses.append(camera)
        complete = not missing and not unavailable_poses
        if not complete and now_monotonic_ns-received < self.wait_ns:
            return None
        result = FrameSelection(
            tuple(selected), anchor_time, self.active_reference_camera, self.sync_epoch,
            *self._generation,
            MappingProxyType({item.camera_id: item.source_realtime_ns-anchor_time for item in selected}),
            tuple(missing), MappingProxyType(poses), tuple(unavailable_poses),
            '' if complete else 'wait_expired',
        )
        for item in selected:
            self._consumed[item.camera_id] = item.source_realtime_ns
        for camera, frames in self._frames.items():
            frames[:] = [item for item in frames
                         if item.source_realtime_ns > self._consumed.get(camera, -1)
                         and item.source_realtime_ns >= anchor_time-self.max_delta_ns]
        self._pending = None
        self._last_scene_time = anchor_time
        return result
