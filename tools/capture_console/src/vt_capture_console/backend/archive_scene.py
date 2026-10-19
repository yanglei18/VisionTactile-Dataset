"""Read-only projection of one archive playhead into the shared spatial view.

This adapter has no clock, reader, calibration lookup, or device connection.
Only recorded Tracker poses in an explicitly common frame are drawable. Images
alone do not establish a camera pose, frustum, or depth reconstruction.
"""
from __future__ import annotations

import copy
import math
from collections import deque
from collections.abc import Mapping


def _stamp(value):
    return value if type(value) is int and 0 <= value <= 2**63 - 1 else None


def _vector(value, length):
    try:
        result = [float(component) for component in value]
        return result if len(result) == length and all(math.isfinite(component) for component in result) else None
    except (TypeError, ValueError, OverflowError):
        return None


class ArchiveSceneBackend:
    """SpatialPage contract driven only by TopicReplayBackend.snapshot()."""

    mode = "replay"
    cloud_backend = None

    def __init__(self):
        self._generation = None
        self._seek_revision = None
        self._last_time = None
        self._last_frame = None
        self._trails = {}
        self._sequence = 0
        self._status = {}
        self.clear()

    def clear(self):
        self._generation = self._seek_revision = self._last_time = self._last_frame = None
        self._trails.clear()
        self._sequence += 1
        self._frame = self._scene(0, "unknown", [], [], "empty")
        self._status = dict(state="waiting", message="选择录制包后，空间视图显示具有共同坐标系的历史 Tracker 位姿。")

    def _scene(self, time_ns, world_frame, trackers, diagnostics, identity):
        return dict(manager_instance_id="archive-" + str(identity)[:128], scene_epoch="archive",
                    sequence=self._sequence, mode="replay", world_frame=world_frame,
                    time_epoch="archive", coordinate_epoch=world_frame,
                    sync_epoch="archive", config_revision="recorded-only", geometry_revision="unavailable",
                    time_mode="monitor", time_basis="bag_playhead", scene_time_ns=time_ns,
                    generated_realtime_ns=time_ns, map_identity=None,
                    display_from_world=dict(translation_m=[0., 0., 0.], quaternion_xyzw=[0., 0., 0., 1.]),
                    trackers=trackers, cameras=[], diagnostics=diagnostics)

    def update(self, snapshot, metadata=None):
        """Apply the exact snapshot used by the archive's other preview tiles."""
        snapshot = snapshot if isinstance(snapshot, Mapping) else {}
        metadata = metadata if isinstance(metadata, Mapping) else {}
        time_ns = _stamp(snapshot.get("time_ns")) or 0
        generation = (snapshot.get("generation"), snapshot.get("path"))
        seek_revision = snapshot.get("seek_revision")
        if (generation != self._generation or seek_revision != self._seek_revision
                or (self._last_time is not None and time_ns < self._last_time)):
            self._trails.clear()
            self._last_frame = None
        self._generation, self._seek_revision, self._last_time = generation, seek_revision, time_ns
        frames = snapshot.get("frames", {})
        frames = frames if isinstance(frames, Mapping) else {}
        topics = [item for item in metadata.get("topics", ()) if isinstance(item, Mapping)
                  and (item.get("kind") == "tracker" or item.get("type") == "vt_tracker_msgs/msg/TrackerSample")]
        known = {item.get("name") for item in topics}
        for name, frame in frames.items():
            device = frame.get("device") if isinstance(frame, Mapping) else None
            if name not in known and isinstance(device, Mapping) and device.get("kind") == "tracker":
                topics.append(dict(name=name, kind="tracker"))
        topics = topics[:32]
        trackers, diagnostics = [], []
        candidates = []
        for topic in topics:
            name = topic.get("name")
            if not isinstance(name, str) or not name:
                continue
            frame = frames.get(name, {})
            frame = frame if isinstance(frame, Mapping) else {}
            device = frame.get("device") or {}
            device = device if isinstance(device, Mapping) else {}
            preview = device.get("preview") or {}
            preview = preview if isinstance(preview, Mapping) else {}
            sample_ns = _stamp(frame.get("sample_ns"))
            frame_id = device.get("frame_id", preview.get("frame_id", frame.get("frame_id")))
            frame_id = frame_id if isinstance(frame_id, str) and frame_id.strip() and len(frame_id) <= 256 else None
            position = _vector(preview.get("position"), 3)
            rotation = _vector(preview.get("orientation"), 4)
            pose_valid = (preview.get("valid") is True and position is not None and rotation is not None
                          and .9 <= math.hypot(*rotation) <= 1.1
                          and sample_ns is not None and sample_ns <= time_ns)
            role = device.get("role", frame.get("role"))
            role = role if isinstance(role, str) and role else next(
                (value for value in ("left_wrist", "right_wrist", "torso") if value in name), name)
            tracker_id = device.get("tracker_id", frame.get("tracker_id"))
            tracker_id = tracker_id if isinstance(tracker_id, str) and tracker_id else None
            tracker_id = tracker_id[:256] if tracker_id else None
            source_ns = _stamp(device.get("source_realtime_ns", preview.get("source_realtime_ns", frame.get("source_realtime_ns"))))
            age_s = (time_ns - sample_ns) / 1e9 if sample_ns is not None and sample_ns <= time_ns else None
            reason = ("该 Tracker 话题没有录制消息。" if topic.get("count") == 0 else
                      "该时刻没有有效的历史 Tracker 位姿。")
            entity = dict(id=name[:256], label=name[:256], role=role[:256], tracker_id=tracker_id,
                          frame_id=frame_id or "—", source_realtime_ns=source_ns,
                          bag_timestamp_ns=sample_ns, position_m=None, quaternion_xyzw=None,
                          state="waiting", valid=False, age_s=age_s, reason=reason, trail_segments=[])
            trackers.append(entity)
            if pose_valid:
                candidates.append((entity, frame_id, position, rotation, sample_ns))

        coordinate_frames = {item[1] for item in candidates if item[1] is not None}
        world_frame = next(iter(coordinate_frames)) if len(coordinate_frames) == 1 else "unknown"
        if world_frame != self._last_frame:
            self._trails.clear()
        self._last_frame = world_frame
        mixed_frames = len(coordinate_frames) > 1
        for entity, frame_id, position, rotation, sample_ns in candidates:
            if not frame_id or mixed_frames:
                entity.update(state="invalid", registration_state="frame_mismatch",
                              reason="Tracker 坐标系缺失或不同，未将这些位姿拼接到同一空间。")
                continue
            entity.update(position_m=position, quaternion_xyzw=rotation, valid=True,
                          state="delayed" if entity["age_s"] > 1. else "fresh",
                          reason="包内历史位姿；各话题保留各自采样时刻，未做插值配准。")
            history = self._trails.setdefault(entity["id"], deque(maxlen=128))
            if entity["age_s"] <= 1. and (not history or history[-1][0] != sample_ns):
                history.append((sample_ns, position))
            while history and time_ns - history[0][0] > 5_000_000_000:
                history.popleft()
            segments, segment, previous = [], [], None
            for stamp, point in history:
                if previous is not None and stamp - previous > 250_000_000:
                    if segment:
                        segments.append(segment)
                    segment = []
                segment.append(point)
                previous = stamp
            if segment:
                segments.append(segment)
            entity["trail_segments"] = segments[-90:]
        for entity in trackers:
            if not entity["valid"]:
                self._trails.pop(entity["id"], None)

        visible_count = sum(entity["valid"] for entity in trackers)
        coordinate_problem = mixed_frames or any(item[1] is None for item in candidates)
        if coordinate_problem:
            diagnostics.append("Tracker 坐标系缺失或不同，未将这些位姿拼接到同一空间。")
        diagnostics.append("兼容空间回放仅显示原始 Tracker 位姿；尚未重建相机位姿、视锥和点云。")
        if not visible_count:
            message = "该时刻无空间数据；已有数据可在“话题详情”查看。"
        else:
            message = "按档案时间轴显示历史 Tracker 位姿；轨迹仅覆盖本次连续播放经过的样本，拖动时间轴后重置。"
        summary = ("Tracker 坐标系缺失或不同，未将这些位姿拼接到同一空间。" if coordinate_problem else
                   "历史 Tracker 位姿" if visible_count else message)
        diagnostics.insert(0, message)
        self._sequence += 1
        self._frame = self._scene(time_ns, world_frame, trackers, diagnostics, snapshot.get("generation", "empty"))
        self._status = dict(state="ready" if visible_count else "waiting", message=summary)
        return self.latest()

    def latest(self):
        return copy.deepcopy(self._frame)

    def status(self):
        return dict(self._status)

    def set_interest(self, _viewer_id, _layers):
        pass

    def close(self):
        self.clear()
