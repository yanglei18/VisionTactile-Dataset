from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Mapping

import yaml


_ROS_TOPIC = re.compile(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+")
_ROS_TYPE = re.compile(
    r"[A-Za-z_][A-Za-z0-9_]*/msg/[A-Za-z_][A-Za-z0-9_]*"
)
_TOKEN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_FIELD_PATH = re.compile(
    r"[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*"
)
_TRACKER_ROLES = frozenset({"left_wrist", "right_wrist", "torso"})


@dataclass(frozen=True)
class CameraConfig:
    name: str
    model: str
    serial: str
    frame_id: str
    tracker_role: str
    color_topic: str
    depth_topic: str
    camera_info_topic: str
    timing_topic: str


@dataclass(frozen=True)
class TrackerConfig:
    role: str
    frame_id: str
    sample_topic: str


@dataclass(frozen=True)
class AdditionalStreamConfig:
    name: str
    topic: str
    type_name: str
    time_source: str
    timestamp_field: str
    strategy: str
    max_delta_ms: float
    required: bool


@dataclass(frozen=True)
class Thresholds:
    max_camera_delta_ms: float
    max_tracker_gap_ms: float
    max_clock_step_ms: float
    min_camera_match_ratio: float
    min_tracker_coverage_ratio: float
    min_required_stream_coverage_ratio: float


@dataclass(frozen=True)
class AlignmentConfig:
    reference_camera: str
    cameras: tuple[CameraConfig, ...]
    trackers: tuple[TrackerConfig, ...]
    additional_streams: tuple[AdditionalStreamConfig, ...]
    thresholds: Thresholds
    world_frame: str

    @property
    def camera_by_name(self) -> Mapping[str, CameraConfig]:
        return {camera.name: camera for camera in self.cameras}

    @property
    def tracker_by_role(self) -> Mapping[str, TrackerConfig]:
        return {tracker.role: tracker for tracker in self.trackers}

    @property
    def required_topics(self) -> tuple[str, ...]:
        topics: list[str] = []
        for camera in self.cameras:
            topics.extend(
                (
                    camera.color_topic,
                    camera.depth_topic,
                    camera.camera_info_topic,
                    camera.timing_topic,
                )
            )
        topics.extend(tracker.sample_topic for tracker in self.trackers)
        topics.extend(
            stream.topic for stream in self.additional_streams if stream.required
        )
        return tuple(topics)


def _mapping(value: object, context: str) -> dict[str, object]:
    if type(value) is not dict:
        raise ValueError(f"{context} must be a mapping")
    return value


def _list(value: object, context: str) -> list[object]:
    if type(value) is not list:
        raise ValueError(f"{context} must be a list")
    return value


def _exact_keys(value: Mapping[str, object], keys: set[str], context: str) -> None:
    if set(value) != keys:
        raise ValueError(
            f"{context} keys mismatch: expected={sorted(keys)} observed={sorted(value)}"
        )


def _text(value: Mapping[str, object], key: str, context: str) -> str:
    result = value.get(key)
    if type(result) is not str or not result or result != result.strip():
        raise ValueError(f"{context}.{key} must be a non-empty trimmed string")
    return result


def _number(value: Mapping[str, object], key: str, context: str) -> float:
    result = value.get(key)
    if type(result) not in {int, float} or not math.isfinite(float(result)):
        raise ValueError(f"{context}.{key} must be finite")
    return float(result)


def _topic(value: Mapping[str, object], key: str, context: str) -> str:
    result = _text(value, key, context)
    if _ROS_TOPIC.fullmatch(result) is None:
        raise ValueError(f"{context}.{key} must be an absolute ROS topic")
    return result


def _load_camera(value: object, index: int) -> CameraConfig:
    context = f"cameras[{index}]"
    item = _mapping(value, context)
    _exact_keys(
        item,
        {
            "name", "model", "serial", "frame_id", "tracker_role",
            "color_topic", "depth_topic", "camera_info_topic", "timing_topic",
        },
        context,
    )
    name = _text(item, "name", context)
    if _TOKEN.fullmatch(name) is None:
        raise ValueError(f"{context}.name must be a ROS token")
    role = _text(item, "tracker_role", context)
    if role not in _TRACKER_ROLES:
        raise ValueError(f"{context}.tracker_role is unsupported")
    return CameraConfig(
        name=name,
        model=_text(item, "model", context),
        serial=_text(item, "serial", context),
        frame_id=_text(item, "frame_id", context),
        tracker_role=role,
        color_topic=_topic(item, "color_topic", context),
        depth_topic=_topic(item, "depth_topic", context),
        camera_info_topic=_topic(item, "camera_info_topic", context),
        timing_topic=_topic(item, "timing_topic", context),
    )


def _load_tracker(value: object, index: int) -> TrackerConfig:
    context = f"trackers[{index}]"
    item = _mapping(value, context)
    _exact_keys(item, {"role", "frame_id", "sample_topic"}, context)
    role = _text(item, "role", context)
    if role not in _TRACKER_ROLES:
        raise ValueError(f"{context}.role is unsupported")
    return TrackerConfig(
        role=role,
        frame_id=_text(item, "frame_id", context),
        sample_topic=_topic(item, "sample_topic", context),
    )


def _load_additional(value: object, index: int) -> AdditionalStreamConfig:
    context = f"additional_streams[{index}]"
    item = _mapping(value, context)
    _exact_keys(
        item,
        {
            "name", "topic", "type", "time_source", "timestamp_field",
            "strategy", "max_delta_ms", "required",
        },
        context,
    )
    name = _text(item, "name", context)
    if _TOKEN.fullmatch(name) is None:
        raise ValueError(f"{context}.name must be a token")
    type_name = _text(item, "type", context)
    if _ROS_TYPE.fullmatch(type_name) is None:
        raise ValueError(f"{context}.type must be a ROS message type")
    time_source = _text(item, "time_source", context)
    if time_source not in {"header_stamp", "field"}:
        raise ValueError(f"{context}.time_source is unsupported")
    strategy = _text(item, "strategy", context)
    if strategy not in {"nearest", "previous"}:
        raise ValueError(f"{context}.strategy is unsupported")
    timestamp_field = _text(item, "timestamp_field", context)
    if time_source == "header_stamp" and timestamp_field != "header.stamp":
        raise ValueError(
            f"{context}.timestamp_field must be header.stamp for header_stamp"
        )
    if _FIELD_PATH.fullmatch(timestamp_field) is None:
        raise ValueError(f"{context}.timestamp_field must be a dotted field path")
    required = item.get("required")
    if type(required) is not bool:
        raise ValueError(f"{context}.required must be bool")
    maximum = _number(item, "max_delta_ms", context)
    if maximum <= 0.0:
        raise ValueError(f"{context}.max_delta_ms must be positive")
    return AdditionalStreamConfig(
        name=name,
        topic=_topic(item, "topic", context),
        type_name=type_name,
        time_source=time_source,
        timestamp_field=timestamp_field,
        strategy=strategy,
        max_delta_ms=maximum,
        required=required,
    )


def load_config(path: str | Path) -> AlignmentConfig:
    source = Path(path)
    try:
        document = yaml.safe_load(source.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML configuration: {exc}") from exc
    root = _mapping(document, "configuration")
    _exact_keys(
        root,
        {
            "schema_version", "reference_camera", "cameras", "trackers",
            "additional_streams", "alignment", "frames",
        },
        "configuration",
    )
    if root.get("schema_version") != 1:
        raise ValueError("unsupported schema_version")
    cameras = tuple(
        _load_camera(item, index)
        for index, item in enumerate(_list(root["cameras"], "cameras"))
    )
    trackers = tuple(
        _load_tracker(item, index)
        for index, item in enumerate(_list(root["trackers"], "trackers"))
    )
    additional = tuple(
        _load_additional(item, index)
        for index, item in enumerate(
            _list(root["additional_streams"], "additional_streams")
        )
    )
    if len(cameras) != 3 or len(trackers) != 3:
        raise ValueError("configuration requires exactly three cameras and trackers")
    for label, values in (
        ("camera name", [camera.name for camera in cameras]),
        ("camera serial", [camera.serial for camera in cameras]),
        ("tracker role", [tracker.role for tracker in trackers]),
        ("additional stream name", [stream.name for stream in additional]),
    ):
        if len(values) != len(set(values)):
            raise ValueError(f"duplicate {label}")
    all_topics = [
        topic
        for camera in cameras
        for topic in (
            camera.color_topic, camera.depth_topic,
            camera.camera_info_topic, camera.timing_topic,
        )
    ] + [tracker.sample_topic for tracker in trackers] + [
        stream.topic for stream in additional
    ]
    if len(all_topics) != len(set(all_topics)):
        raise ValueError("all configured topics must be unique")
    camera_roles = {camera.tracker_role for camera in cameras}
    tracker_roles = {tracker.role for tracker in trackers}
    if camera_roles != tracker_roles or tracker_roles != _TRACKER_ROLES:
        raise ValueError("cameras must bind one-to-one to all Tracker roles")
    reference = _text(root, "reference_camera", "configuration")
    if reference not in {camera.name for camera in cameras}:
        raise ValueError("reference_camera is not configured")
    alignment = _mapping(root["alignment"], "alignment")
    _exact_keys(
        alignment,
        {
            "max_camera_delta_ms", "max_tracker_gap_ms", "max_clock_step_ms",
            "min_camera_match_ratio", "min_tracker_coverage_ratio",
            "min_required_stream_coverage_ratio",
        },
        "alignment",
    )
    thresholds = Thresholds(
        max_camera_delta_ms=_number(alignment, "max_camera_delta_ms", "alignment"),
        max_tracker_gap_ms=_number(alignment, "max_tracker_gap_ms", "alignment"),
        max_clock_step_ms=_number(alignment, "max_clock_step_ms", "alignment"),
        min_camera_match_ratio=_number(
            alignment, "min_camera_match_ratio", "alignment"
        ),
        min_tracker_coverage_ratio=_number(
            alignment, "min_tracker_coverage_ratio", "alignment"
        ),
        min_required_stream_coverage_ratio=_number(
            alignment, "min_required_stream_coverage_ratio", "alignment"
        ),
    )
    if (
        min(
            thresholds.max_camera_delta_ms,
            thresholds.max_tracker_gap_ms,
            thresholds.max_clock_step_ms,
        ) <= 0.0
        or not 0.0 <= thresholds.min_camera_match_ratio <= 1.0
        or not 0.0 <= thresholds.min_tracker_coverage_ratio <= 1.0
        or not 0.0 <= thresholds.min_required_stream_coverage_ratio <= 1.0
    ):
        raise ValueError("alignment thresholds are outside supported ranges")
    frames = _mapping(root["frames"], "frames")
    _exact_keys(frames, {"world"}, "frames")
    return AlignmentConfig(
        reference_camera=reference,
        cameras=cameras,
        trackers=trackers,
        additional_streams=additional,
        thresholds=thresholds,
        world_frame=_text(frames, "world", "frames"),
    )
