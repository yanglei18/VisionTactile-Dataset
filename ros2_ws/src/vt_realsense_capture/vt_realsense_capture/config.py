from __future__ import annotations

import math
import re
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, TypeVar, cast

import yaml


ColorModule = Literal["depth_module", "rgb_camera"]
_COLOR_MODULES = frozenset({"depth_module", "rgb_camera"})
_MODEL_COLOR_MODULES: dict[str, ColorModule] = {
    "D405": "depth_module",
    "D436": "rgb_camera",
}
_ROOT_KEYS = frozenset({"cameras", "stream", "recording"})
_CAMERA_KEYS = frozenset(
    {"name", "model_token", "serial", "firmware", "asic_serial", "color_module"}
)
_STREAM_KEYS = frozenset(
    {
        "width",
        "height",
        "fps",
        "color_format",
        "color_encoding",
        "depth_format",
        "depth_encoding",
    }
)
_RECORDING_KEYS = frozenset(
    {
        "max_bag_duration_seconds",
        "max_bag_size_bytes",
        "max_cache_size_bytes",
        "additional_streams",
    }
)
_ADDITIONAL_STREAM_KEYS = frozenset({"topic", "type"})
_T = TypeVar("_T")


@dataclass(frozen=True)
class CameraConfig:
    name: str
    model_token: str
    serial: str
    firmware: str
    asic_serial: str
    color_module: ColorModule


@dataclass(frozen=True)
class AdditionalStreamConfig:
    topic: str
    type_name: str


@dataclass(frozen=True)
class CaptureConfig:
    cameras: tuple[CameraConfig, ...]
    width: int
    height: int
    fps: int
    color_format: str
    color_encoding: str
    depth_format: str
    depth_encoding: str
    max_bag_duration_seconds: int
    max_bag_size_bytes: int
    max_cache_size_bytes: int
    additional_streams: tuple[AdditionalStreamConfig, ...]


_CAMERA_NAME_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_ABSOLUTE_TOPIC_PATTERN = re.compile(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+")
_ROS_TYPE_PATTERN = re.compile(
    r"[A-Za-z_][A-Za-z0-9_]*/msg/[A-Za-z_][A-Za-z0-9_]*"
)
_EXPECTED_CAMERA_TOPOLOGY = (
    ("D405", "depth_module"),
    ("D405", "depth_module"),
    ("D436", "rgb_camera"),
)
_EXPECTED_STREAM = (1280, 720, 30, "RGB8", "rgb8", "Z16", "16UC1")
_EXPECTED_RECORDING = (300, 137438953472, 1073741824)


def _mapping(value: object, context: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{context} must be a mapping")
    return value


def _reject_unexpected_keys(
    values: Mapping[str, object], expected: frozenset[str], context: str
) -> None:
    unexpected = set(values) - expected
    if unexpected:
        names = ", ".join(sorted(str(key) for key in unexpected))
        raise ValueError(f"unexpected {context} key(s): {names}")


def _required(
    values: Mapping[str, object], key: str, expected_type: type[_T], context: str
) -> _T:
    try:
        value = values[key]
    except KeyError as exc:
        raise ValueError(f"missing {context}.{key}") from exc
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"{context}.{key} must be finite")
    if type(value) is not expected_type:
        raise ValueError(f"{context}.{key} must be {expected_type.__name__}")
    return cast(_T, value)


def _required_nonempty_string(
    values: Mapping[str, object], key: str, context: str
) -> str:
    value = _required(values, key, str, context)
    if not value or value != value.strip():
        raise ValueError(f"{context}.{key} must be a non-empty trimmed string")
    return value


def _load_camera(value: object, index: int) -> CameraConfig:
    context = f"cameras[{index}]"
    camera = _mapping(value, context)
    _reject_unexpected_keys(camera, _CAMERA_KEYS, context)
    name = _required(camera, "name", str, context)
    if _CAMERA_NAME_PATTERN.fullmatch(name) is None:
        raise ValueError(f"{context}.name must be a ROS name token")
    name = _required_nonempty_string(camera, "name", context)
    model_token = _required_nonempty_string(camera, "model_token", context)
    color_module = _required(camera, "color_module", str, context)
    if color_module not in _COLOR_MODULES:
        raise ValueError(f"unknown color_module: {color_module}")
    expected_color_module = _MODEL_COLOR_MODULES.get(model_token)
    if expected_color_module is not None and color_module != expected_color_module:
        raise ValueError(
            f"{model_token} cameras require color_module {expected_color_module}"
        )
    return CameraConfig(
        name=name,
        model_token=model_token,
        serial=_required_nonempty_string(camera, "serial", context),
        firmware=_required_nonempty_string(camera, "firmware", context),
        asic_serial=_required_nonempty_string(camera, "asic_serial", context),
        color_module=cast(ColorModule, color_module),
    )


def _reject_duplicates(cameras: tuple[CameraConfig, ...]) -> None:
    identity_fields = {
        "name": [camera.name for camera in cameras],
        "serial": [camera.serial for camera in cameras],
        "asic_serial": [camera.asic_serial for camera in cameras],
    }
    for field, values in identity_fields.items():
        if len(values) != len(set(values)):
            raise ValueError(f"duplicate {field} in camera configuration")


def _load_additional_stream(value: object, index: int) -> AdditionalStreamConfig:
    context = f"recording.additional_streams[{index}]"
    stream = _mapping(value, context)
    _reject_unexpected_keys(stream, _ADDITIONAL_STREAM_KEYS, context)
    topic = _required_nonempty_string(stream, "topic", context)
    type_name = _required_nonempty_string(stream, "type", context)
    if _ABSOLUTE_TOPIC_PATTERN.fullmatch(topic) is None:
        raise ValueError(f"{context}.topic must be an absolute ROS topic")
    if _ROS_TYPE_PATTERN.fullmatch(type_name) is None:
        raise ValueError(f"{context}.type must be a ROS message type")
    return AdditionalStreamConfig(topic=topic, type_name=type_name)


def _load_additional_streams(recording: Mapping[str, object]) -> tuple[AdditionalStreamConfig, ...]:
    values = recording.get("additional_streams")
    if not isinstance(values, list):
        raise ValueError("recording.additional_streams must be a list")
    streams = tuple(
        _load_additional_stream(value, index)
        for index, value in enumerate(values)
    )
    topics = [stream.topic for stream in streams]
    if len(topics) != len(set(topics)):
        raise ValueError("duplicate topic in recording.additional_streams")
    return tuple(sorted(streams, key=lambda stream: stream.topic))


def _validate_capture_contract(config: CaptureConfig) -> None:
    if len(config.cameras) != 3:
        raise ValueError("capture configuration requires exactly three cameras")
    topology = tuple(
        (camera.model_token, camera.color_module) for camera in config.cameras
    )
    if topology != _EXPECTED_CAMERA_TOPOLOGY:
        raise ValueError(
            "camera topology must be D405, D405, D436 in that order"
        )
    stream = (
        config.width,
        config.height,
        config.fps,
        config.color_format,
        config.color_encoding,
        config.depth_format,
        config.depth_encoding,
    )
    if stream != _EXPECTED_STREAM:
        raise ValueError(
            "stream configuration does not match fixed capture contract"
        )
    recording = (
        config.max_bag_duration_seconds,
        config.max_bag_size_bytes,
        config.max_cache_size_bytes,
    )
    if recording != _EXPECTED_RECORDING:
        raise ValueError(
            "recording configuration does not match fixed capture contract"
        )
    from .bag_contract import expected_topics

    core_topics = set(
        expected_topics(tuple(camera.name for camera in config.cameras))
    )
    for stream in config.additional_streams:
        if stream.topic in core_topics:
            raise ValueError(
                f"additional stream duplicates core topic: {stream.topic}"
            )


def load_config(path: Path) -> CaptureConfig:
    """Load and validate the capture configuration at *path*."""

    try:
        document = yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML configuration: {exc}") from exc

    root = _mapping(document, "configuration")
    _reject_unexpected_keys(root, _ROOT_KEYS, "configuration")
    camera_values = root.get("cameras")
    if not isinstance(camera_values, list):
        raise ValueError("cameras must be a list")
    cameras = tuple(
        _load_camera(camera_value, index)
        for index, camera_value in enumerate(camera_values)
    )
    _reject_duplicates(cameras)

    stream = _mapping(root.get("stream"), "stream")
    _reject_unexpected_keys(stream, _STREAM_KEYS, "stream")
    fps = _required(stream, "fps", int, "stream")
    if fps != 30:
        raise ValueError("stream profile must run at 30 Hz")

    recording = _mapping(root.get("recording"), "recording")
    _reject_unexpected_keys(recording, _RECORDING_KEYS, "recording")
    config = CaptureConfig(
        cameras=cameras,
        width=_required(stream, "width", int, "stream"),
        height=_required(stream, "height", int, "stream"),
        fps=fps,
        color_format=_required(stream, "color_format", str, "stream"),
        color_encoding=_required(stream, "color_encoding", str, "stream"),
        depth_format=_required(stream, "depth_format", str, "stream"),
        depth_encoding=_required(stream, "depth_encoding", str, "stream"),
        max_bag_duration_seconds=_required(
            recording, "max_bag_duration_seconds", int, "recording"
        ),
        max_bag_size_bytes=_required(
            recording, "max_bag_size_bytes", int, "recording"
        ),
        max_cache_size_bytes=_required(
            recording, "max_cache_size_bytes", int, "recording"
        ),
        additional_streams=_load_additional_streams(recording),
    )
    _validate_capture_contract(config)
    return config
