from __future__ import annotations

from dataclasses import dataclass
import math
import re
from types import MappingProxyType
from typing import Mapping

import numpy as np

from .errors import DatasetFormatError
from .model import MessageRef, Transform


_TRACKER_ID = re.compile(r"[0-9a-f]{64}")


def _immutable_mapping(value: Mapping[str, object]) -> Mapping[str, object]:
    return MappingProxyType(dict(value))


def _readonly_array(
    value: object,
    *,
    dtype: np.dtype[object] | type[object],
    shape: tuple[int, ...] | None = None,
    context: str,
) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=dtype)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{context} is not a valid array") from error
    if shape is not None and result.shape != shape:
        raise ValueError(f"{context} must have shape {shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{context} must contain finite values")
    result = result.copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class ImageData:
    array: np.ndarray
    encoding: str
    frame_id: str
    source_timestamp_ns: int
    reference: MessageRef

    def __post_init__(self) -> None:
        array = np.asarray(self.array).copy()
        array.setflags(write=False)
        object.__setattr__(self, "array", array)


@dataclass(frozen=True)
class RegionOfInterestData:
    x_offset: int
    y_offset: int
    height: int
    width: int
    do_rectify: bool


@dataclass(frozen=True)
class CameraInfoData:
    camera_name: str
    frame_id: str
    width: int
    height: int
    distortion_model: str
    d: np.ndarray
    k: np.ndarray
    r: np.ndarray
    p: np.ndarray
    binning_x: int
    binning_y: int
    roi: RegionOfInterestData

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "d",
            _readonly_array(
                self.d, dtype=np.float64, context="camera_info.d"
            ),
        )
        for name, shape in (("k", (3, 3)), ("r", (3, 3)), ("p", (3, 4))):
            object.__setattr__(
                self,
                name,
                _readonly_array(
                    getattr(self, name),
                    dtype=np.float64,
                    shape=shape,
                    context=f"camera_info.{name}",
                ),
            )


@dataclass(frozen=True)
class TrackerPose:
    role: str
    tracker_id: str
    timestamp_ns: int
    bracket_gap_ns: int
    before_sequence: int
    after_sequence: int
    world_from_tracker: Transform


@dataclass(frozen=True)
class CameraRecord:
    camera_name: str
    host_realtime_ns: int
    source_timestamp_ns: int
    delta_ns: int
    color: MessageRef
    depth: MessageRef
    timing: MessageRef
    attached_tracker: TrackerPose | None
    world_from_camera: Transform | None


@dataclass(frozen=True)
class AdditionalRecord:
    stream_name: str
    timestamp_ns: int
    delta_ns: int
    message: MessageRef


@dataclass(frozen=True)
class FrameRecord:
    frame_index: int
    reference_camera: str
    reference_time_ns: int
    cameras: Mapping[str, CameraRecord | None]
    trackers: Mapping[str, TrackerPose | None]
    additional_streams: Mapping[str, AdditionalRecord | None]
    quality_flags: tuple[str, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "cameras", _immutable_mapping(self.cameras))
        object.__setattr__(self, "trackers", _immutable_mapping(self.trackers))
        object.__setattr__(
            self,
            "additional_streams",
            _immutable_mapping(self.additional_streams),
        )
        object.__setattr__(self, "quality_flags", tuple(self.quality_flags))


@dataclass(frozen=True)
class CameraSample:
    camera_name: str
    host_realtime_ns: int
    source_timestamp_ns: int
    delta_ns: int
    color: ImageData | None
    depth: ImageData | None
    timing: object | None
    timing_reference: MessageRef
    attached_tracker: TrackerPose | None
    world_from_camera: Transform | None


@dataclass(frozen=True)
class AdditionalSample:
    stream_name: str
    timestamp_ns: int
    delta_ns: int
    reference: MessageRef
    message: object


@dataclass(frozen=True)
class AlignedFrame:
    frame_index: int
    reference_camera: str
    reference_time_ns: int
    cameras: Mapping[str, CameraSample | None]
    trackers: Mapping[str, TrackerPose | None]
    additional_streams: Mapping[str, AdditionalSample | None]
    quality_flags: tuple[str, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "cameras", _immutable_mapping(self.cameras))
        object.__setattr__(self, "trackers", _immutable_mapping(self.trackers))
        object.__setattr__(
            self,
            "additional_streams",
            _immutable_mapping(self.additional_streams),
        )
        object.__setattr__(self, "quality_flags", tuple(self.quality_flags))


def _mapping(value: object, context: str) -> dict[str, object]:
    if type(value) is not dict:
        raise DatasetFormatError(f"{context} must be a mapping")
    return value


def _keys(value: Mapping[str, object], expected: set[str], context: str) -> None:
    if set(value) != expected:
        raise DatasetFormatError(
            f"{context} keys mismatch: "
            f"expected={sorted(expected)} observed={sorted(value)}"
        )


def _text(value: Mapping[str, object], key: str, context: str) -> str:
    result = value.get(key)
    if type(result) is not str or not result:
        raise DatasetFormatError(f"{context}.{key} must be non-empty text")
    return result


def _integer(
    value: Mapping[str, object],
    key: str,
    context: str,
    *,
    non_negative: bool = True,
) -> int:
    result = value.get(key)
    if type(result) is not int or (non_negative and result < 0):
        qualifier = "a non-negative integer" if non_negative else "an integer"
        raise DatasetFormatError(f"{context}.{key} must be {qualifier}")
    return result


def _parse_reference(value: object, context: str) -> MessageRef:
    document = _mapping(value, context)
    _keys(
        document,
        {"topic", "sequence", "bag_timestamp_ns", "source_timestamp_ns"},
        context,
    )
    try:
        return MessageRef(
            topic=_text(document, "topic", context),
            sequence=_integer(document, "sequence", context),
            bag_timestamp_ns=_integer(document, "bag_timestamp_ns", context),
            source_timestamp_ns=_integer(
                document, "source_timestamp_ns", context
            ),
        )
    except (TypeError, ValueError) as error:
        raise DatasetFormatError(f"{context} is invalid: {error}") from error


def _parse_transform(value: object, context: str) -> Transform:
    document = _mapping(value, context)
    _keys(document, {"translation_m", "quaternion_xyzw"}, context)
    try:
        return Transform(
            np.asarray(document["translation_m"], dtype=np.float64),
            np.asarray(document["quaternion_xyzw"], dtype=np.float64),
        )
    except (TypeError, ValueError) as error:
        raise DatasetFormatError(f"{context} transform is invalid: {error}") from error


def _parse_pose(value: object, context: str) -> TrackerPose:
    document = _mapping(value, context)
    _keys(
        document,
        {
            "role",
            "tracker_id",
            "timestamp_ns",
            "bracket_gap_ns",
            "before_sequence",
            "after_sequence",
            "world_from_tracker",
        },
        context,
    )
    role = _text(document, "role", context)
    tracker_id = _text(document, "tracker_id", context)
    if _TRACKER_ID.fullmatch(tracker_id) is None:
        raise DatasetFormatError(f"{context}.tracker_id is malformed")
    before = _integer(document, "before_sequence", context)
    after = _integer(document, "after_sequence", context)
    if before > after:
        raise DatasetFormatError(
            f"{context}.before_sequence must not exceed after_sequence"
        )
    return TrackerPose(
        role=role,
        tracker_id=tracker_id,
        timestamp_ns=_integer(document, "timestamp_ns", context),
        bracket_gap_ns=_integer(document, "bracket_gap_ns", context),
        before_sequence=before,
        after_sequence=after,
        world_from_tracker=_parse_transform(
            document["world_from_tracker"], f"{context}.world_from_tracker"
        ),
    )


def _parse_camera(
    name: str, value: object, context: str
) -> CameraRecord | None:
    if value is None:
        return None
    document = _mapping(value, context)
    _keys(
        document,
        {
            "host_realtime_ns",
            "source_timestamp_ns",
            "delta_ns",
            "color",
            "depth",
            "timing",
            "attached_tracker",
            "world_from_camera",
        },
        context,
    )
    source = _integer(document, "source_timestamp_ns", context)
    color = _parse_reference(document["color"], f"{context}.color")
    depth = _parse_reference(document["depth"], f"{context}.depth")
    timing = _parse_reference(document["timing"], f"{context}.timing")
    if any(
        reference.source_timestamp_ns != source
        for reference in (color, depth, timing)
    ):
        raise DatasetFormatError(
            f"{context} reference source timestamp does not match camera source timestamp"
        )
    attached_value = document["attached_tracker"]
    world_value = document["world_from_camera"]
    return CameraRecord(
        camera_name=name,
        host_realtime_ns=_integer(document, "host_realtime_ns", context),
        source_timestamp_ns=source,
        delta_ns=_integer(
            document, "delta_ns", context, non_negative=False
        ),
        color=color,
        depth=depth,
        timing=timing,
        attached_tracker=(
            None
            if attached_value is None
            else _parse_pose(attached_value, f"{context}.attached_tracker")
        ),
        world_from_camera=(
            None
            if world_value is None
            else _parse_transform(world_value, f"{context}.world_from_camera")
        ),
    )


def _parse_additional(
    name: str, value: object, context: str
) -> AdditionalRecord | None:
    if value is None:
        return None
    document = _mapping(value, context)
    _keys(document, {"timestamp_ns", "delta_ns", "message"}, context)
    timestamp = _integer(document, "timestamp_ns", context)
    reference = _parse_reference(document["message"], f"{context}.message")
    if reference.source_timestamp_ns != timestamp:
        raise DatasetFormatError(
            f"{context} reference source timestamp does not match stream timestamp"
        )
    return AdditionalRecord(
        stream_name=name,
        timestamp_ns=timestamp,
        delta_ns=_integer(
            document, "delta_ns", context, non_negative=False
        ),
        message=reference,
    )


def _require_names(
    observed: Mapping[str, object],
    expected: tuple[str, ...] | None,
    label: str,
) -> None:
    if expected is not None and set(observed) != set(expected):
        raise DatasetFormatError(
            f"{label} mismatch: "
            f"expected={sorted(expected)} observed={sorted(observed)}"
        )


def parse_frame_record(
    value: object,
    *,
    expected_index: int,
    camera_names: tuple[str, ...] | None = None,
    tracker_roles: tuple[str, ...] | None = None,
    additional_stream_names: tuple[str, ...] | None = None,
) -> FrameRecord:
    document = _mapping(value, "aligned frame")
    _keys(
        document,
        {
            "frame_index",
            "reference_camera",
            "reference_time_ns",
            "cameras",
            "trackers",
            "additional_streams",
            "quality_flags",
        },
        "aligned frame",
    )
    frame_index = _integer(document, "frame_index", "aligned frame")
    if frame_index != expected_index:
        raise DatasetFormatError(
            f"aligned frame.frame_index expected {expected_index}, observed {frame_index}"
        )
    cameras = _mapping(document["cameras"], "aligned frame.cameras")
    trackers = _mapping(document["trackers"], "aligned frame.trackers")
    additional = _mapping(
        document["additional_streams"], "aligned frame.additional_streams"
    )
    _require_names(cameras, camera_names, "camera names")
    _require_names(trackers, tracker_roles, "Tracker roles")
    _require_names(additional, additional_stream_names, "additional stream names")
    parsed_trackers: dict[str, TrackerPose | None] = {}
    for role, pose in trackers.items():
        if type(role) is not str or not role:
            raise DatasetFormatError("Tracker role names must be non-empty text")
        parsed = None if pose is None else _parse_pose(pose, f"trackers.{role}")
        if parsed is not None and parsed.role != role:
            raise DatasetFormatError(f"trackers.{role}.role does not match its key")
        parsed_trackers[role] = parsed
    flags = document["quality_flags"]
    if type(flags) is not list or any(
        type(item) is not str or not item for item in flags
    ):
        raise DatasetFormatError("aligned frame.quality_flags must be a text list")
    return FrameRecord(
        frame_index=frame_index,
        reference_camera=_text(document, "reference_camera", "aligned frame"),
        reference_time_ns=_integer(
            document, "reference_time_ns", "aligned frame"
        ),
        cameras={
            name: _parse_camera(name, camera, f"cameras.{name}")
            for name, camera in cameras.items()
        },
        trackers=parsed_trackers,
        additional_streams={
            name: _parse_additional(name, stream, f"additional_streams.{name}")
            for name, stream in additional.items()
        },
        quality_flags=tuple(flags),
    )
