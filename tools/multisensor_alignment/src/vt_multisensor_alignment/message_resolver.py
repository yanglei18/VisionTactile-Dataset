from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable, Iterable, Mapping
from pathlib import Path
from types import MappingProxyType
from typing import Protocol

import numpy as np

from .bag_reader import extract_timestamp_ns
from .errors import (
    DatasetClosedError,
    DatasetError,
    DatasetFormatError,
    MissingMessageError,
)
from .model import MessageRef
from .sdk_model import CameraInfoData, RegionOfInterestData


class ReaderBackend(Protocol):
    def seek(self, timestamp_ns: int) -> None: ...

    def has_next(self) -> bool: ...

    def read_next(self) -> tuple[str, object, int]: ...

    def close(self) -> None: ...


class MessageResolver:
    """Resolve alignment references using one seekable, ordered bag reader."""

    def __init__(
        self,
        *,
        backend: ReaderBackend,
        deserialize: Callable[[str, object], object],
        timestamp_fields: Mapping[str, str],
        reusable_topics: frozenset[str],
        reusable_cache_size: int = 32,
    ) -> None:
        if type(reusable_cache_size) is not int or reusable_cache_size < 0:
            raise ValueError("reusable_cache_size must be a non-negative integer")
        self._backend = backend
        self._deserialize = deserialize
        self._timestamp_fields = dict(timestamp_fields)
        self._reusable_topics = frozenset(reusable_topics)
        self._reusable_cache_size = reusable_cache_size
        self._reusable_cache: OrderedDict[MessageRef, object] = OrderedDict()
        self._lookahead: tuple[str, object, int] | None = None
        self._fully_scanned_through = -1
        self._positioned = False
        self._closed = False

    def _require_open(self) -> None:
        if self._closed:
            raise DatasetClosedError("message resolver is closed")

    def _seek(self, timestamp_ns: int) -> None:
        try:
            self._backend.seek(timestamp_ns)
        except Exception as error:
            raise DatasetError(
                f"failed to seek source bag to {timestamp_ns} ns"
            ) from error
        self._lookahead = None
        self._fully_scanned_through = timestamp_ns - 1
        self._positioned = True

    def _next_record(self) -> tuple[str, object, int] | None:
        if self._lookahead is not None:
            result = self._lookahead
            self._lookahead = None
            return result
        try:
            if not self._backend.has_next():
                return None
            topic, payload, timestamp_ns = self._backend.read_next()
        except Exception as error:
            raise DatasetError("failed while reading the source bag") from error
        if type(topic) is not str or type(timestamp_ns) is not int:
            raise DatasetFormatError("bag reader returned a malformed record")
        return topic, payload, timestamp_ns

    def _decode(self, topic: str, payload: object) -> object:
        try:
            return self._deserialize(topic, payload)
        except DatasetError:
            raise
        except Exception as error:
            raise DatasetFormatError(
                f"failed to deserialize ROS message on {topic}"
            ) from error

    def _source_timestamp(self, topic: str, message: object) -> int:
        field = self._timestamp_fields.get(topic)
        if field is None:
            raise DatasetFormatError(
                f"no source timestamp field is configured for {topic}"
            )
        try:
            return extract_timestamp_ns(message, field)
        except (TypeError, ValueError) as error:
            raise DatasetFormatError(
                f"cannot read source timestamp {field} on {topic}: {error}"
            ) from error

    def _cache_reusable(self, reference: MessageRef, message: object) -> None:
        if (
            reference.topic not in self._reusable_topics
            or self._reusable_cache_size == 0
        ):
            return
        self._reusable_cache[reference] = message
        self._reusable_cache.move_to_end(reference)
        while len(self._reusable_cache) > self._reusable_cache_size:
            self._reusable_cache.popitem(last=False)

    def resolve_many(
        self, references: Iterable[MessageRef]
    ) -> Mapping[MessageRef, object]:
        self._require_open()
        ordered: list[MessageRef] = []
        seen: set[MessageRef] = set()
        for reference in references:
            if not isinstance(reference, MessageRef):
                raise TypeError("references must contain MessageRef values")
            if reference not in seen:
                ordered.append(reference)
                seen.add(reference)
        result: dict[MessageRef, object] = {}
        pending: list[MessageRef] = []
        for reference in ordered:
            cached = self._reusable_cache.get(reference)
            if cached is None:
                pending.append(reference)
            else:
                self._reusable_cache.move_to_end(reference)
                result[reference] = cached
        if not pending:
            return MappingProxyType(result)

        minimum = min(value.bag_timestamp_ns for value in pending)
        maximum = max(value.bag_timestamp_ns for value in pending)
        if not self._positioned or minimum <= self._fully_scanned_through:
            self._seek(minimum)

        by_pair: dict[tuple[str, int], list[MessageRef]] = {}
        for reference in pending:
            by_pair.setdefault(
                (reference.topic, reference.bag_timestamp_ns), []
            ).append(reference)
        observed_sources: dict[tuple[str, int], set[int]] = {}
        while True:
            record = self._next_record()
            if record is None:
                break
            topic, payload, bag_timestamp_ns = record
            if bag_timestamp_ns > maximum:
                self._lookahead = record
                break
            pair = (topic, bag_timestamp_ns)
            candidates = by_pair.get(pair)
            if not candidates:
                continue
            message = self._decode(topic, payload)
            source = self._source_timestamp(topic, message)
            observed_sources.setdefault(pair, set()).add(source)
            matches = [
                reference
                for reference in candidates
                if reference.source_timestamp_ns == source
            ]
            if len(matches) > 1:
                raise DatasetFormatError(
                    "multiple message references identify one bag record: "
                    f"topic={topic} bag_timestamp_ns={bag_timestamp_ns}"
                )
            if matches:
                reference = matches[0]
                if reference in result:
                    raise DatasetFormatError(
                        "duplicate exact message in source bag: "
                        f"topic={topic} bag_timestamp_ns={bag_timestamp_ns} "
                        f"source_timestamp_ns={source}"
                    )
                result[reference] = message
                self._cache_reusable(reference, message)
        self._fully_scanned_through = max(
            self._fully_scanned_through, maximum
        )
        missing = [reference for reference in pending if reference not in result]
        if missing:
            reference = missing[0]
            sources = sorted(
                observed_sources.get(
                    (reference.topic, reference.bag_timestamp_ns), set()
                )
            )
            suffix = f" observed_source_timestamps={sources}" if sources else ""
            raise MissingMessageError(
                "referenced message is absent from source bag: "
                f"topic={reference.topic} "
                f"bag_timestamp_ns={reference.bag_timestamp_ns} "
                f"source_timestamp_ns={reference.source_timestamp_ns}"
                f"{suffix}"
            )
        return MappingProxyType(
            {reference: result[reference] for reference in ordered}
        )

    def load_camera_info(
        self,
        *,
        camera_frames: Mapping[str, str],
        camera_info_topics: frozenset[str],
    ) -> Mapping[str, CameraInfoData]:
        self._require_open()
        frames = dict(camera_frames)
        if len(set(frames.values())) != len(frames):
            raise DatasetFormatError("camera optical frames must be unique")
        by_frame = {frame: name for name, frame in frames.items()}
        found: dict[str, CameraInfoData] = {}
        self._seek(0)
        try:
            while len(found) < len(frames):
                record = self._next_record()
                if record is None:
                    break
                topic, payload, _ = record
                if topic not in camera_info_topics:
                    continue
                message = self._decode(topic, payload)
                try:
                    frame_id = message.header.frame_id
                except AttributeError as error:
                    raise DatasetFormatError(
                        f"CameraInfo.header is malformed on {topic}"
                    ) from error
                camera_name = by_frame.get(frame_id)
                if camera_name is None or camera_name in found:
                    continue
                found[camera_name] = _camera_info_data(camera_name, message)
        finally:
            self._seek(0)
        missing = sorted(set(frames) - set(found))
        if missing:
            raise MissingMessageError(
                "source bag contains no CameraInfo for " + ", ".join(missing)
            )
        return MappingProxyType(found)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._lookahead = None
        self._reusable_cache.clear()
        try:
            self._backend.close()
        except Exception as error:
            raise DatasetError("failed to close source bag reader") from error


def _non_negative_integer(value: object, context: str) -> int:
    if type(value) is not int or value < 0:
        raise DatasetFormatError(f"{context} must be a non-negative integer")
    return value


def _camera_info_data(camera_name: str, message: object) -> CameraInfoData:
    try:
        frame_id = message.header.frame_id
        width = _non_negative_integer(message.width, "CameraInfo.width")
        height = _non_negative_integer(message.height, "CameraInfo.height")
        distortion_model = message.distortion_model
        d = np.asarray(message.d, dtype=np.float64)
        k = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        r = np.asarray(message.r, dtype=np.float64).reshape(3, 3)
        p = np.asarray(message.p, dtype=np.float64).reshape(3, 4)
        binning_x = _non_negative_integer(
            message.binning_x, "CameraInfo.binning_x"
        )
        binning_y = _non_negative_integer(
            message.binning_y, "CameraInfo.binning_y"
        )
        roi = RegionOfInterestData(
            x_offset=_non_negative_integer(
                message.roi.x_offset, "CameraInfo.roi.x_offset"
            ),
            y_offset=_non_negative_integer(
                message.roi.y_offset, "CameraInfo.roi.y_offset"
            ),
            height=_non_negative_integer(
                message.roi.height, "CameraInfo.roi.height"
            ),
            width=_non_negative_integer(
                message.roi.width, "CameraInfo.roi.width"
            ),
            do_rectify=message.roi.do_rectify,
        )
    except (AttributeError, TypeError, ValueError) as error:
        raise DatasetFormatError(
            f"CameraInfo for {camera_name} is malformed: {error}"
        ) from error
    if type(frame_id) is not str or not frame_id:
        raise DatasetFormatError(
            f"CameraInfo for {camera_name} has an invalid frame_id"
        )
    if type(distortion_model) is not str:
        raise DatasetFormatError(
            f"CameraInfo for {camera_name} has an invalid distortion_model"
        )
    if type(roi.do_rectify) is not bool:
        raise DatasetFormatError(
            f"CameraInfo for {camera_name} has an invalid roi.do_rectify"
        )
    try:
        return CameraInfoData(
            camera_name=camera_name,
            frame_id=frame_id,
            width=width,
            height=height,
            distortion_model=distortion_model,
            d=d,
            k=k,
            r=r,
            p=p,
            binning_x=binning_x,
            binning_y=binning_y,
            roi=roi,
        )
    except ValueError as error:
        raise DatasetFormatError(
            f"CameraInfo for {camera_name} is malformed: {error}"
        ) from error


class _RosbagBackend:
    def __init__(self, reader: object) -> None:
        self._reader = reader

    def seek(self, timestamp_ns: int) -> None:
        self._reader.seek(timestamp_ns)

    def has_next(self) -> bool:
        return bool(self._reader.has_next())

    def read_next(self) -> tuple[str, object, int]:
        return self._reader.read_next()

    def close(self) -> None:
        close = getattr(self._reader, "close", None)
        if close is not None:
            close()


def open_rosbag_resolver(
    *,
    bag_path: str | Path,
    storage_identifier: str,
    expected_topic_types: Mapping[str, str],
    read_topics: frozenset[str],
    timestamp_fields: Mapping[str, str],
    reusable_topics: frozenset[str],
) -> MessageResolver:
    """Open the ROS adapter lazily so metadata inspection works without ROS."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise DatasetError(
            "ROS 2 Jazzy and the built workspace must be sourced before "
            "reading aligned payloads"
        ) from error
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(
                uri=str(Path(bag_path).resolve()),
                storage_id=storage_identifier,
            ),
            rosbag2_py.ConverterOptions("", ""),
        )
        observed = {
            entry.name: entry.type
            for entry in reader.get_all_topics_and_types()
        }
        for topic, expected_type in expected_topic_types.items():
            observed_type = observed.get(topic)
            if observed_type is None:
                raise DatasetFormatError(
                    f"source bag is missing configured topic: {topic}"
                )
            if observed_type != expected_type:
                raise DatasetFormatError(
                    f"source bag topic type mismatch on {topic}: "
                    f"expected={expected_type} observed={observed_type}"
                )
        reader.set_filter(
            rosbag2_py.StorageFilter(topics=sorted(read_topics))
        )
        message_classes = {
            topic: get_message(expected_topic_types[topic])
            for topic in read_topics
        }
    except DatasetError:
        close = getattr(reader, "close", None)
        if close is not None:
            close()
        raise
    except Exception as error:
        close = getattr(reader, "close", None)
        if close is not None:
            close()
        raise DatasetError(f"failed to open ROS 2 bag at {bag_path}") from error

    def deserialize(topic: str, payload: object) -> object:
        try:
            return deserialize_message(payload, message_classes[topic])
        except Exception as error:
            raise DatasetFormatError(
                f"failed to deserialize ROS message on {topic}"
            ) from error

    return MessageResolver(
        backend=_RosbagBackend(reader),
        deserialize=deserialize,
        timestamp_fields=timestamp_fields,
        reusable_topics=reusable_topics,
    )
