from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable, Iterable, Iterator, Mapping
import hashlib
import json
from pathlib import Path
from types import MappingProxyType
from typing import BinaryIO

import yaml

from .errors import (
    DatasetClosedError,
    DatasetError,
    DatasetFormatError,
    IntegrityError,
    MissingMessageError,
    RejectedDatasetError,
    SourceBagMismatchError,
)
from .export import OUTPUT_FILES, validate_output
from .image_decoder import decode_image
from .message_resolver import MessageResolver, open_rosbag_resolver
from .sdk_model import (
    AdditionalSample,
    AlignedFrame,
    CameraInfoData,
    CameraSample,
    FrameRecord,
    parse_frame_record,
)


ResolverFactory = Callable[..., MessageResolver]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_json(path: Path, context: str) -> dict[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise DatasetFormatError(f"cannot read {context}: {path}: {error}") from error
    if type(value) is not dict:
        raise DatasetFormatError(f"{context} must contain a JSON mapping")
    return value


def _mapping(value: object, context: str) -> dict[str, object]:
    if type(value) is not dict:
        raise DatasetFormatError(f"{context} must be a mapping")
    return value


def _text(value: Mapping[str, object], key: str, context: str) -> str:
    result = value.get(key)
    if type(result) is not str or not result:
        raise DatasetFormatError(f"{context}.{key} must be non-empty text")
    return result


def _freeze(value: object) -> object:
    if type(value) is dict:
        return MappingProxyType(
            {str(key): _freeze(item) for key, item in value.items()}
        )
    if type(value) is list:
        return tuple(_freeze(item) for item in value)
    return value


def _source_storage_identifier(metadata_path: Path) -> str:
    try:
        document = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        root = document["rosbag2_bagfile_information"]
        identifier = root["storage_identifier"]
    except (OSError, UnicodeError, yaml.YAMLError, KeyError, TypeError) as error:
        raise SourceBagMismatchError(
            f"source bag metadata is malformed: {metadata_path}"
        ) from error
    if type(identifier) is not str or not identifier:
        raise SourceBagMismatchError(
            f"source bag storage identifier is malformed: {metadata_path}"
        )
    return identifier


def _validate_source_bag(
    bag_path: Path, source: Mapping[str, object]
) -> str:
    if not bag_path.is_dir():
        raise SourceBagMismatchError(
            f"source bag directory does not exist: {bag_path}"
        )
    expected_name = _text(source, "name", "manifest.source_bag")
    if bag_path.name != expected_name:
        raise SourceBagMismatchError(
            "source bag name mismatch: "
            f"expected={expected_name} observed={bag_path.name}"
        )
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        raise SourceBagMismatchError(
            f"source bag metadata does not exist: {metadata_path}"
        )
    expected_hash = _text(
        source, "metadata_sha256", "manifest.source_bag"
    )
    observed_hash = _sha256(metadata_path)
    if observed_hash != expected_hash:
        raise SourceBagMismatchError(
            "source bag metadata SHA-256 mismatch: "
            f"expected={expected_hash} observed={observed_hash}"
        )
    expected_storage = _text(
        source, "storage_identifier", "manifest.source_bag"
    )
    observed_storage = _source_storage_identifier(metadata_path)
    if observed_storage != expected_storage:
        raise SourceBagMismatchError(
            "source bag storage identifier mismatch: "
            f"expected={expected_storage} observed={observed_storage}"
        )
    return observed_storage


class AlignedDataset:
    """Read an alignment index and resolve selected payloads from its source bag."""

    def __init__(
        self,
        *,
        alignment_dir: Path,
        bag_path: Path,
        manifest: dict[str, object],
        quality_report: dict[str, object],
        frame_stream: BinaryIO,
        frame_offsets: tuple[int, ...],
        camera_names: tuple[str, ...],
        tracker_roles: tuple[str, ...],
        additional_stream_names: tuple[str, ...],
        camera_frames: Mapping[str, str],
        camera_info_topics: frozenset[str],
        expected_topic_types: Mapping[str, str],
        read_topics: frozenset[str],
        timestamp_fields: Mapping[str, str],
        reusable_topics: frozenset[str],
        storage_identifier: str,
        cache_size: int,
        resolver_factory: ResolverFactory,
    ) -> None:
        self._alignment_dir = alignment_dir
        self._bag_path = bag_path
        self._manifest = _freeze(manifest)
        self._quality_report = _freeze(quality_report)
        self._frame_stream = frame_stream
        self._frame_offsets = frame_offsets
        self._camera_names = camera_names
        self._tracker_roles = tracker_roles
        self._additional_stream_names = additional_stream_names
        self._camera_frames = MappingProxyType(dict(camera_frames))
        self._camera_info_topics = camera_info_topics
        self._expected_topic_types = MappingProxyType(
            dict(expected_topic_types)
        )
        self._read_topics = read_topics
        self._timestamp_fields = MappingProxyType(dict(timestamp_fields))
        self._reusable_topics = reusable_topics
        self._storage_identifier = storage_identifier
        self._cache_size = cache_size
        self._resolver_factory = resolver_factory
        self._resolver: MessageResolver | None = None
        self._camera_info: Mapping[str, CameraInfoData] | None = None
        self._record_cache: OrderedDict[int, FrameRecord] = OrderedDict()
        self._frame_cache: OrderedDict[tuple[object, ...], AlignedFrame] = (
            OrderedDict()
        )
        self._closed = False

    @classmethod
    def open(
        cls,
        alignment_dir: str | Path,
        bag_path: str | Path,
        *,
        allow_rejected: bool = False,
        verify_integrity: bool = True,
        cache_size: int = 8,
        _resolver_factory: ResolverFactory | None = None,
    ) -> AlignedDataset:
        if type(allow_rejected) is not bool:
            raise ValueError("allow_rejected must be bool")
        if type(verify_integrity) is not bool:
            raise ValueError("verify_integrity must be bool")
        if type(cache_size) is not int or cache_size < 0:
            raise ValueError("cache_size must be a non-negative integer")
        output = Path(alignment_dir).expanduser().resolve()
        source_bag = Path(bag_path).expanduser().resolve()
        if not output.is_dir():
            raise DatasetFormatError(
                f"alignment output directory does not exist: {output}"
            )
        observed_files = {
            path.name for path in output.iterdir() if path.is_file()
        }
        if observed_files != OUTPUT_FILES:
            raise DatasetFormatError(
                "alignment output file set mismatch: "
                f"expected={sorted(OUTPUT_FILES)} "
                f"observed={sorted(observed_files)}"
            )
        if verify_integrity:
            try:
                validate_output(output)
            except Exception as error:
                raise IntegrityError(
                    f"alignment output integrity validation failed: {error}"
                ) from error

        manifest = _load_json(output / "manifest.json", "alignment manifest")
        catalog = _load_json(
            output / "stream_catalog.json", "alignment stream catalog"
        )
        quality = _load_json(
            output / "quality_report.json", "alignment quality report"
        )
        if manifest.get("schema_version") != 1:
            raise DatasetFormatError("unsupported alignment manifest schema")
        inventory = _mapping(
            manifest.get("files"), "manifest.files"
        )
        integrity_files = OUTPUT_FILES - {"manifest.json"}
        if set(inventory) != integrity_files:
            raise DatasetFormatError(
                "alignment manifest file inventory is malformed: "
                f"expected={sorted(integrity_files)} "
                f"observed={sorted(inventory)}"
            )
        for name, raw_expected in inventory.items():
            expected = _mapping(raw_expected, f"manifest.files.{name}")
            size = expected.get("size_bytes")
            digest = expected.get("sha256")
            if type(size) is not int or size < 0:
                raise DatasetFormatError(
                    f"manifest.files.{name}.size_bytes is malformed"
                )
            if (
                type(digest) is not str
                or len(digest) != 64
                or any(character not in "0123456789abcdef" for character in digest)
            ):
                raise DatasetFormatError(
                    f"manifest.files.{name}.sha256 is malformed"
                )
            observed_size = (output / name).stat().st_size
            if observed_size != size:
                raise IntegrityError(
                    f"alignment output size mismatch for {name}: "
                    f"expected={size} observed={observed_size}"
                )
        if catalog.get("schema_version") != 1:
            raise DatasetFormatError("unsupported alignment stream catalog schema")
        verdict = quality.get("verdict")
        if verdict not in {"ACCEPTED", "REJECTED"}:
            raise DatasetFormatError("quality report verdict is malformed")
        if manifest.get("verdict") != verdict:
            raise DatasetFormatError(
                "quality verdict differs from alignment manifest"
            )
        if verdict == "REJECTED" and not allow_rejected:
            reasons = quality.get("rejection_reasons", [])
            raise RejectedDatasetError(
                f"alignment quality verdict is REJECTED: {reasons}"
            )

        source_document = _mapping(
            manifest.get("source_bag"), "manifest.source_bag"
        )
        storage_identifier = _validate_source_bag(
            source_bag, source_document
        )
        extrinsics = _mapping(
            manifest.get("extrinsics"), "manifest.extrinsics"
        )
        if not extrinsics:
            raise DatasetFormatError("manifest.extrinsics must not be empty")
        camera_names = tuple(extrinsics)
        camera_frames: dict[str, str] = {}
        for name, raw_binding in extrinsics.items():
            binding = _mapping(
                raw_binding, f"manifest.extrinsics.{name}"
            )
            if _text(
                binding, "camera_name", f"manifest.extrinsics.{name}"
            ) != name:
                raise DatasetFormatError(
                    f"manifest.extrinsics.{name}.camera_name mismatch"
                )
            camera_frames[name] = _text(
                binding, "camera_frame", f"manifest.extrinsics.{name}"
            )
        tracker_ids = _mapping(
            manifest.get("tracker_ids"), "manifest.tracker_ids"
        )
        tracker_roles = tuple(tracker_ids)
        if not tracker_roles:
            raise DatasetFormatError("manifest.tracker_ids must not be empty")
        configuration = _mapping(
            manifest.get("configuration"), "manifest.configuration"
        )
        reference_camera = _text(
            configuration, "reference_camera", "manifest.configuration"
        )
        if reference_camera not in camera_names:
            raise DatasetFormatError(
                "manifest reference camera is not in extrinsics"
            )

        streams = catalog.get("streams")
        if type(streams) is not list:
            raise DatasetFormatError("stream catalog streams must be a list")
        expected_topic_types: dict[str, str] = {}
        timestamp_fields: dict[str, str] = {}
        camera_info_topics: set[str] = set()
        reusable_topics: set[str] = set()
        additional_stream_names_list: list[str] = []
        seen_stream_names: set[str] = set()
        for index, raw_entry in enumerate(streams):
            entry = _mapping(raw_entry, f"stream catalog entry {index}")
            topic = _text(entry, "topic", f"stream catalog entry {index}")
            type_name = _text(
                entry, "type", f"stream catalog entry {index}"
            )
            if topic in expected_topic_types:
                raise DatasetFormatError(
                    f"duplicate Topic in stream catalog: {topic}"
                )
            present = entry.get("present")
            if type(present) is not bool:
                raise DatasetFormatError(
                    f"stream catalog present flag is malformed: {topic}"
                )
            if present:
                expected_topic_types[topic] = type_name
            if type_name == "sensor_msgs/msg/CameraInfo" and present:
                camera_info_topics.add(topic)
            elif type_name in {
                "sensor_msgs/msg/Image",
                "vt_camera_msgs/msg/CameraFrameTiming",
            } and present:
                timestamp_fields[topic] = "header.stamp"
            if entry.get("contract") == "extension":
                stream_name = _text(
                    entry, "stream_name", f"stream catalog entry {index}"
                )
                if stream_name in seen_stream_names:
                    raise DatasetFormatError(
                        f"duplicate extension stream name: {stream_name}"
                    )
                seen_stream_names.add(stream_name)
                additional_stream_names_list.append(stream_name)
                if present:
                    timestamp_fields[topic] = _text(
                        entry,
                        "timestamp_field",
                        f"stream catalog entry {index}",
                    )
                    reusable_topics.add(topic)
        additional_stream_names = tuple(additional_stream_names_list)
        read_topics = frozenset(timestamp_fields) | frozenset(camera_info_topics)

        frame_path = output / "aligned_frames.jsonl"
        frame_stream: BinaryIO | None = None
        try:
            frame_stream = frame_path.open("rb")
            offsets: list[int] = []
            previous_time: int | None = None
            while True:
                offset = frame_stream.tell()
                line = frame_stream.readline()
                if not line:
                    break
                line_number = len(offsets) + 1
                if not line.strip():
                    raise DatasetFormatError(
                        f"blank aligned record at line {line_number}"
                    )
                try:
                    raw_record = json.loads(line)
                except (UnicodeError, json.JSONDecodeError) as error:
                    raise DatasetFormatError(
                        f"invalid aligned record at line {line_number}: {error}"
                    ) from error
                record = parse_frame_record(
                    raw_record,
                    expected_index=len(offsets),
                    camera_names=camera_names,
                    tracker_roles=tracker_roles,
                    additional_stream_names=additional_stream_names,
                )
                if record.reference_camera != reference_camera:
                    raise DatasetFormatError(
                        f"reference camera mismatch at line {line_number}"
                    )
                if (
                    previous_time is not None
                    and record.reference_time_ns <= previous_time
                ):
                    raise DatasetFormatError(
                        "aligned reference times must be strictly increasing"
                    )
                previous_time = record.reference_time_ns
                offsets.append(offset)
            expected_count = manifest.get("aligned_frame_count")
            if type(expected_count) is not int or expected_count != len(offsets):
                raise DatasetFormatError(
                    "aligned frame count differs from manifest"
                )
            frame_stream.seek(0)
            return cls(
                alignment_dir=output,
                bag_path=source_bag,
                manifest=manifest,
                quality_report=quality,
                frame_stream=frame_stream,
                frame_offsets=tuple(offsets),
                camera_names=camera_names,
                tracker_roles=tracker_roles,
                additional_stream_names=additional_stream_names,
                camera_frames=camera_frames,
                camera_info_topics=frozenset(camera_info_topics),
                expected_topic_types=expected_topic_types,
                read_topics=read_topics,
                timestamp_fields=timestamp_fields,
                reusable_topics=frozenset(reusable_topics),
                storage_identifier=storage_identifier,
                cache_size=cache_size,
                resolver_factory=(
                    open_rosbag_resolver
                    if _resolver_factory is None
                    else _resolver_factory
                ),
            )
        except Exception:
            if frame_stream is not None:
                frame_stream.close()
            raise

    def _require_open(self) -> None:
        if self._closed:
            raise DatasetClosedError("aligned dataset is closed")

    def __len__(self) -> int:
        self._require_open()
        return len(self._frame_offsets)

    @property
    def manifest(self) -> Mapping[str, object]:
        self._require_open()
        return self._manifest

    @property
    def quality_report(self) -> Mapping[str, object]:
        self._require_open()
        return self._quality_report

    @property
    def camera_names(self) -> tuple[str, ...]:
        self._require_open()
        return self._camera_names

    @property
    def tracker_roles(self) -> tuple[str, ...]:
        self._require_open()
        return self._tracker_roles

    @property
    def additional_stream_names(self) -> tuple[str, ...]:
        self._require_open()
        return self._additional_stream_names

    def _normalize_index(self, index: int) -> int:
        if type(index) is not int:
            raise TypeError("frame index must be an integer")
        count = len(self._frame_offsets)
        normalized = index + count if index < 0 else index
        if not 0 <= normalized < count:
            raise IndexError(f"frame index out of range: {index}")
        return normalized

    def record(self, index: int) -> FrameRecord:
        self._require_open()
        normalized = self._normalize_index(index)
        cached = self._record_cache.get(normalized)
        if cached is not None:
            self._record_cache.move_to_end(normalized)
            return cached
        self._frame_stream.seek(self._frame_offsets[normalized])
        line = self._frame_stream.readline()
        try:
            document = json.loads(line)
        except (UnicodeError, json.JSONDecodeError) as error:
            raise DatasetFormatError(
                f"cannot reload aligned frame {normalized}: {error}"
            ) from error
        record = parse_frame_record(
            document,
            expected_index=normalized,
            camera_names=self._camera_names,
            tracker_roles=self._tracker_roles,
            additional_stream_names=self._additional_stream_names,
        )
        self._record_cache[normalized] = record
        self._record_cache.move_to_end(normalized)
        while len(self._record_cache) > 128:
            self._record_cache.popitem(last=False)
        return record

    def _ensure_resolver(self) -> MessageResolver:
        self._require_open()
        if self._resolver is None:
            self._resolver = self._resolver_factory(
                bag_path=self._bag_path,
                storage_identifier=self._storage_identifier,
                expected_topic_types=self._expected_topic_types,
                read_topics=self._read_topics,
                timestamp_fields=self._timestamp_fields,
                reusable_topics=self._reusable_topics,
            )
        return self._resolver

    @property
    def camera_info(self) -> Mapping[str, CameraInfoData]:
        self._require_open()
        if self._camera_info is None:
            values = self._ensure_resolver().load_camera_info(
                camera_frames=self._camera_frames,
                camera_info_topics=self._camera_info_topics,
            )
            self._camera_info = MappingProxyType(dict(values))
        return self._camera_info

    @staticmethod
    def _selection(
        values: Iterable[str] | None,
        allowed: tuple[str, ...],
        label: str,
    ) -> tuple[str, ...]:
        if values is None:
            return allowed
        if isinstance(values, (str, bytes)):
            raise ValueError(f"{label} must be an iterable of names")
        try:
            requested = tuple(values)
        except TypeError as error:
            raise ValueError(f"{label} must be an iterable of names") from error
        if any(type(value) is not str for value in requested):
            raise ValueError(f"{label} must contain text names")
        if len(requested) != len(set(requested)):
            raise ValueError(f"{label} contains duplicate names")
        unknown = set(requested) - set(allowed)
        if unknown:
            raise ValueError(f"unknown {label}: {sorted(unknown)[0]}")
        requested_set = set(requested)
        return tuple(value for value in allowed if value in requested_set)

    @staticmethod
    def _resolved_message(
        resolved: Mapping[object, object], reference: object
    ) -> object:
        try:
            return resolved[reference]
        except KeyError as error:
            raise MissingMessageError(
                f"resolver did not return requested reference: {reference}"
            ) from error

    def frame(
        self,
        index: int,
        *,
        cameras: Iterable[str] | None = None,
        image_kinds: Iterable[str] = ("color", "depth"),
        include_timing: bool = True,
        additional_streams: Iterable[str] | None = None,
    ) -> AlignedFrame:
        self._require_open()
        normalized = self._normalize_index(index)
        selected_cameras = self._selection(
            cameras, self._camera_names, "cameras"
        )
        selected_images = self._selection(
            image_kinds, ("color", "depth"), "image_kinds"
        )
        if type(include_timing) is not bool:
            raise ValueError("include_timing must be bool")
        selected_streams = self._selection(
            additional_streams,
            self._additional_stream_names,
            "additional_streams",
        )
        cache_key: tuple[object, ...] = (
            normalized,
            selected_cameras,
            selected_images,
            include_timing,
            selected_streams,
        )
        cached = self._frame_cache.get(cache_key)
        if cached is not None:
            self._frame_cache.move_to_end(cache_key)
            return cached
        record = self.record(normalized)
        references = []
        for name in selected_cameras:
            camera = record.cameras[name]
            if camera is None:
                continue
            if "color" in selected_images:
                references.append(camera.color)
            if "depth" in selected_images:
                references.append(camera.depth)
            if include_timing:
                references.append(camera.timing)
        for name in selected_streams:
            stream = record.additional_streams[name]
            if stream is not None:
                references.append(stream.message)
        resolved: Mapping[object, object] = {}
        if references:
            resolved = self._ensure_resolver().resolve_many(references)

        camera_samples: dict[str, CameraSample | None] = {}
        for name in selected_cameras:
            source = record.cameras[name]
            if source is None:
                camera_samples[name] = None
                continue
            color = (
                decode_image(
                    self._resolved_message(resolved, source.color), source.color
                )
                if "color" in selected_images
                else None
            )
            depth = (
                decode_image(
                    self._resolved_message(resolved, source.depth), source.depth
                )
                if "depth" in selected_images
                else None
            )
            timing = (
                self._resolved_message(resolved, source.timing)
                if include_timing
                else None
            )
            camera_samples[name] = CameraSample(
                camera_name=name,
                host_realtime_ns=source.host_realtime_ns,
                source_timestamp_ns=source.source_timestamp_ns,
                delta_ns=source.delta_ns,
                color=color,
                depth=depth,
                timing=timing,
                timing_reference=source.timing,
                attached_tracker=source.attached_tracker,
                world_from_camera=source.world_from_camera,
            )

        additional_samples: dict[str, AdditionalSample | None] = {}
        for name in selected_streams:
            source = record.additional_streams[name]
            if source is None:
                additional_samples[name] = None
            else:
                additional_samples[name] = AdditionalSample(
                    stream_name=name,
                    timestamp_ns=source.timestamp_ns,
                    delta_ns=source.delta_ns,
                    reference=source.message,
                    message=self._resolved_message(
                        resolved, source.message
                    ),
                )
        result = AlignedFrame(
            frame_index=record.frame_index,
            reference_camera=record.reference_camera,
            reference_time_ns=record.reference_time_ns,
            cameras=camera_samples,
            trackers=record.trackers,
            additional_streams=additional_samples,
            quality_flags=record.quality_flags,
        )
        if self._cache_size > 0:
            self._frame_cache[cache_key] = result
            self._frame_cache.move_to_end(cache_key)
            while len(self._frame_cache) > self._cache_size:
                self._frame_cache.popitem(last=False)
        return result

    def iter_frames(
        self,
        *,
        start: int = 0,
        stop: int | None = None,
        step: int = 1,
        cameras: Iterable[str] | None = None,
        image_kinds: Iterable[str] = ("color", "depth"),
        include_timing: bool = True,
        additional_streams: Iterable[str] | None = None,
    ) -> Iterator[AlignedFrame]:
        self._require_open()
        if type(step) is not int or step <= 0:
            raise ValueError("step must be a positive integer")
        if type(start) is not int or (stop is not None and type(stop) is not int):
            raise ValueError("start and stop must be integers or None")
        selected_cameras = self._selection(
            cameras, self._camera_names, "cameras"
        )
        selected_images = self._selection(
            image_kinds, ("color", "depth"), "image_kinds"
        )
        if type(include_timing) is not bool:
            raise ValueError("include_timing must be bool")
        selected_streams = self._selection(
            additional_streams,
            self._additional_stream_names,
            "additional_streams",
        )
        begin, end, normalized_step = slice(start, stop, step).indices(
            len(self._frame_offsets)
        )
        for index in range(begin, end, normalized_step):
            yield self.frame(
                index,
                cameras=selected_cameras,
                image_kinds=selected_images,
                include_timing=include_timing,
                additional_streams=selected_streams,
            )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._record_cache.clear()
        self._frame_cache.clear()
        self._camera_info = None
        try:
            if self._resolver is not None:
                self._resolver.close()
        finally:
            self._frame_stream.close()

    def __enter__(self) -> AlignedDataset:
        self._require_open()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> bool:
        self.close()
        return False
