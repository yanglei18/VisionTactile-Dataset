from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
import re
from typing import Callable

import numpy as np
import yaml

from .config import AlignmentConfig
from .model import (
    BagDataset,
    CameraFrame,
    ClockObservation,
    GenericSample,
    MessageRef,
    TimedPose,
    TimingRecord,
    Transform,
)


_TRACKER_ID = re.compile(r"[0-9a-f]{64}")


def stamp_to_nanoseconds(stamp: object) -> int:
    seconds = getattr(stamp, "sec", None)
    nanoseconds = getattr(stamp, "nanosec", None)
    if type(seconds) is not int or type(nanoseconds) is not int:
        raise ValueError("message stamp is malformed")
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("message stamp is outside the supported range")
    return seconds * 1_000_000_000 + nanoseconds


def extract_timestamp_ns(message: object, field_path: str) -> int:
    if type(field_path) is not str or not field_path:
        raise ValueError("field_path must not be empty")
    value = message
    for token in field_path.split("."):
        try:
            value = getattr(value, token)
        except AttributeError as error:
            raise ValueError(
                f"message is missing timestamp field: {field_path}"
            ) from error
    if type(value) is int:
        if value < 0:
            raise ValueError("timestamp integer must be non-negative")
        return value
    return stamp_to_nanoseconds(value)


def configured_topic_types(config: AlignmentConfig) -> dict[str, str]:
    result: dict[str, str] = {}
    for camera in config.cameras:
        result.update(
            {
                camera.color_topic: "sensor_msgs/msg/Image",
                camera.depth_topic: "sensor_msgs/msg/Image",
                camera.camera_info_topic: "sensor_msgs/msg/CameraInfo",
                camera.timing_topic: "vt_camera_msgs/msg/CameraFrameTiming",
            }
        )
    result.update(
        {
            tracker.sample_topic: "vt_tracker_msgs/msg/TrackerSample"
            for tracker in config.trackers
        }
    )
    result.update(
        {stream.topic: stream.type_name for stream in config.additional_streams}
    )
    return result


def require_topic_types(
    observed: Mapping[str, str], config: AlignmentConfig
) -> None:
    expected = configured_topic_types(config)
    required = set(config.required_topics)
    unexpected = set(observed) - set(expected)
    if unexpected:
        raise ValueError(
            f"bag contains unconfigured topic: {sorted(unexpected)[0]}"
        )
    for topic, type_name in expected.items():
        actual = observed.get(topic)
        if actual is None:
            if topic in required:
                raise ValueError(f"required topic is absent from bag: {topic}")
            continue
        if actual != type_name:
            raise ValueError(
                f"topic type mismatch for {topic}: expected {type_name}, got {actual}"
            )


def join_camera_frames(
    camera_name: str,
    color_by_stamp: Mapping[int, MessageRef],
    depth_by_stamp: Mapping[int, MessageRef],
    timing_by_stamp: Mapping[int, TimingRecord],
) -> tuple[CameraFrame, ...]:
    common = set(color_by_stamp) & set(depth_by_stamp) & set(timing_by_stamp)
    frames = [
        CameraFrame(
            camera_name=camera_name,
            source_timestamp_ns=stamp,
            host_realtime_ns=timing_by_stamp[stamp].host_realtime_ns,
            host_monotonic_ns=timing_by_stamp[stamp].host_monotonic_ns,
            color=color_by_stamp[stamp],
            depth=depth_by_stamp[stamp],
            timing=timing_by_stamp[stamp].reference,
        )
        for stamp in common
    ]
    frames.sort(key=lambda value: value.host_realtime_ns)
    if any(
        left.host_realtime_ns >= right.host_realtime_ns
        for left, right in zip(frames, frames[1:])
    ):
        raise ValueError(f"camera host realtime is not increasing: {camera_name}")
    return tuple(frames)


def _storage_identifier(path: Path) -> str:
    metadata_path = path / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"rosbag2 metadata.yaml does not exist: {path}")
    document = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    try:
        identifier = document["rosbag2_bagfile_information"]["storage_identifier"]
    except (KeyError, TypeError) as error:
        raise ValueError(
            "rosbag2 metadata does not declare storage_identifier"
        ) from error
    if type(identifier) is not str or not identifier:
        raise ValueError("rosbag2 storage_identifier is malformed")
    return identifier


def _open_reader(path: Path) -> object:
    try:
        import rosbag2_py
    except ImportError as error:
        raise RuntimeError(
            "rosbag2_py is unavailable; source /opt/ros/jazzy/setup.bash"
        ) from error
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(path), storage_id=_storage_identifier(path)
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def _message_types(reader: object) -> dict[str, str]:
    return {entry.name: entry.type for entry in reader.get_all_topics_and_types()}


def _deserializer(type_name: str) -> Callable[[bytes], object]:
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python message support is unavailable; source the ROS and "
            "workspace setups"
        ) from error
    message_type = get_message(type_name)
    return lambda payload: deserialize_message(payload, message_type)


def _insert_unique(
    values: dict[int, object], key: int, value: object, context: str
) -> None:
    if key in values:
        raise ValueError(f"duplicate source timestamp in {context}: {key}")
    values[key] = value


def _pose_from_message(
    message: object,
    *,
    role: str,
    world_frame: str,
    reference: MessageRef,
) -> TimedPose | None:
    if str(message.role) != role:
        raise ValueError(f"TrackerSample role mismatch on {reference.topic}")
    if str(message.header.frame_id) != world_frame:
        raise ValueError(f"TrackerSample world frame mismatch on {reference.topic}")
    tracker_id = str(message.tracker_id)
    if _TRACKER_ID.fullmatch(tracker_id) is None:
        raise ValueError(f"TrackerSample tracker_id is malformed on {reference.topic}")
    realtime = int(message.host_realtime_ns)
    monotonic = int(message.host_monotonic_ns)
    if realtime <= 0 or monotonic <= 0:
        raise ValueError(f"TrackerSample host clock is invalid on {reference.topic}")
    if not bool(message.pose_valid) or int(message.tracking_status) & 0x0F != 2:
        return None
    pose = message.pose
    return TimedPose(
        role=role,
        tracker_id=tracker_id,
        host_realtime_ns=realtime,
        host_monotonic_ns=monotonic,
        transform=Transform(
            np.array(
                [pose.position.x, pose.position.y, pose.position.z],
                dtype=np.float64,
            ),
            np.array(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                dtype=np.float64,
            ),
        ),
        reference=reference,
    )


def read_unified_bag(
    bag_path: str | Path, config: AlignmentConfig
) -> BagDataset:
    path = Path(bag_path).resolve()
    if not path.is_dir():
        raise FileNotFoundError(f"rosbag2 directory does not exist: {path}")
    reader = _open_reader(path)
    observed_types = _message_types(reader)
    require_topic_types(observed_types, config)
    expected_types = configured_topic_types(config)
    selected_types = {
        topic: type_name
        for topic, type_name in expected_types.items()
        if topic in observed_types
    }
    deserializers = {
        topic: _deserializer(type_name)
        for topic, type_name in selected_types.items()
    }

    image_topics: dict[str, tuple[str, str, str]] = {}
    info_topics: dict[str, tuple[str, str]] = {}
    timing_topics: dict[str, object] = {}
    for camera in config.cameras:
        image_topics[camera.color_topic] = (camera.name, "color", camera.frame_id)
        image_topics[camera.depth_topic] = (camera.name, "depth", camera.frame_id)
        info_topics[camera.camera_info_topic] = (camera.name, camera.frame_id)
        timing_topics[camera.timing_topic] = camera
    tracker_topics = {tracker.sample_topic: tracker for tracker in config.trackers}
    generic_topics = {
        stream.topic: stream
        for stream in config.additional_streams
        if stream.topic in observed_types
    }

    color: dict[str, dict[int, MessageRef]] = {
        camera.name: {} for camera in config.cameras
    }
    depth: dict[str, dict[int, MessageRef]] = {
        camera.name: {} for camera in config.cameras
    }
    timing: dict[str, dict[int, TimingRecord]] = {
        camera.name: {} for camera in config.cameras
    }
    info_counts = {camera.name: 0 for camera in config.cameras}
    poses: dict[str, list[TimedPose]] = {
        tracker.role: [] for tracker in config.trackers
    }
    tracker_ids: dict[str, str] = {}
    generic: dict[str, list[GenericSample]] = {
        stream.name: [] for stream in config.additional_streams
    }
    message_counts = {topic: 0 for topic in selected_types}
    accepted_counts = {topic: 0 for topic in selected_types}
    clock_observations: dict[str, list[ClockObservation]] = {
        **{f"camera:{camera.name}": [] for camera in config.cameras},
        **{f"tracker:{tracker.role}": [] for tracker in config.trackers},
    }

    while reader.has_next():
        topic, payload, bag_timestamp_ns = reader.read_next()
        deserialize = deserializers.get(topic)
        if deserialize is None:
            continue
        sequence = message_counts[topic]
        message_counts[topic] += 1
        message = deserialize(payload)
        if topic in image_topics:
            camera_name, kind, frame_id = image_topics[topic]
            if str(message.header.frame_id) != frame_id:
                raise ValueError(f"Image frame mismatch on {topic}")
            source = stamp_to_nanoseconds(message.header.stamp)
            value = MessageRef(topic, sequence, int(bag_timestamp_ns), source)
            target = color[camera_name] if kind == "color" else depth[camera_name]
            _insert_unique(target, source, value, topic)
            accepted_counts[topic] += 1
        elif topic in info_topics:
            camera_name, frame_id = info_topics[topic]
            if str(message.header.frame_id) != frame_id:
                raise ValueError(f"CameraInfo frame mismatch on {topic}")
            info_counts[camera_name] += 1
            accepted_counts[topic] += 1
        elif topic in timing_topics:
            camera = timing_topics[topic]
            if (
                str(message.camera_name) != camera.name
                or str(message.camera_model) != camera.model
                or str(message.serial_number) != camera.serial
            ):
                raise ValueError(f"CameraFrameTiming identity mismatch on {topic}")
            if str(message.header.frame_id) != camera.frame_id:
                raise ValueError(f"CameraFrameTiming frame mismatch on {topic}")
            required_flags = (
                int(message.GROUP_VALID_COMMON_STAMP)
                | int(message.GROUP_VALID_IDENTITY)
                | int(message.GROUP_VALID_CALLBACK_CLOCKS)
            )
            if int(message.group_validity_flags) & required_flags != required_flags:
                continue
            source = int(message.shared_ros_timestamp_ns)
            realtime = int(message.group_host_realtime_ns)
            monotonic = int(message.group_host_monotonic_raw_ns)
            if source < 0 or realtime <= 0 or monotonic <= 0:
                raise ValueError(f"CameraFrameTiming host clock is invalid on {topic}")
            header_source = stamp_to_nanoseconds(message.header.stamp)
            if header_source != source:
                raise ValueError(f"CameraFrameTiming header stamp mismatch on {topic}")
            reference = MessageRef(
                topic, sequence, int(bag_timestamp_ns), source
            )
            record = TimingRecord(source, realtime, monotonic, reference)
            _insert_unique(timing[camera.name], source, record, topic)
            clock_observations[f"camera:{camera.name}"].append(
                ClockObservation(realtime, monotonic)
            )
            accepted_counts[topic] += 1
        elif topic in tracker_topics:
            tracker = tracker_topics[topic]
            header_stamp = stamp_to_nanoseconds(message.header.stamp)
            reference = MessageRef(
                topic, sequence, int(bag_timestamp_ns), header_stamp
            )
            pose = _pose_from_message(
                message,
                role=tracker.role,
                world_frame=config.world_frame,
                reference=reference,
            )
            clock_observations[f"tracker:{tracker.role}"].append(
                ClockObservation(
                    int(message.host_realtime_ns),
                    int(message.host_monotonic_ns),
                )
            )
            observed_id = str(message.tracker_id)
            existing_id = tracker_ids.setdefault(tracker.role, observed_id)
            if existing_id != observed_id:
                raise ValueError(f"Tracker identity changed for role {tracker.role}")
            if pose is not None:
                poses[tracker.role].append(pose)
                accepted_counts[topic] += 1
        else:
            stream = generic_topics[topic]
            source = extract_timestamp_ns(message, stream.timestamp_field)
            reference = MessageRef(
                topic, sequence, int(bag_timestamp_ns), source
            )
            generic[stream.name].append(
                GenericSample(stream.name, source, reference)
            )
            accepted_counts[topic] += 1

    camera_frames: dict[str, tuple[CameraFrame, ...]] = {}
    incomplete: dict[str, int] = {}
    for camera in config.cameras:
        if info_counts[camera.name] == 0:
            raise ValueError(f"bag contains no CameraInfo for {camera.name}")
        frames = join_camera_frames(
            camera.name,
            color[camera.name],
            depth[camera.name],
            timing[camera.name],
        )
        if not frames:
            raise ValueError(f"bag contains no complete frames for {camera.name}")
        camera_frames[camera.name] = frames
        union = set(color[camera.name]) | set(depth[camera.name]) | set(timing[camera.name])
        incomplete[camera.name] = len(union) - len(frames)
    for tracker in config.trackers:
        values = poses[tracker.role]
        if len(values) < 2:
            raise ValueError(
                f"bag contains fewer than two valid poses for {tracker.role}"
            )
        if any(
            left.host_realtime_ns >= right.host_realtime_ns
            for left, right in zip(values, values[1:])
        ):
            raise ValueError(f"Tracker host realtime is not increasing: {tracker.role}")
    for stream in config.additional_streams:
        values = generic[stream.name]
        if stream.required and not values:
            raise ValueError(f"bag contains no samples for {stream.name}")
        if any(
            left.timestamp_ns >= right.timestamp_ns
            for left, right in zip(values, values[1:])
        ):
            raise ValueError(f"generic stream time is not increasing: {stream.name}")
    return BagDataset(
        bag_path=path,
        storage_identifier=_storage_identifier(path),
        topic_types=selected_types,
        message_counts=message_counts,
        accepted_counts=accepted_counts,
        camera_frames=camera_frames,
        tracker_poses={role: tuple(values) for role, values in poses.items()},
        additional_samples={
            name: tuple(values) for name, values in generic.items()
        },
        tracker_ids=tracker_ids,
        incomplete_camera_groups=incomplete,
        clock_observations={
            name: tuple(values)
            for name, values in clock_observations.items()
        },
    )
