from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
from typing import Callable

import numpy as np
import yaml

from .charuco import detect_board_pose
from .config import CalibrationConfig
from .model import BoardObservation, CameraIntrinsics, TimedTransform
from .transforms import Transform


_TRACKER_ID_PATTERN = re.compile(r"[0-9a-f]{64}")


@dataclass(frozen=True)
class BagObservations:
    intrinsics: CameraIntrinsics
    tracker_samples: tuple[TimedTransform, ...]
    board_observations: tuple[BoardObservation, ...]
    tracker_id: str
    image_count: int
    timed_image_count: int
    rejected_image_count: int


def stamp_to_nanoseconds(stamp: object) -> int:
    seconds = getattr(stamp, "sec", None)
    nanoseconds = getattr(stamp, "nanosec", None)
    if type(seconds) is not int or type(nanoseconds) is not int:
        raise ValueError("message stamp is malformed")
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("message stamp is outside the supported range")
    return seconds * 1_000_000_000 + nanoseconds


def decode_image_message(message: object) -> tuple[np.ndarray, str]:
    width = int(getattr(message, "width"))
    height = int(getattr(message, "height"))
    step = int(getattr(message, "step"))
    encoding = str(getattr(message, "encoding")).lower()
    channels_by_encoding = {
        "mono8": 1,
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
    }
    channels = channels_by_encoding.get(encoding)
    if width <= 0 or height <= 0 or channels is None:
        raise ValueError(
            f"unsupported or malformed image: {width}x{height} {encoding}"
        )
    packed_width = width * channels
    if step < packed_width:
        raise ValueError("image step is smaller than packed row width")
    data = np.frombuffer(bytes(getattr(message, "data")), dtype=np.uint8)
    required_size = height * step
    if data.size < required_size:
        raise ValueError("image data is shorter than height*step")
    rows = data[:required_size].reshape(height, step)[:, :packed_width]
    if channels == 1:
        image = rows.reshape(height, width)
    else:
        image = rows.reshape(height, width, channels)
    return image, encoding


def intrinsics_from_message(message: object) -> CameraIntrinsics:
    distortion_model = str(getattr(message, "distortion_model"))
    if distortion_model not in {"plumb_bob", "rational_polynomial"}:
        raise ValueError(
            f"unsupported CameraInfo distortion model: {distortion_model}"
        )
    return CameraIntrinsics(
        width=int(getattr(message, "width")),
        height=int(getattr(message, "height")),
        camera_matrix=np.asarray(
            getattr(message, "k"), dtype=np.float64
        ).reshape(3, 3),
        distortion=np.asarray(getattr(message, "d"), dtype=np.float64),
        distortion_model=distortion_model,
    )


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
    if not isinstance(identifier, str) or not identifier:
        raise ValueError("rosbag2 storage_identifier is malformed")
    return identifier


def _open_reader(path: Path, topics: tuple[str, ...]) -> object:
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
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader


def _message_types(reader: object) -> dict[str, str]:
    return {entry.name: entry.type for entry in reader.get_all_topics_and_types()}


def _deserializer(type_name: str) -> Callable[[bytes], object]:
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python message support is unavailable; "
            "source the ROS and workspace setups"
        ) from error
    message_type = get_message(type_name)
    return lambda payload: deserialize_message(payload, message_type)


def _require_topics(types: dict[str, str], config: CalibrationConfig) -> None:
    required = {
        config.image_topic: "sensor_msgs/msg/Image",
        config.timing_topic: "vt_camera_msgs/msg/CameraFrameTiming",
        config.tracker_sample_topic: "vt_tracker_msgs/msg/TrackerSample",
    }
    if config.fallback_intrinsics is None:
        required[config.camera_info_topic] = "sensor_msgs/msg/CameraInfo"
    for topic, expected_type in required.items():
        actual_type = types.get(topic)
        if actual_type is None:
            raise ValueError(f"required topic is absent from bag: {topic}")
        if actual_type != expected_type:
            raise ValueError(
                f"topic type mismatch for {topic}: "
                f"expected {expected_type}, got {actual_type}"
            )


def _same_intrinsics(left: CameraIntrinsics, right: CameraIntrinsics) -> bool:
    return (
        left.width == right.width
        and left.height == right.height
        and left.distortion_model == right.distortion_model
        and left.distortion.shape == right.distortion.shape
        and np.allclose(left.camera_matrix, right.camera_matrix, atol=1e-12)
        and np.allclose(left.distortion, right.distortion, atol=1e-12)
    )


def read_calibration_bag(
    bag_path: str | Path,
    config: CalibrationConfig,
) -> BagObservations:
    path = Path(bag_path).resolve()
    if not path.is_dir():
        raise FileNotFoundError(f"rosbag2 directory does not exist: {path}")
    first_pass_topics = (
        config.camera_info_topic,
        config.timing_topic,
        config.tracker_sample_topic,
    )
    reader = _open_reader(path, first_pass_topics)
    types = _message_types(reader)
    _require_topics(types, config)
    deserializers = {
        topic: _deserializer(type_name)
        for topic, type_name in types.items()
        if topic in first_pass_topics
    }
    intrinsics = config.fallback_intrinsics
    timing_by_stamp: dict[int, int] = {}
    tracker_samples: list[TimedTransform] = []
    tracker_ids: set[str] = set()

    while reader.has_next():
        topic, payload, _ = reader.read_next()
        deserialize = deserializers.get(topic)
        if deserialize is None:
            continue
        message = deserialize(payload)
        if topic == config.camera_info_topic:
            frame_id = str(message.header.frame_id)
            if frame_id != config.camera_frame:
                raise ValueError(
                    f"CameraInfo frame mismatch: "
                    f"expected {config.camera_frame}, got {frame_id}"
                )
            candidate = intrinsics_from_message(message)
            if intrinsics is not None and not _same_intrinsics(
                intrinsics, candidate
            ):
                raise ValueError("CameraInfo changed within the calibration bag")
            intrinsics = candidate
        elif topic == config.timing_topic:
            if str(message.camera_name) != config.camera_name:
                raise ValueError(
                    "CameraFrameTiming camera_name does not match config"
                )
            if str(message.camera_model) != config.camera_model:
                raise ValueError(
                    "CameraFrameTiming camera_model does not match config"
                )
            if str(message.serial_number) != config.camera_serial:
                raise ValueError(
                    "CameraFrameTiming serial_number does not match config"
                )
            required_flags = (
                int(message.GROUP_VALID_COMMON_STAMP)
                | int(message.GROUP_VALID_IDENTITY)
                | int(message.GROUP_VALID_CALLBACK_CLOCKS)
            )
            if int(message.group_validity_flags) & required_flags != required_flags:
                continue
            source_stamp_ns = int(message.shared_ros_timestamp_ns)
            host_realtime_ns = int(message.group_host_realtime_ns)
            if source_stamp_ns < 0 or host_realtime_ns <= 0:
                continue
            existing = timing_by_stamp.setdefault(source_stamp_ns, host_realtime_ns)
            if existing != host_realtime_ns:
                raise ValueError("conflicting timing observations for one image stamp")
        elif topic == config.tracker_sample_topic:
            if str(message.role) != config.tracker_role:
                raise ValueError("TrackerSample role does not match config")
            if str(message.header.frame_id) != config.world_frame:
                raise ValueError("TrackerSample world frame does not match config")
            if (
                not bool(message.pose_valid)
                or int(message.tracking_status) & 0x0F != 2
            ):
                continue
            tracker_id = str(message.tracker_id)
            if _TRACKER_ID_PATTERN.fullmatch(tracker_id) is None:
                raise ValueError("TrackerSample tracker_id is malformed")
            if int(message.host_realtime_ns) <= 0:
                raise ValueError("TrackerSample host_realtime_ns is invalid")
            tracker_ids.add(tracker_id)
            tracker_samples.append(
                TimedTransform(
                    timestamp_ns=int(message.host_realtime_ns),
                    transform=Transform.from_quaternion_xyzw(
                        (
                            float(message.pose.position.x),
                            float(message.pose.position.y),
                            float(message.pose.position.z),
                        ),
                        (
                            float(message.pose.orientation.x),
                            float(message.pose.orientation.y),
                            float(message.pose.orientation.z),
                            float(message.pose.orientation.w),
                        ),
                    ),
                )
            )
    if intrinsics is None:
        raise ValueError(
            "no usable CameraInfo was recorded and no fallback was configured"
        )
    if len(tracker_ids) != 1:
        raise ValueError(
            f"expected one stable tracker_id, observed {len(tracker_ids)}"
        )
    if len(tracker_samples) < 2:
        raise ValueError("bag contains fewer than two valid TrackerSample messages")
    if not timing_by_stamp:
        raise ValueError("bag contains no valid CameraFrameTiming mappings")

    image_reader = _open_reader(path, (config.image_topic,))
    image_types = _message_types(image_reader)
    image_deserialize = _deserializer(image_types[config.image_topic])
    board_observations: list[BoardObservation] = []
    image_count = 0
    timed_image_count = 0
    rejected_image_count = 0
    while image_reader.has_next():
        topic, payload, _ = image_reader.read_next()
        if topic != config.image_topic:
            continue
        image_count += 1
        message = image_deserialize(payload)
        if str(message.header.frame_id) != config.camera_frame:
            raise ValueError(
                f"Image frame mismatch: expected {config.camera_frame}, "
                f"got {message.header.frame_id}"
            )
        source_stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        host_realtime_ns = timing_by_stamp.get(source_stamp_ns)
        if host_realtime_ns is None:
            rejected_image_count += 1
            continue
        timed_image_count += 1
        image, encoding = decode_image_message(message)
        observation = detect_board_pose(
            image,
            encoding=encoding,
            intrinsics=intrinsics,
            config=config.board,
            timestamp_ns=host_realtime_ns,
            source_stamp_ns=source_stamp_ns,
        )
        if observation is None:
            rejected_image_count += 1
        else:
            board_observations.append(observation)
    return BagObservations(
        intrinsics=intrinsics,
        tracker_samples=tuple(
            sorted(tracker_samples, key=lambda value: value.timestamp_ns)
        ),
        board_observations=tuple(board_observations),
        tracker_id=next(iter(tracker_ids)),
        image_count=image_count,
        timed_image_count=timed_image_count,
        rejected_image_count=rejected_image_count,
    )
