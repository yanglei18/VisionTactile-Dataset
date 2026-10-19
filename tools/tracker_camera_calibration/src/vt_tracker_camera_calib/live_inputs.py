"""Shared read-only ROS input decoding for both preview and guided collection."""
from __future__ import annotations
import re
from types import SimpleNamespace
import numpy as np
from .bag_reader import decode_image_message, stamp_to_nanoseconds
from .config import CalibrationConfig
from .transforms import Transform
_TRACKER_ID = re.compile(r"[0-9a-f]{64}")

def load_ros() -> SimpleNamespace:
    try:
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import (
            QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
        )
        from sensor_msgs.msg import Image, CameraInfo
        from vt_camera_msgs.msg import CameraFrameTiming
        from vt_tracker_msgs.msg import TrackerSample
    except (ImportError, OSError) as error:
        raise RuntimeError(
            "ROS 2 预览依赖不可用；请先 source /opt/ros/jazzy/setup.bash 和 "
            "ros2_ws/install/setup.bash，再使用 .venv-calibration 启动。"
        ) from error
    return SimpleNamespace(**locals())


def image_rgb(message: object, config: CalibrationConfig) -> tuple[np.ndarray, int]:
    if str(message.header.frame_id) != config.camera_frame:
        raise ValueError("Image frame_id 与配置不一致")
    stamp = stamp_to_nanoseconds(message.header.stamp)
    image, encoding = decode_image_message(message)
    if encoding == "mono8":
        rgb = np.repeat(image[:, :, None], 3, axis=2)
    elif encoding in {"bgr8", "bgra8"}:
        rgb = image[:, :, :3][:, :, ::-1].copy()
    else:
        rgb = image[:, :, :3].copy()
    rgb.setflags(write=False)
    return rgb, stamp


def timing_pair(message: object, config: CalibrationConfig) -> tuple[int, int]:
    for field, expected in (("camera_name", config.camera_name),
                            ("camera_model", config.camera_model),
                            ("serial_number", config.camera_serial)):
        if str(getattr(message, field)) != expected:
            raise ValueError(f"CameraFrameTiming {field} 与配置不一致")
    required = (int(message.GROUP_VALID_COMMON_STAMP)
                | int(message.GROUP_VALID_IDENTITY)
                | int(message.GROUP_VALID_CALLBACK_CLOCKS))
    if int(message.group_validity_flags) & required != required:
        raise ValueError("CameraFrameTiming 缺少有效共同时间戳/身份/主机回调时钟")
    stamp, host = int(message.shared_ros_timestamp_ns), int(message.group_host_realtime_ns)
    if stamp < 0 or host <= 0:
        raise ValueError("CameraFrameTiming 主机时间映射无效")
    return stamp, host


def tracker_pose(message: object, config: CalibrationConfig) -> tuple[str, Transform | None]:
    if str(message.role) != config.tracker_role:
        raise ValueError("TrackerSample role 与配置不一致")
    if str(message.header.frame_id) != config.world_frame:
        raise ValueError("TrackerSample world frame 与配置不一致")
    if not bool(message.pose_valid) or int(message.tracking_status) & 0x0F != 2:
        return "", None
    identity = str(message.tracker_id)
    if _TRACKER_ID.fullmatch(identity) is None:
        raise ValueError("TrackerSample tracker_id 不是有效的 64 位身份指纹")
    if int(message.host_realtime_ns) <= 0:
        raise ValueError("TrackerSample host_realtime_ns 无效")
    p, q = message.pose.position, message.pose.orientation
    transform = Transform.from_quaternion_xyzw(
        (float(p.x), float(p.y), float(p.z)),
        (float(q.x), float(q.y), float(q.z), float(q.w)),
    )
    return identity, transform
