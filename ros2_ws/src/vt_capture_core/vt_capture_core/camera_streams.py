"""Versioned camera bundles shared by recipes, frozen plans and the Recorder.

Version 1 is retained verbatim for historical plans. Version 2 adds the native
depth intrinsics and the factory depth-to-color transform; it makes no claim
about an external Tracker, another camera or a world coordinate system.
"""

from __future__ import annotations


RECIPE_VERSION = 2
SUPPORTED_CAPTURE_SCHEMAS = ("unified-dataset-v1", "unified-dataset-v2")
_LEGACY_CAMERA_STREAMS = (
    ("color/image_raw", "sensor_msgs/msg/Image"),
    ("depth/image_rect_raw", "sensor_msgs/msg/Image"),
    ("color/camera_info", "sensor_msgs/msg/CameraInfo"),
    ("frame_timing", "vt_camera_msgs/msg/CameraFrameTiming"),
)
CAMERA_STREAMS = (
    *_LEGACY_CAMERA_STREAMS,
    ("depth/camera_info", "sensor_msgs/msg/CameraInfo"),
    ("extrinsics/depth_to_color", "realsense2_camera_msgs/msg/Extrinsics"),
)
CAMERA_TOPIC_SUFFIXES = tuple(suffix for suffix, _type_name in CAMERA_STREAMS)


def capture_schema_version(recipe_version: int = RECIPE_VERSION) -> str:
    """Map an explicit recipe version to its corresponding bag schema."""
    if type(recipe_version) is not int or recipe_version not in (1, RECIPE_VERSION):
        raise ValueError("unsupported recipe version; expected integer 1 or 2")
    return SUPPORTED_CAPTURE_SCHEMAS[recipe_version - 1]


def camera_streams(recipe_version: int = RECIPE_VERSION) -> tuple[tuple[str, str], ...]:
    """Return the immutable, ordered (suffix, ROS message type) contract."""
    capture_schema_version(recipe_version)
    return _LEGACY_CAMERA_STREAMS if recipe_version == 1 else CAMERA_STREAMS
