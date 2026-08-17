from __future__ import annotations

import re
from collections.abc import Mapping, Sequence


CAMERA_TOPIC_SUFFIXES = (
    "color/image_raw",
    "depth/image_rect_raw",
    "frame_timing",
)
SYSTEM_TOPICS: tuple[str, ...] = ()
SCHEMA_VERSION = "recorder-only-v1"
_CAMERA_NAME_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_CAMERA_TOPIC_TYPES = {
    "color/image_raw": "sensor_msgs/msg/Image",
    "depth/image_rect_raw": "sensor_msgs/msg/Image",
    "frame_timing": "vt_camera_msgs/msg/CameraFrameTiming",
}


def expected_topics(camera_names: Sequence[str]) -> tuple[str, ...]:
    return tuple(
        f"/{name}/{suffix}"
        for name in sorted(camera_names)
        for suffix in CAMERA_TOPIC_SUFFIXES
    )


def expected_topic_type(topic: str) -> str:
    if not isinstance(topic, str):
        raise ValueError(f"unsupported recorder-only-v1 topic: {topic}")
    if topic.startswith("/"):
        camera_name, separator, suffix = topic[1:].partition("/")
        if (
            separator
            and _CAMERA_NAME_PATTERN.fullmatch(camera_name) is not None
            and suffix in CAMERA_TOPIC_SUFFIXES
        ):
            return _CAMERA_TOPIC_TYPES[suffix]
    raise ValueError(f"unsupported recorder-only-v1 topic: {topic}")


def expected_topic_types(camera_names: Sequence[str]) -> Mapping[str, str]:
    return {
        topic: expected_topic_type(topic)
        for topic in expected_topics(camera_names)
    }
