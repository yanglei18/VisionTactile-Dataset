from __future__ import annotations

import re
from collections.abc import Mapping, Sequence


CAMERA_TOPIC_SUFFIXES = (
    "color/image_raw",
    "depth/image_rect_raw",
    "color/camera_info",
    "frame_timing",
)
TRACKER_ROLES = ("left_wrist", "right_wrist", "torso")
SYSTEM_TOPICS: tuple[str, ...] = ()
SCHEMA_VERSION = "unified-dataset-v1"
_CAMERA_NAME_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_ABSOLUTE_TOPIC_PATTERN = re.compile(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+")
_CAMERA_TOPIC_TYPES = {
    "color/image_raw": "sensor_msgs/msg/Image",
    "depth/image_rect_raw": "sensor_msgs/msg/Image",
    "color/camera_info": "sensor_msgs/msg/CameraInfo",
    "frame_timing": "vt_camera_msgs/msg/CameraFrameTiming",
}


def camera_topics(camera_names: Sequence[str]) -> tuple[str, ...]:
    return tuple(
        f"/{name}/{suffix}"
        for name in sorted(camera_names)
        for suffix in CAMERA_TOPIC_SUFFIXES
    )


def tracker_topics() -> tuple[str, ...]:
    return tuple(f"/vive/{role}/sample" for role in TRACKER_ROLES)


def expected_topics(
    camera_names: Sequence[str],
    additional_topics: Sequence[str] = (),
) -> tuple[str, ...]:
    return (
        *camera_topics(camera_names),
        *tracker_topics(),
        *tuple(sorted(additional_topics)),
    )


def expected_topic_type(topic: str) -> str:
    if not isinstance(topic, str):
        raise ValueError(f"unsupported unified-dataset-v1 topic: {topic}")
    if topic.startswith("/"):
        camera_name, separator, suffix = topic[1:].partition("/")
        if (
            separator
            and _CAMERA_NAME_PATTERN.fullmatch(camera_name) is not None
            and suffix in CAMERA_TOPIC_SUFFIXES
        ):
            return _CAMERA_TOPIC_TYPES[suffix]
        parts = topic.split("/")
        if (
            len(parts) == 4
            and parts[1] == "vive"
            and parts[2] in TRACKER_ROLES
            and parts[3] == "sample"
        ):
            return "vt_tracker_msgs/msg/TrackerSample"
    raise ValueError(f"unsupported unified-dataset-v1 topic: {topic}")


def expected_topic_types(
    camera_names: Sequence[str],
    additional_stream_types: Mapping[str, str] | None = None,
) -> Mapping[str, str]:
    additional = dict(additional_stream_types or {})
    for topic, type_name in additional.items():
        if _ABSOLUTE_TOPIC_PATTERN.fullmatch(topic) is None:
            raise ValueError(f"invalid additional stream topic: {topic}")
        if not isinstance(type_name, str) or "/msg/" not in type_name:
            raise ValueError(f"invalid additional stream type: {type_name}")
    result = {
        topic: expected_topic_type(topic)
        for topic in expected_topics(camera_names)
    }
    result.update({topic: additional[topic] for topic in sorted(additional)})
    return result
