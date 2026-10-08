"""Bounded per-topic recorder buffers; never change publisher QoS or sampling."""

from __future__ import annotations

import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass


MIN_DEPTH = 1
MAX_DEPTH = 4096
_TOPIC = re.compile(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+")
_TYPE_DEPTHS = {
    "vt_tactile_msgs/msg/TactileSampleArray": 512,
    "vt_glove_msgs/msg/GloveState": 256,
}


@dataclass(frozen=True)
class RecorderQosOverride:
    topic: str
    depth: int | None = None
    reliability: str | None = None


def validate_overrides(
    overrides: Sequence[RecorderQosOverride], topics: Sequence[str]
) -> tuple[RecorderQosOverride, ...]:
    """Reject typos and unknown topics before a session is started."""
    if isinstance(overrides, (str, bytes)) or not isinstance(overrides, Sequence):
        raise ValueError("recording.qos_overrides must be explicit topic overrides")
    allowed = set(topics)
    seen: set[str] = set()
    for override in overrides:
        if not isinstance(override, RecorderQosOverride):
            raise ValueError("invalid recording.qos_overrides entry")
        topic = override.topic
        if not isinstance(topic, str) or _TOPIC.fullmatch(topic) is None:
            raise ValueError(
                "recording.qos_overrides topic must be an absolute "
                "wildcard-free ROS name"
            )
        if topic not in allowed:
            raise ValueError(f"recording.qos_overrides topic is not recorded: {topic}")
        if topic in seen:
            raise ValueError(f"duplicate recording.qos_overrides topic: {topic}")
        seen.add(topic)
        if override.depth is not None and (
            type(override.depth) is not int
            or not MIN_DEPTH <= override.depth <= MAX_DEPTH
        ):
            raise ValueError(
                "recording.qos_overrides depth must be an integer "
                f"in [{MIN_DEPTH}, {MAX_DEPTH}]"
            )
        if override.reliability is not None and (
            type(override.reliability) is not str
            or override.reliability not in {"best_effort", "reliable"}
        ):
            raise ValueError(
                "recording.qos_overrides reliability must be best_effort or reliable"
            )
        if override.depth is None and override.reliability is None:
            raise ValueError(
                "recording.qos_overrides profile must set depth or reliability"
            )
    return tuple(sorted(overrides, key=lambda value: value.topic))


def load_overrides(
    value: object, topics: Sequence[str]
) -> tuple[RecorderQosOverride, ...]:
    if not isinstance(value, Mapping):
        raise ValueError("recording.qos_overrides must be a topic-to-profile mapping")
    result = []
    for topic, profile in value.items():
        if not isinstance(profile, Mapping):
            raise ValueError("recording.qos_overrides profile must be a mapping")
        if set(profile) - {"depth", "reliability"}:
            raise ValueError(
                "unexpected recording.qos_overrides profile key; "
                "only depth and reliability are supported"
            )
        # Explicit nulls are errors, not silently ignored user choices.
        if any(entry is None for entry in profile.values()):
            raise ValueError("recording.qos_overrides profile values must not be null")
        result.append(
            RecorderQosOverride(topic, profile.get("depth"), profile.get("reliability"))
        )
    return validate_overrides(result, topics)


def profiles_for_topics(
    topics: Sequence[str],
    *,
    topic_types: Mapping[str, str] | None = None,
    overrides: Sequence[RecorderQosOverride] = (),
) -> dict[str, dict[str, object]]:
    selected = {value.topic: value for value in validate_overrides(overrides, topics)}
    types = {} if topic_types is None else topic_types
    if not isinstance(types, Mapping) or any(
        topic not in topics or not isinstance(type_name, str)
        for topic, type_name in types.items()
    ):
        raise ValueError("recorder topic_types must describe only recorded topics")
    result = {}
    for topic in topics:
        profile: dict[str, object] = {
            "history": "keep_last",
            "depth": _TYPE_DEPTHS.get(types.get(topic, ""), 30),
            "reliability": "best_effort",
            "durability": "volatile",
        }
        if topic in selected:
            override = selected[topic]
            if override.depth is not None:
                profile["depth"] = override.depth
            if override.reliability is not None:
                profile["reliability"] = override.reliability
        result[topic] = profile
    return result
