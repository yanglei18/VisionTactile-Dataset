"""Identity expectations for canonical topics, without constraining custom topics."""

from __future__ import annotations

import re


_CANONICAL = re.compile(r"/gloves/(left|right)/(state|tactile/samples)")


def stream_identity(topic: str, type_name: str, *, expected_hand: str | None = None,
                    expected_frame_id: str | None = None) -> tuple[str | None, str | None]:
    match = _CANONICAL.fullmatch(topic)
    canonical_hand = None
    canonical_frame = None
    if match is not None:
        hand, suffix = match.groups()
        if suffix == "state" and type_name == "vt_glove_msgs/msg/GloveState":
            canonical_hand, canonical_frame = hand, f"glove_{hand}_reference"
        elif suffix == "tactile/samples" and type_name == "vt_tactile_msgs/msg/TactileSampleArray":
            canonical_hand, canonical_frame = hand, f"{hand}_tactile"
    if expected_hand is not None and expected_hand not in {"left", "right"}:
        raise ValueError("expected_hand must be left or right")
    if canonical_hand is not None and expected_hand not in {None, canonical_hand}:
        raise ValueError(f"expected_hand contradicts canonical topic: {topic}")
    return expected_hand or canonical_hand, expected_frame_id or canonical_frame


def check_identity(message: object, topic: str, *, expected_hand: str | None,
                   expected_frame_id: str | None) -> None:
    if expected_hand is not None and getattr(message, "hand", None) != expected_hand:
        raise ValueError(f"hand mismatch on {topic}: expected {expected_hand}")
    if (expected_frame_id is not None
            and getattr(getattr(message, "header", None), "frame_id", None) != expected_frame_id):
        raise ValueError(f"frame_id mismatch on {topic}: expected {expected_frame_id}")
