"""Select independent pose/tactile streams and emit matching unified profiles.

The fixed camera/Tracker core is preserved. These profiles are not a replacement
for the independent glove/tactile capture paths when that core is absent.
"""

from __future__ import annotations

from copy import deepcopy
import os
from pathlib import Path
import shutil
import tempfile
from collections.abc import Mapping, Sequence

import yaml

from .config import load_config
from .glove_adapter import GLOVE_STATE_TYPE
from .tactile_adapter import TACTILE_SAMPLE_TYPE


def parse_hands(value: str) -> tuple[str, ...]:
    if value == "none":
        return ()
    if value == "both":
        return ("left", "right")
    hands = value.split(",")
    if len(hands) != len(set(hands)) or not hands or any(hand not in {"left", "right"} for hand in hands):
        raise ValueError("hands must be none, left, right, both, or left,right")
    return tuple(sorted(hands))


def selected_streams(*, pose_hands: Sequence[str] = (), tactile_hands: Sequence[str] = (),
                     require_tactile: bool = False,
                     reference_node_ids: Mapping[str, int] | None = None) -> list[dict[str, object]]:
    """A pure selector, reusable without ROS or filesystem writes."""
    selections = {}
    for name, values in (("pose", pose_hands), ("tactile", tactile_hands)):
        if isinstance(values, (str, bytes)):
            raise ValueError(f"{name}_hands must be a sequence of hand names, not text")
        values = tuple(values)
        if len(set(values)) != len(values) or any(hand not in {"left", "right"} for hand in values):
            raise ValueError(f"invalid or duplicate {name} hand")
        selections[name] = tuple(sorted(values))
    if type(require_tactile) is not bool:
        raise ValueError("require_tactile must be bool")
    nodes = dict(reference_node_ids or {})
    for hand, node in nodes.items():
        if hand not in selections["pose"]:
            raise ValueError("reference_node_ids must only name enabled pose hands")
        if type(node) is not int or not 1 <= node <= 12:
            raise ValueError("reference_node_id must be an integer from 1 to 12")
    streams = []
    for kind, hands in selections.items():
        for hand in hands:
            pose = kind == "pose"
            stream = {
                "name": f"{hand}_{'glove' if pose else 'tactile'}",
                "topic": f"/gloves/{hand}/{'state' if pose else 'tactile/samples'}",
                "type": GLOVE_STATE_TYPE if pose else TACTILE_SAMPLE_TYPE,
                "time_source": "field", "timestamp_field": "host_realtime_ns",
                "strategy": "nearest" if pose else "previous",
                "max_delta_ms": 20.0 if pose else 100.0,
                "required": True if pose else require_tactile,
            }
            if pose and hand in nodes:
                stream["reference_node_id"] = nodes[hand]
            streams.append(stream)
    return sorted(streams, key=lambda stream: stream["topic"])


def profile_documents(capture: Mapping[str, object], alignment: Mapping[str, object], **selection):
    """Replace only extension lists and prune overrides for deselected extensions."""
    capture, alignment = deepcopy(dict(capture)), deepcopy(dict(alignment))
    if not isinstance(capture.get("recording"), dict) or "additional_streams" not in alignment:
        raise ValueError("templates must be capture and alignment configurations")
    streams = selected_streams(**selection)
    recording = capture["recording"]
    old_topics = {stream["topic"] for stream in recording.get("additional_streams", [])}
    new_topics = {stream["topic"] for stream in streams}
    recording["additional_streams"] = [{"topic": stream["topic"], "type": stream["type"]}
                                       for stream in streams]
    if isinstance(recording.get("qos_overrides"), dict):
        recording["qos_overrides"] = {
            topic: value for topic, value in recording["qos_overrides"].items()
            if topic not in old_topics or topic in new_topics
        }
    alignment["additional_streams"] = streams
    return capture, alignment


def make_profiles(*, capture_template: str | Path, alignment_template: str | Path,
                  output_directory: str | Path, **selection) -> Path:
    """Write a fresh directory; never overwrite templates or an existing profile."""
    load_config(alignment_template)
    documents = [yaml.safe_load(Path(path).read_text(encoding="utf-8"))
                 for path in (capture_template, alignment_template)]
    if any(not isinstance(document, dict) for document in documents):
        raise ValueError("profile templates must contain YAML mappings")
    capture, alignment = profile_documents(*documents, **selection)
    target = Path(output_directory).absolute()
    if target.exists() or target.is_symlink():
        raise FileExistsError(f"refusing to overwrite profile directory: {target}")
    if not target.parent.is_dir():
        raise FileNotFoundError(f"output parent does not exist: {target.parent}")
    temporary = Path(tempfile.mkdtemp(prefix=f".{target.name}.", dir=target.parent))
    try:
        for name, document in (("cameras.yaml", capture), ("alignment.yaml", alignment)):
            (temporary / name).write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        load_config(temporary / "alignment.yaml")
        # mkdir is exclusive, including when another producer wins a race.
        target.mkdir(mode=0o700)
        for source in temporary.iterdir():
            os.rename(source, target / source.name)
    finally:
        shutil.rmtree(temporary)
    return target
