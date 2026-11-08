"""Synthetic hand demonstrations, independent of Qt and device acquisition.

The command runs in the configured numerical environment. It verifies the
explicit model bundle and writes only teaching geometry, never live poses.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def finger_clips(renderer, neutral):
    """The shared teaching motion, sampled at most five degrees per step."""
    open_vertices = np.asarray(neutral["vertices_m"], dtype=np.float32)
    open_joints = np.asarray(neutral["joints_m"], dtype=np.float32)
    fingers = np.zeros((16, 3))
    for base in (1, 4, 7, 10):
        fingers[base:base + 3, 2] = np.deg2rad([65., 80., 52.])
    thumb = np.zeros((16, 3))
    thumb[15, 2] = np.deg2rad(65.)
    clips = {}
    for action, target in (("flex_fingers", fingers), ("flex_thumb", thumb)):
        vertices, joints = [open_vertices], [open_joints]
        for phase in np.linspace(0., 1., 17)[1:]:
            frame = renderer.render(target * phase)
            vertices.append(frame["vertices_m"])
            joints.append(frame["joints_m"])
        clips[action] = (np.asarray(vertices, dtype=np.float32), np.asarray(joints, dtype=np.float32))
    return clips


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--resources", required=True, type=Path)
    parser.add_argument("--hand", required=True, choices=("left", "right"))
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    from .hand_service_resources import validate_bundle_manifest
    from .mano import ManoBackend
    root, _ = validate_bundle_manifest(args.resources, hand=args.hand)
    bundle = json.loads((root / "hand.json").read_text(encoding="utf-8"))
    clips = finger_clips(ManoBackend(root / "mano", hand=args.hand, compact_live_geometry=True),
                         bundle["neutral_geometry"])
    document = dict(schema_version=1, hand=args.hand, synthetic_demonstration=True,
                    clips={key: [vertices.tolist(), joints.tolist()] for key, (vertices, joints) in clips.items()})
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(document, stream, allow_nan=False, separators=(",", ":"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
