"""Persistent CPU MANO display worker; JSON lines only, no sensor ownership.

Input poses are already solved MANO axis-angle rotations including the root.
This worker only renders them with the user's trusted licensed model. It never
re-estimates pose, loads calibration, opens serial, or imports ROS/Qt.
"""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
import json
import sys

MAX_INPUT_CHARS = 32 * 1024
MAX_OUTPUT_CHARS = 512 * 1024
JOINT_TOLERANCE_M = .002


def initial_bundle(backend, hand):
    import numpy as np
    from .guided_calibration import model_motion_axes

    neutral = backend.render_mano(np.zeros((16, 3)), np.zeros(10))
    geometry = {name: neutral[name] for name in ("vertices_m", "joints_m", "faces")}
    geometry["provenance"] = {"hand": hand}
    return {"schema_version": 1, "neutral_geometry": geometry,
            "model_axes": model_motion_axes(neutral["joints_m"], hand=hand)}


def render_request(backend, request, hand):
    import numpy as np

    if not isinstance(request, dict) or type(request.get("id")) is not int or request["id"] < 0:
        raise ValueError("preview request requires a nonnegative integer id")
    mano = request.get("mano")
    if not isinstance(mano, dict) or mano.get("hand") != hand:
        raise ValueError("preview pose hand does not match model")
    pose, shape = np.asarray(mano.get("pose")), np.asarray(mano.get("shape"))
    if (pose.shape != (48,) or shape.shape != (10,)
            or pose.dtype.kind not in "fiu" or shape.dtype.kind not in "fiu"
            or not np.isfinite(pose).all() or not np.isfinite(shape).all()):
        raise ValueError("preview pose requires 48 finite rotations and 10 shape parameters")
    geometry = backend.render_mano(pose.reshape(16, 3), shape)
    if request.get("joints") is not None:
        expected = np.asarray(request["joints"])
        if (expected.shape != (21, 3) or expected.dtype.kind not in "fiu"
                or not np.isfinite(expected).all()):
            raise ValueError("published joints must be finite (21,3) metres")
        residual = np.linalg.norm(np.asarray(geometry["joints_m"]) - expected, axis=1)
        if residual.max() > JOINT_TOLERANCE_M:
            raise ValueError(f"MANO model/pose does not match published joints ({residual.max()*1000:.1f} mm)")
    return {"type": "geometry", "id": request["id"], "geometry": {
        "vertices_m": geometry["vertices_m"], "joints_m": geometry["joints_m"]}}


def _write(sink, value):
    text = json.dumps(value, allow_nan=False, separators=(",", ":"))
    if len(text) > MAX_OUTPUT_CHARS:
        raise ValueError("preview output exceeds size limit")
    sink.write(text + "\n")
    sink.flush()


def serve(backend, hand, source, sink):
    with redirect_stdout(sys.stderr):
        bundle = initial_bundle(backend, hand)
    _write(sink, {"type": "ready", "bundle": bundle})
    while True:
        line = source.readline(MAX_INPUT_CHARS + 1)
        if not line:
            return
        if len(line) > MAX_INPUT_CHARS or not line.endswith("\n"):
            raise ValueError("preview input exceeds size limit or is incomplete")
        request = None
        try:
            request = json.loads(line)
            with redirect_stdout(sys.stderr):
                result = render_request(backend, request, hand)
        except (TypeError, ValueError, RuntimeError, KeyError) as exc:
            request_id = request.get("id") if isinstance(request, dict) else None
            result = {"type": "error", "id": request_id, "message": str(exc)[:1024]}
        _write(sink, result)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mano-assets", required=True)
    parser.add_argument("--hand", required=True, choices=("right", "left"))
    args = parser.parse_args(argv)
    try:
        with redirect_stdout(sys.stderr):
            import torch
            from .mano import ManoBackend

            torch.set_num_threads(1)
            backend = ManoBackend(args.mano_assets, hand=args.hand)
        serve(backend, args.hand, sys.stdin, sys.stdout)
    except Exception as exc:
        print(f"MANO 位姿显示进程失败：{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
