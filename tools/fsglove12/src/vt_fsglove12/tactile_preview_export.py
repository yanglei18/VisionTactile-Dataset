"""Export a neutral licensed MANO model for a display-only subprocess consumer.

This module does not connect devices, load installation calibration, or import
Qt/ROS/serial. The caller supplies its trusted model and numerical environment.
Only one JSON document is written to stdout; backend diagnostics go to stderr.
"""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
import json
from pathlib import Path
import sys


def export_preview(mano_assets, hand="right"):
    """Generate a static reference hand, not a measured or personalized pose."""
    with redirect_stdout(sys.stderr):
        import numpy as np

        from .guided_calibration import model_motion_axes
        from .mano import ManoBackend

        backend = ManoBackend(mano_assets, hand=hand)
        neutral = backend.render(np.zeros((16, 3)))
        axes = model_motion_axes(neutral["joints_m"], hand=hand)
    # Do not transmit model paths, package provenance, calibration or poses.
    geometry = {name: neutral[name] for name in ("vertices_m", "faces", "joints_m")}
    geometry["provenance"] = {"hand": hand}
    return {"schema_version": 1, "neutral_geometry": geometry, "model_axes": axes}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mano-assets", required=True)
    parser.add_argument("--hand", required=True, choices=("right", "left"))
    parser.add_argument("--output", type=Path,
                        help="Write a local static JSON model instead of stdout; never overwrite an existing file")
    args = parser.parse_args(argv)
    try:
        value = export_preview(args.mano_assets, args.hand)
        document = json.dumps(value, allow_nan=False, separators=(",", ":"))
        if args.output is not None:
            # Explicit local export for an entitled deployment. Conversion
            # does not grant redistribution rights to the source model.
            with args.output.open("x", encoding="utf-8", newline="\n") as stream:
                stream.write(document + "\n")
            return 0
    except Exception as exc:
        print(f"MANO 静态模型加载失败：{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    sys.stdout.write(document + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
