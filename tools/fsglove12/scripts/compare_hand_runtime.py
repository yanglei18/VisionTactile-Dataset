#!/usr/bin/env python3
"""Compare two hand-runtime source trees against the same archived inputs.

The parent uses only the standard library. Both workers run with one explicitly
chosen numerical Python and their own exact PYTHONPATH; no device is opened.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


POSE_FIELDS = (
    "pose_available", "solver_success", "reason", "mano_pose_rotvec",
    "root_quat_xyzw", "anatomical_euler_rad", "joint_valid_mask",
    "joint_display_available_mask", "source_labels", "joint_quality",
    "inferred_mask", "joint_observed_mask", "functional_calibration_coverage",
)


def digest_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def worker(spec_path):
    import numpy as np
    import vt_fsglove12
    from vt_fsglove12.mano import ManoBackend
    from vt_fsglove12.replay import replay_raw
    from vt_fsglove12.upstream_backend import OfficialFSGloveBackend

    spec = json.loads(Path(spec_path).read_text())
    output = Path(spec["output"])
    report = {"module": vt_fsglove12.__file__, "cases": {}}
    for case in spec["cases"]:
        label, session, upstream, assets = case
        session = Path(session)
        config = json.loads((session / "config.json").read_text())
        rest = json.loads((session / "rest.json").read_text())
        raw_paths = sorted(session.glob("input_scale_*/raw.csv"))
        if len(raw_paths) != 1:
            raise ValueError(f"{label}: exactly one archived input_scale raw.csv is required")
        raw = raw_paths[0]
        capture_paths = sorted((session / "calibration").glob("capture.*.json"))
        captures, frames = {}, []
        steps = {"neutral": "base_pose", "fingers_down": "z_rot_pose_1", "thumb_up": "z_rot_pose_2"}
        for path in capture_paths:
            document = json.loads(path.read_text())
            step = steps[document["step"]]
            if step in captures:
                raise ValueError(f"{label}: select one archived capture per step before comparison")
            captures[step] = document["frames"]
            frames.extend(document["frames"])
        if set(captures) != set(steps.values()) or not frames:
            raise ValueError(f"{label}: all three original mounting captures are required")
        frames.sort(key=lambda frame: frame["timestamp_s"])
        replay_path = output / f"{label}-fusion.jsonl"
        fusion = replay_raw(raw, config, replay_path, max_frames=spec["raw_frames"])
        backend = OfficialFSGloveBackend(config, rest, checkout=upstream, mano_assets_root=assets)
        backend.core.torch.set_num_threads(1)
        model = ManoBackend(assets, hand=config["hand"], compact_live_geometry=True)
        profile = backend.fit_calibration(captures, session_id=frames[0]["session_id"])
        available = 0
        pose_path = output / f"{label}-poses.jsonl"
        with pose_path.open("w") as stream:
            for frame in frames:
                pose = backend.solve(frame, profile, timestamp_s=frame["timestamp_s"])
                value = {field: pose.get(field) for field in POSE_FIELDS}
                if pose.get("pose_available"):
                    available += 1
                    mesh = model.render_mano(pose["mano_pose_rotvec"], shape_params=pose["shape_params"])
                    value["joints_m"] = np.asarray(mesh["joints_m"]).tolist()
                stream.write(json.dumps(value, sort_keys=True, allow_nan=False) + "\n")
        report["cases"][label] = dict(
            hand=config["hand"], frames=len(frames), available=available,
            fusion_frames=fusion["frames"], fusion_sha256=digest_file(replay_path),
            pose_sha256=digest_file(pose_path), raw_sha256=digest_file(raw),
            config_sha256=digest_file(session / "config.json"),
            rest_sha256=digest_file(session / "rest.json"),
            capture_sha256=sorted(digest_file(path) for path in capture_paths))
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-source", type=Path, help="Directory containing baseline vt_fsglove12 package")
    parser.add_argument("--candidate-source", type=Path, help="Directory containing candidate vt_fsglove12 package")
    parser.add_argument("--python", type=Path, help="Python with the explicit trusted numerical/model dependencies")
    parser.add_argument("--case", action="append", nargs=4, metavar=("LABEL", "SESSION", "UPSTREAM", "MANO"))
    parser.add_argument("--output", type=Path, help="New output directory; never replaces a previous report")
    parser.add_argument("--raw-frames", type=int, default=1000)
    parser.add_argument("--worker", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker is not None:
        worker(args.worker)
        return 0
    if not all([args.baseline_source, args.candidate_source, args.python, args.case, args.output]):
        parser.error("Both sources, --python, at least one --case, and --output are required")
    if not 1 <= args.raw_frames <= 100000:
        parser.error("--raw-frames must be in [1, 100000]")
    labels = [case[0] for case in args.case]
    if len(set(labels)) != len(labels) or any(not label or not all(c.isalnum() or c in "_-" for c in label) for label in labels):
        parser.error("Case labels must be unique alphanumeric names with optional _ or -")
    cases = [[label, *[str(Path(path).expanduser().resolve(strict=True)) for path in paths]]
             for label, *paths in args.case]
    # Preserve the venv executable path: resolving its symlink would bypass it.
    python = str(args.python.expanduser().absolute())
    sources = [path.expanduser().resolve(strict=True) for path in [args.baseline_source, args.candidate_source]]
    if any(not (source / "vt_fsglove12" / "__init__.py").is_file() for source in sources):
        parser.error("Source arguments must contain the vt_fsglove12 package")
    output = args.output.expanduser().absolute()
    output.mkdir(parents=True, exist_ok=False)
    reports = {}
    for name, source in zip(("baseline", "candidate"), sources):
        destination = output / name
        destination.mkdir()
        spec = dict(cases=cases, output=str(destination), raw_frames=args.raw_frames)
        spec_path = destination / "inputs.json"
        spec_path.write_text(json.dumps(spec, indent=2) + "\n")
        environment = {**os.environ, "PYTHONPATH": str(source), "PYTHONNOUSERSITE": "1",
                       "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1"}
        with (destination / "worker.log").open("w") as log:
            subprocess.run([python, str(Path(__file__).resolve()), "--worker", str(spec_path)],
                           env=environment, stdout=log, stderr=subprocess.STDOUT, check=True)
        reports[name] = json.loads((destination / "report.json").read_text())
        actual_module = Path(reports[name]["module"]).resolve()
        if not actual_module.is_relative_to(source):
            raise RuntimeError(f"{name} imported a package outside its selected source tree")
    identical = reports["baseline"]["cases"] == reports["candidate"]["cases"]
    # Publishable comparison contains input content identity, no host paths.
    comparison = dict(schema_version=1, method="Refit original mounting captures and solve every captured frame; replay fixed raw prefix",
        identical=identical, raw_frame_limit=args.raw_frames, baseline=reports["baseline"]["cases"],
        candidate=reports["candidate"]["cases"])
    (output / "comparison.json").write_text(json.dumps(comparison, indent=2) + "\n")
    print(json.dumps(comparison, indent=2))
    return 0 if identical else 1


if __name__ == "__main__":
    raise SystemExit(main())
