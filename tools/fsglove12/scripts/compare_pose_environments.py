#!/usr/bin/env python3
"""Compare isolated SciPy environments on unchanged recorded FSGlove inputs.

No installed dependency, input, calibration file, device or GUI is changed.
The candidate directory is added only to one child's PYTHONPATH. Both children
run the same official solver, functional pipeline and real MANO for every input
frame; they also refit the official three-pose calibration with the saved pose
means and seed zero. These reconstructed captures test numerical equivalence,
not new physical calibration evidence. Results are saved to NEW paths only.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys


THREAD_VARIABLES = ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                    "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "BLIS_NUM_THREADS")
NUMERIC_FIELDS = ("mano_pose_rotvec", "anatomical_euler_rad", "root_quat_xyzw", "joints_m")
_OMIT = object()


def semantic_status(value):
    """Retain every qualitative field, including nested quality/continuity flags.

    Floating diagnostics/angles are evaluated separately, never mistaken for
    exact state labels. Integer slot indices/counters, strings, booleans and
    explicit nulls remain exact; wholly numeric floating arrays are omitted.
    """
    if isinstance(value, float):
        return _OMIT
    if isinstance(value, dict):
        selected = {key: transformed for key, child in value.items()
                    if (transformed := semantic_status(child)) is not _OMIT}
        return selected if selected or not value else _OMIT
    if isinstance(value, list):
        selected = [transformed for child in value if (transformed := semantic_status(child)) is not _OMIT]
        return selected if selected or not value else _OMIT
    return value


def calibration_captures(profile):
    """Three unique index wrappers around each saved calibration pose mean."""
    captures = {}
    for step_index, (step, values) in enumerate(profile["calibration_inputs"].items()):
        captures[step] = [dict(session_id=profile["session_id"],
            timestamp_s=float(step_index * 10 + index), nodes={node: dict(
                quat_xyzw=deepcopy(values[binding["slot"]]), age_s=0.,
                epoch=binding["epoch"], sample_index=step_index * 10 + index + 1)
                for node, binding in profile["nodes"].items()}) for index in range(3)]
    return captures


def environment_for_child(candidate=None):
    environment = os.environ.copy()
    environment.update({key: "1" for key in THREAD_VARIABLES})
    if candidate is not None:
        current = environment.get("PYTHONPATH")
        environment["PYTHONPATH"] = str(Path(candidate).resolve()) + (os.pathsep + current if current else "")
    return environment


def _write_new(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write("\n")


def worker(args):
    # Numerical libraries are imported only after the parent sets thread limits
    # and (candidate only) PYTHONPATH, in a fresh interpreter.
    import numpy as np
    import scipy
    import torch
    from benchmark_upstream_runtime import implementation_provenance, load_recording, source_summary
    from vt_fsglove12.mano import ManoBackend
    from vt_fsglove12.upstream_backend import OfficialFSGloveBackend

    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    recording = load_recording(args.session, count=args.frames)
    profile, frames = recording["profile"], recording["frames"]
    original = deepcopy(profile)
    frame_hash = source_summary(frames, recording["config"])["input_window_sha256"]
    backend = OfficialFSGloveBackend(recording["config"], recording["rest"],
                                    checkout=args.fsglove_source, mano_assets_root=args.mano_assets)
    geometry = ManoBackend(args.mano_assets, compact_live_geometry=True)
    outputs = []
    for frame in frames:
        pose = backend.solve(frame, profile)
        if (not pose.get("pose_available") or not pose.get("solver_success")
                or pose.get("pose_postprocessing") != "qmt_functional_axis_and_joint_constraint"):
            raise ValueError("Every comparison frame must execute official + functional inference")
        mesh = geometry.render_mano(pose["mano_pose_rotvec"], pose["shape_params"])
        outputs.append(dict(frame_index=frame["frame_index"],
            **{key: pose[key] for key in NUMERIC_FIELDS if key != "joints_m"},
            joints_m=mesh["joints_m"], status=semantic_status(pose)))
    captures = calibration_captures(profile)
    fitted = backend.fit_calibration(captures, session_id=profile["session_id"])
    if profile != original:
        raise RuntimeError("Supplied profile was mutated")
    final_hash = source_summary(frames, recording["config"])["input_window_sha256"]
    if final_hash != frame_hash:
        raise RuntimeError("Recorded frames were mutated")
    _write_new(args.worker_output, dict(scipy_version=scipy.__version__, scipy_file=scipy.__file__,
        numpy_version=np.__version__, python=sys.version, executable=sys.executable,
        input_window_sha256=frame_hash, frame_count=len(frames), profile_unchanged=True,
        frames_unchanged=True, input=source_summary(frames, recording["config"]),
        implementation=implementation_provenance(), outputs=outputs,
        calibration=dict(seed=fitted["calibration_seed"], state=fitted["upstream_state"],
            calibration_inputs=fitted["calibration_inputs"], measured_slots=fitted["measured_calibration_slots"],
            virtual_slots=fitted["virtual_calibration_slots"], nodes=fitted["nodes"],
            rejected_nodes=fitted["rejected_nodes"], method=fitted["method"],
            state_sha256=fitted["upstream_state_sha256"],
            capture_kind="reconstructed_from_saved_three_pose_means_not_new_hardware_capture")))


def numerical_difference(left, right, *, tolerance):
    import numpy as np
    left, right = np.asarray(left, dtype=float), np.asarray(right, dtype=float)
    if left.shape != right.shape or not np.isfinite(left).all() or not np.isfinite(right).all():
        raise ValueError("Comparison arrays must be finite and have identical shapes")
    difference = right - left
    return dict(shape=list(left.shape), max_absolute=float(np.max(np.abs(difference), initial=0.)),
                rms=float(np.sqrt(np.mean(difference ** 2))) if difference.size else 0.,
                atol=tolerance, within_tolerance=bool(np.all(np.abs(difference) <= tolerance)),
                exactly_equal=bool(np.array_equal(left, right)))


def status_differences(left, right, path=""):
    if isinstance(left, dict) and isinstance(right, dict):
        differences = []
        for key in sorted(set(left) | set(right)):
            child = f"{path}.{key}" if path else key
            if key not in left or key not in right:
                differences.append(dict(path=child, baseline=left.get(key), candidate=right.get(key)))
            else:
                differences.extend(status_differences(left[key], right[key], child))
        return differences
    if type(left) is not type(right) or left != right:
        return [dict(path=path, baseline=left, candidate=right)]
    return []


def compare_results(baseline, candidate, *, angle_atol=1e-6, joint_atol=1e-7, calibration_atol=1e-10):
    import numpy as np
    from scipy.spatial.transform import Rotation

    if (baseline["input_window_sha256"] != candidate["input_window_sha256"]
            or baseline["frame_count"] != candidate["frame_count"]):
        raise ValueError("Children did not use the same exact input window")
    first, second = baseline["outputs"], candidate["outputs"]
    if [row["frame_index"] for row in first] != [row["frame_index"] for row in second]:
        raise ValueError("Children produced different recorded frame sequences")
    numerical = {key: numerical_difference([row[key] for row in first], [row[key] for row in second],
        tolerance=joint_atol if key == "joints_m" else angle_atol) for key in NUMERIC_FIELDS}
    roots = [Rotation.from_quat([row["root_quat_xyzw"] for row in rows]) for rows in (first, second)]
    root_error = (roots[0].inv() * roots[1]).magnitude()
    numerical["root_geodesic_rad"] = dict(max_absolute=float(root_error.max()),
        rms=float(np.sqrt(np.mean(root_error ** 2))), atol=angle_atol,
        within_tolerance=bool(np.all(root_error <= angle_atol)))
    joints = np.asarray([row["joints_m"] for row in second]) - np.asarray([row["joints_m"] for row in first])
    numerical["joint_euclidean_m"] = dict(max_absolute=float(np.linalg.norm(joints, axis=-1).max()),
                                         units="metres")
    differences, mismatch_frames = [], 0
    for left, right in zip(first, second):
        mismatch = status_differences(left["status"], right["status"])
        if mismatch:
            mismatch_frames += 1
            if len(differences) < 30:
                differences.extend(dict(frame_index=left["frame_index"], **entry) for entry in mismatch[:30 - len(differences)])
    calibration = {key: numerical_difference(baseline["calibration"]["state"][key],
                                             candidate["calibration"]["state"][key], tolerance=calibration_atol)
                   for key in baseline["calibration"]["state"]}
    calibration_semantics = status_differences(
        {key: value for key, value in baseline["calibration"].items() if key not in ("state", "calibration_inputs", "state_sha256")},
        {key: value for key, value in candidate["calibration"].items() if key not in ("state", "calibration_inputs", "state_sha256")})
    pose_passed = all(entry.get("within_tolerance", True) for entry in numerical.values())
    fit_passed = all(entry["within_tolerance"] for entry in calibration.values()) and not calibration_semantics
    return dict(schema_version=1, passed=pose_passed and fit_passed and not mismatch_frames,
        frame_count=len(first), input_window_sha256=baseline["input_window_sha256"],
        input=baseline["input"], numerical=numerical,
        qualitative_status=dict(exactly_equal=mismatch_frames == 0, mismatch_frames=mismatch_frames,
            first_differences=differences, coverage="all nested non-floating pose fields, including masks, "
            "validity/display flags, quality, source labels, rejection and continuity states"),
        calibration=dict(seed=baseline["calibration"]["seed"], numerical=calibration,
            status_differences=calibration_semantics,
            state_digest_equal=baseline["calibration"]["state_sha256"] == candidate["calibration"]["state_sha256"],
            capture_kind=baseline["calibration"]["capture_kind"]),
        baseline={key: value for key, value in baseline.items() if key not in ("outputs", "calibration", "input")},
        candidate={key: value for key, value in candidate.items() if key not in ("outputs", "calibration", "input")},
        hardware_accessed=False, installed_dependencies_changed=False,
        limitations=["Recorded-window numerical equivalence is not physical accuracy validation.",
            "Small floating differences do not authorize silently changing installed dependencies.",
            "This comparison does not measure real-time performance or 120 Hz acquisition."])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--fsglove-source", type=Path, required=True)
    parser.add_argument("--mano-assets", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=250)
    parser.add_argument("--candidate-path", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--worker-output", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.frames < 2:
        parser.error("Need at least two recorded frames")
    if args.worker_output is not None:
        worker(args)
        return 0
    if args.output is None or args.candidate_path is None:
        parser.error("Need --output and --candidate-path")
    if not (args.candidate_path / "scipy").is_dir():
        parser.error("Candidate path must contain the independently supplied scipy package")
    directory = args.output.with_suffix(".runs")
    if args.output.exists() or directory.exists():
        parser.error("Refusing to overwrite an existing comparison or child-result directory")
    directory.mkdir(parents=True)
    command = [sys.executable, str(Path(__file__).resolve()), "--session", str(args.session.resolve()),
               "--fsglove-source", str(args.fsglove_source.resolve()), "--mano-assets", str(args.mano_assets.resolve()),
               "--frames", str(args.frames)]
    results = []
    for label, sidecar in (("baseline", None), ("candidate", args.candidate_path)):
        output = directory / f"{label}.json"
        subprocess.run(command + ["--worker-output", str(output.resolve())],
                       env=environment_for_child(sidecar), check=True)
        with output.open(encoding="utf-8") as stream:
            results.append(json.load(stream))
    candidate_path = args.candidate_path.resolve()
    if (Path(results[0]["scipy_file"]).resolve().is_relative_to(candidate_path)
            or not Path(results[1]["scipy_file"]).resolve().is_relative_to(candidate_path)):
        raise ValueError("SciPy import paths did not isolate baseline and candidate environments")
    report = compare_results(*results)
    report["child_results_directory"] = str(directory.resolve())
    _write_new(args.output, report)
    print(json.dumps(dict(output=str(args.output), passed=report["passed"], frames=report["frame_count"],
                         numerical=report["numerical"], qualitative_status=report["qualitative_status"]), ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
