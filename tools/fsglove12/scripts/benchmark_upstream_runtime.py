#!/usr/bin/env python3
"""Bounded, read-only official FSGlove/MANO compute benchmark; no GUI or device.

Example (the package and its real model dependencies must be installed)::

    python tools/fsglove12/scripts/benchmark_upstream_runtime.py \
      --session SESSION --fsglove-source CHECKOUT --mano-assets ASSETS \
      --frames 200 --warmup 50 --mesh-every 4 --output NEW_RESULT.json

The latest numbered profile is selected unless --profile is supplied. The
default requires twelve recorded channels and enabled five-finger functional
calibration. Loading, warmup and the optional separate cProfile pass are not
timed. Original timestamps, sample identities, epochs and calibration are never
relabeled. --snapshot-dir freezes only the bounded input window into a NEW
directory, making repeated before/after runs use identical inputs.

This measures CPU compute capacity, NOT 120 Hz acquisition, sensor-to-display
latency, real-time scheduling, accuracy or a hard deadline guarantee. Recorded
orientations commonly have only a 30 Hz publication cadence even when their
underlying sensor stream was received at 120 Hz.
"""
from __future__ import annotations

import argparse
import cProfile
from copy import deepcopy
import gc
import hashlib
import importlib.metadata
import io
import json
import math
import os
from pathlib import Path
import platform
import pstats
import re
import statistics
import sys
import time


THREAD_VARIABLES = ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                    "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "BLIS_NUM_THREADS")
FINGERS = ("index", "middle", "ring", "little", "thumb")


def _json(path):
    with Path(path).open(encoding="utf-8") as stream:
        return json.load(stream)


def complete_tail(path, *, count, max_bytes=16 * 1024 * 1024):
    """Read at most max_bytes from one EOF snapshot; ignore incomplete append."""
    if type(count) is not int or count < 1 or type(max_bytes) is not int or max_bytes < 1:
        raise ValueError("count and max_bytes must be positive integers")
    with Path(path).open("rb") as stream:
        end = stream.seek(0, 2)
        start = max(0, end - max_bytes)
        stream.seek(start)
        lines = stream.read(end - start).splitlines(keepends=True)
    if start:
        lines = lines[1:]  # A leading partial line cannot be trusted.
    complete = [line for line in lines if line.endswith(b"\n") and line.strip()]
    if len(complete) < count:
        raise ValueError(f"Need {count} complete frames; bounded tail contains {len(complete)}. "
                         "Increase --tail-mib explicitly; no full-file scan is performed.")
    frames = [json.loads(line) for line in complete[-count:]]
    return frames, dict(file_size_at_read_bytes=end, read_bytes=end - start,
                        max_read_bytes=max_bytes, complete_lines_in_tail=len(complete))


def bounded_frame_range(path, *, start_frame, count, max_bytes=16 * 1024 * 1024):
    """Binary-seek a recorded contiguous frame range, under a total read budget.

    Recorded session frame_index must be monotonically increasing. This allows
    recovering a previously measured window from a still-growing multi-GB file
    without reading/scanning the prefix or retaining unrelated records.
    """
    if min(start_frame, count - 1, max_bytes - 1) < 0:
        raise ValueError("Need nonnegative start_frame and positive count/read budget")
    read_bytes = 0
    with Path(path).open("rb") as stream:
        end = stream.seek(0, 2)

        def read_line():
            nonlocal read_bytes
            allowance = min(1024 * 1024, end - stream.tell(), max_bytes - read_bytes)
            if allowance <= 0:
                raise ValueError("Requested frame range exceeds bounded read budget or snapshot EOF")
            line = stream.readline(allowance)
            read_bytes += len(line)
            return line

        low, high, candidate = 0, end, None
        while low < high:
            middle = (low + high) // 2
            stream.seek(max(0, middle - 1))
            if middle:
                if read_bytes >= max_bytes:
                    raise ValueError("Binary search exceeds bounded read budget")
                previous = stream.read(1)
                read_bytes += 1
                if previous != b"\n":
                    read_line()
            position = stream.tell()
            if position >= end:
                high = middle
                continue
            line = read_line()
            if not line.endswith(b"\n"):
                high = middle
                continue
            frame = json.loads(line)
            if frame["frame_index"] >= start_frame:
                candidate = position
                high = middle
            else:
                low = stream.tell()
        if candidate is None:
            raise ValueError("Requested start frame is unavailable")
        stream.seek(candidate)
        frames = []
        for expected in range(start_frame, start_frame + count):
            line = read_line()
            if not line.endswith(b"\n"):
                raise ValueError("Requested frame range ends in an incomplete record")
            frame = json.loads(line)
            if frame.get("frame_index") != expected:
                raise ValueError("Requested complete contiguous recorded frame range is unavailable")
            frames.append(frame)
    return frames, dict(file_size_at_read_bytes=end, read_bytes=read_bytes,
                        max_read_bytes=max_bytes, requested_start_frame=start_frame,
                        lookup="bounded_binary_seek_by_monotone_frame_index")


def select_profile(session, explicit=None):
    session = Path(session)
    if explicit is not None:
        selected = Path(explicit)
        return selected if selected.is_absolute() else session / selected
    versions = []
    for path in session.glob("profile.v*.json"):
        match = re.fullmatch(r"profile\.v(\d+)\.json", path.name)
        if match:
            versions.append((int(match[1]), path))
    return max(versions)[1] if versions else session / "profile.json"


def load_recording(session, *, count, profile_path=None, max_bytes=16 * 1024 * 1024, start_frame=None):
    from vt_fsglove12.calibration import validate_rest
    from vt_fsglove12.config import digest, validate_config

    session = Path(session).resolve()
    config = validate_config(_json(session / "config.json"))
    rest = _json(session / "rest.json")
    validate_rest(rest)
    selected = select_profile(session, profile_path)
    profile, metadata = _json(selected), _json(session / "session_metadata.json")
    config_hash, rest_hash, identity = digest(config), digest(rest), metadata["session_id"]
    if (profile.get("session_id") != identity or profile.get("config_sha256") != config_hash
            or profile.get("rest_sha256") != rest_hash
            or metadata.get("config_sha256") != config_hash
            or metadata.get("model_rest_sha256") != rest_hash):
        raise ValueError("Recorded profile/config/rest/session bindings do not match")
    if (profile.get("functional_enabled") is not True
            or set(profile.get("functional_fingers", {})) != set(FINGERS)):
        raise ValueError("Benchmark requires enabled five-finger functional calibration")
    path = session / "orientations.jsonl"
    frames, tail = (complete_tail(path, count=count, max_bytes=max_bytes) if start_frame is None else
                    bounded_frame_range(path, start_frame=start_frame, count=count, max_bytes=max_bytes))
    previous = -math.inf
    for frame in frames:
        if (frame.get("session_id") != identity or frame.get("config_sha256") != config_hash
                or frame.get("model_rest_sha256") != rest_hash
                or frame.get("orientation_source") != "vqf_6d"
                or frame.get("orientation_frame") != "independent_initial_heading"):
            raise ValueError("Recorded orientation identity does not match the profile")
        stamp = frame.get("timestamp_s")
        if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                or not math.isfinite(stamp) or stamp < previous):
            raise ValueError("Recorded timestamps are invalid or decreasing")
        previous = stamp
        if set(frame.get("nodes", {})) != {str(node) for node in range(1, 13)}:
            raise ValueError("Benchmark requires all twelve recorded input channels")
        for node, params in profile["nodes"].items():
            if frame["nodes"][node].get("epoch") != params["epoch"]:
                raise ValueError(f"Recorded node {node} does not match the calibration epoch")
        for node, data in frame["nodes"].items():
            if (type(data.get("epoch")) is not int or data["epoch"] < 0
                    or type(data.get("sample_index")) is not int or data["sample_index"] <= 0):
                raise ValueError(f"Node {node} has invalid source sample identity")
    return dict(session=session, config=config, rest=rest, profile=profile,
                metadata=metadata, frames=frames, profile_path=selected, tail=tail)


def percentile(values, percent):
    """Linear interpolation, equivalent to NumPy's default percentile method."""
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.
    low, high = math.floor(position), math.ceil(position)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def latency_stats(values, *, deadline_ms):
    if not values:
        return dict(count=0, median_ms=None, p95_ms=None, p99_ms=None, max_ms=None,
                    deadline_ms=deadline_ms, deadline_misses=0, deadline_miss_fraction=None)
    misses = sum(value > deadline_ms for value in values)
    return dict(count=len(values), median_ms=statistics.median(values),
                p95_ms=percentile(values, 95), p99_ms=percentile(values, 99),
                max_ms=max(values), deadline_ms=deadline_ms,
                deadline_misses=misses, deadline_miss_fraction=misses / len(values))


def source_summary(frames, config):
    nodes = sorted(frames[0]["nodes"], key=int)
    signatures = [tuple((node, frame["nodes"][node]["epoch"],
                         frame["nodes"][node]["sample_index"]) for node in nodes) for frame in frames]
    stamps = [frame["timestamp_s"] for frame in frames]
    positive_intervals = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    span = stamps[-1] - stamps[0]
    root = next(node for node, slot in config["node_to_slot"].items() if slot == 0)
    by_node = {node: len({(frame["nodes"][node]["epoch"], frame["nodes"][node]["sample_index"])
                         for frame in frames}) for node in nodes}
    encoded = json.dumps(frames, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return dict(frame_count=len(frames), first_frame_index=frames[0].get("frame_index"),
                last_frame_index=frames[-1].get("frame_index"),
                unique_recorded_frame_indices=len({frame.get("frame_index") for frame in frames}),
                unique_source_vectors=len(set(signatures)),
                duplicate_source_vectors=len(frames) - len(set(signatures)),
                unique_root_samples=by_node[root], unique_samples_by_node=by_node,
                input_channel_count=len(nodes), mapped_node_count=len(config["node_to_slot"]),
                recorded_span_s=span, recorded_publication_hz=(len(frames) - 1) / span if span > 0 else None,
                median_recorded_interval_ms=1000. * statistics.median(positive_intervals)
                    if positive_intervals else None,
                configured_recording_output_hz=config["output_rate_hz"],
                configured_sensor_sample_hz=config["sample_rate_hz"],
                input_window_sha256=hashlib.sha256(encoded).hexdigest(),
                timestamps_rebased=False, recorded_values_reused=False,
                is_120hz_hardware_validation=False)


def freeze_recording(recording, destination):
    """Create a small standalone input fixture, never overwrite a session."""
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    for name, key in (("config.json", "config"), ("rest.json", "rest"),
                      ("profile.json", "profile"), ("session_metadata.json", "metadata")):
        with (destination / name).open("x", encoding="utf-8") as stream:
            json.dump(recording[key], stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write("\n")
    with (destination / "orientations.jsonl").open("x", encoding="utf-8") as stream:
        for frame in recording["frames"]:
            stream.write(json.dumps(frame, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")
    with (destination / "benchmark_snapshot.json").open("x", encoding="utf-8") as stream:
        json.dump(dict(source_session=str(recording["session"]),
                       source_profile=str(recording["profile_path"]),
                       source_tail=recording["tail"], hardware_accessed_by_snapshot=False,
                       note="Recorded provenance retained verbatim; this copy never accessed hardware."), stream, indent=2)


class GcPauseRecorder:
    """Observe automatic GC callbacks without collecting, disabling or tuning GC."""

    def __init__(self, clock=time.perf_counter, max_events=4096):
        self.clock, self.max_events = clock, max_events
        self.frame_index, self.stage = None, "bookkeeping"
        self.events, self._starts = [], {}
        self.origin, self.dropped_events = None, 0
        self.enabled_at_start, self.thresholds, self.counts_at_start = None, None, None

    def _callback(self, phase, info):
        generation, now = info["generation"], self.clock()
        if phase == "start":
            self._starts[generation] = (now, self.frame_index, self.stage)
        elif phase == "stop" and generation in self._starts:
            start, frame, stage = self._starts.pop(generation)
            if len(self.events) < self.max_events:
                self.events.append(dict(generation=generation, duration_ms=(now - start) * 1000.,
                    start_elapsed_ms=(start - self.origin) * 1000., frame_index=frame, stage=stage,
                    collected=info.get("collected"), uncollectable=info.get("uncollectable")))
            else:
                self.dropped_events += 1

    def start(self):
        self.enabled_at_start, self.thresholds = gc.isenabled(), gc.get_threshold()
        self.counts_at_start, self.origin = gc.get_count(), self.clock()
        gc.callbacks.append(self._callback)

    def finish(self):
        gc.callbacks.remove(self._callback)
        generations = {}
        for generation in range(3):
            durations = [event["duration_ms"] for event in self.events if event["generation"] == generation]
            generations[str(generation)] = dict(count=len(durations), total_ms=sum(durations),
                                                max_ms=max(durations, default=0.))
        return dict(enabled=True, automatic_gc_enabled_before=self.enabled_at_start,
                    automatic_gc_enabled_after=gc.isenabled(), gc_policy_changed=False,
                    manual_collect_called=False, thresholds=self.thresholds,
                    counts_before=self.counts_at_start, counts_after=gc.get_count(),
                    by_generation=generations, events=self.events, dropped_events=self.dropped_events,
                    instrumentation="gc.callbacks only; no outliers removed or latency values adjusted",
                    limitations="Tracing adds callback overhead and may shift automatic collection timing; "
                                "non-overlapping delays are not automatically attributed to OS jitter.")


def benchmark_frames(backend, geometry, display, frames, profile, *, warmup, mesh_every, deadline_ms,
                     trace_gc=False):
    """Run ordered distinct records with optional lower-frequency mesh work."""
    if warmup < 0 or warmup >= len(frames) or mesh_every < 1:
        raise ValueError("Need measured frames and a positive mesh interval")
    context = (profile["session_id"], profile.get("profile_version"), profile["upstream_state_sha256"])
    gc_trace = None

    def run_one(frame, render):
        if gc_trace is not None:
            gc_trace.frame_index, gc_trace.stage = frame.get("frame_index"), "solve"
        start = time.perf_counter()
        pose = backend.solve(frame, profile)
        solved = time.perf_counter()
        if gc_trace is not None:
            gc_trace.stage = "display"
        if not pose.get("pose_available") or not pose.get("solver_success"):
            raise ValueError(f"No valid benchmark pose at frame {frame.get('frame_index')}: {pose.get('reason')}")
        if not pose.get("upstream_fsglove_solver_executed"):
            raise ValueError("Benchmark did not execute the official solver")
        if (pose.get("functional_adapter", {}).get("enabled") is not True
                or pose.get("pose_postprocessing") != "qmt_functional_axis_and_joint_constraint"):
            raise ValueError("Benchmark did not execute the functional pose pipeline")
        visual = display.update(pose, context=context, now=frame["timestamp_s"])
        displayed = time.perf_counter()
        if gc_trace is not None:
            gc_trace.stage = "mesh"
        if render:
            visual = visual if visual is not None else pose
            geometry.render_mano(visual["mano_pose_rotvec"], visual.get("shape_params"))
        finished = time.perf_counter()
        if gc_trace is not None:
            gc_trace.stage = "bookkeeping"
        return ((solved - start) * 1000., (displayed - solved) * 1000.,
                (finished - displayed) * 1000. if render else None, (finished - start) * 1000.)

    # Always initialize MANO even if the measured cadence skips most meshes.
    # The priming solve is reset before chronological warmup so no source-time
    # reversal or duplicate sample changes measured estimator state.
    run_one(frames[0], True)
    backend.reset_calibration()
    display.reset()
    for index, frame in enumerate(frames[:warmup]):
        run_one(frame, index % mesh_every == 0)
    # With no requested warmup the first measured solve includes lazy profile
    # restoration; require at least one chronological warmup in the CLI.
    values = {key: [] for key in ("solve", "display", "mesh", "total", "total_with_mesh", "total_without_mesh")}
    if trace_gc:
        gc_trace = GcPauseRecorder()
        gc_trace.start()
    started = time.perf_counter()
    try:
        for index, frame in enumerate(frames[warmup:]):
            render = index % mesh_every == 0
            solve, visual, mesh, total = run_one(frame, render)
            values["solve"].append(solve)
            values["display"].append(visual)
            values["total"].append(total)
            values["total_with_mesh" if render else "total_without_mesh"].append(total)
            if mesh is not None:
                values["mesh"].append(mesh)
        elapsed = time.perf_counter() - started
    finally:
        gc_diagnostic = gc_trace.finish() if gc_trace is not None else None
    result = dict(latency_ms={key: latency_stats(value, deadline_ms=deadline_ms) for key, value in values.items()},
                measured_wall_s=elapsed, measured_frames=len(frames) - warmup,
                processing_throughput_fps=(len(frames) - warmup) / elapsed,
                rendered_mesh_count=len(values["mesh"]), mesh_every=mesh_every,
                warmup_frames=warmup, initialization_excluded=True, profiling_excluded=True)
    if gc_diagnostic is not None:
        slowest = sorted(range(len(values["total"])), key=lambda index: values["total"][index], reverse=True)[:10]
        gc_diagnostic["slowest_frames"] = [dict(frame_index=frames[warmup + index].get("frame_index"),
            solve_ms=values["solve"][index], display_ms=values["display"][index],
            mesh_ms=values["mesh"][index // mesh_every] if index % mesh_every == 0 else None,
            total_ms=values["total"][index],
            gc_events=[event for event in gc_diagnostic["events"]
                       if event["frame_index"] == frames[warmup + index].get("frame_index")]) for index in slowest]
        result["gc_diagnostic"] = gc_diagnostic
    return result


def profile_frames(backend, geometry, display, frames, profile, *, warmup, count, mesh_every):
    """Independent second replay; profiling overhead never enters latency stats."""
    backend.reset_calibration()
    display.reset()
    context = (profile["session_id"], "separate-cprofile-pass")

    def one(frame, render):
        pose = backend.solve(frame, profile)
        visual = display.update(pose, context=context, now=frame["timestamp_s"])
        if render:
            visual = visual if visual is not None else pose
            geometry.render_mano(visual["mano_pose_rotvec"], visual.get("shape_params"))

    for index, frame in enumerate(frames[:warmup]):
        one(frame, index % mesh_every == 0)
    profiler = cProfile.Profile()
    profiler.enable()
    for index, frame in enumerate(frames[warmup:warmup + count]):
        one(frame, index % mesh_every == 0)
    profiler.disable()
    text = io.StringIO()
    pstats.Stats(profiler, stream=text).strip_dirs().sort_stats("cumulative").print_stats(45)
    return dict(frame_count=count, pass_is_separate_from_latency=True, cumulative_top45=text.getvalue())


def implementation_provenance():
    """Hash small implementation files only, never full session data or assets."""
    import vt_fsglove12
    package = Path(vt_fsglove12.__file__).parent
    names = ("upstream_backend", "upstream_inference", "upstream_priors", "functional_pose",
             "upstream_functional", "hinge_estimator", "heading_window", "heading_correction", "mano", "display_pose")
    sources = {name: hashlib.sha256((package / f"{name}.py").read_bytes()).hexdigest()
               for name in names if (package / f"{name}.py").is_file()}
    versions = {}
    for package_name in ("numpy", "scipy", "torch", "qmt", "manotorch"):
        try:
            versions[package_name] = importlib.metadata.version(package_name)
        except importlib.metadata.PackageNotFoundError:
            versions[package_name] = "unknown"
    return dict(source_sha256=sources, installed_package_versions=versions,
                benchmark_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--profile", help="Relative to session or absolute; default latest numbered profile")
    parser.add_argument("--fsglove-source", required=True, type=Path)
    parser.add_argument("--mano-assets", required=True, type=Path)
    parser.add_argument("--frames", type=int, default=200, help="Measured frames, excluding warmup")
    parser.add_argument("--warmup", type=int, default=50)
    parser.add_argument("--tail-mib", type=int, default=16)
    parser.add_argument("--start-frame", type=int, help="Bounded binary-seek a known frame range instead of tail")
    parser.add_argument("--mesh-every", type=int, default=1, help="Render one real MANO mesh per N solves")
    parser.add_argument("--target-hz", type=float, default=120., help="Compute budget only, not input rate")
    parser.add_argument("--cprofile-frames", type=int, default=0, help="Additional separately profiled replay")
    parser.add_argument("--trace-gc", action="store_true", help="Observe GC pauses without changing GC policy")
    parser.add_argument("--snapshot-dir", type=Path, help="NEW directory for the small exact replay window")
    parser.add_argument("--output", type=Path, help="NEW JSON report; otherwise print report to stdout")
    args = parser.parse_args(argv)
    if (args.frames < 2 or args.warmup < 1 or args.tail_mib < 1 or args.mesh_every < 1
            or not math.isfinite(args.target_hz) or args.target_hz <= 0
            or not 0 <= args.cprofile_frames <= args.frames
            or (args.start_frame is not None and args.start_frame < 0)):
        parser.error("Need frames>=2, warmup>=1, positive tail/mesh/rate, and cprofile-frames in [0, frames]")
    for path in (args.output, args.snapshot_dir):
        if path is not None and path.exists():
            parser.error(f"Refusing to overwrite existing path: {path}")
    for variable in THREAD_VARIABLES:
        os.environ[variable] = "1"  # Before importing NumPy/Scipy/Torch.
    import torch
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    from vt_fsglove12.display_pose import DisplayPoseCache
    from vt_fsglove12.mano import ManoBackend
    from vt_fsglove12.upstream_backend import OfficialFSGloveBackend

    recording = load_recording(args.session, count=args.frames + args.warmup,
                               profile_path=args.profile, max_bytes=args.tail_mib * 1024 * 1024,
                               start_frame=args.start_frame)
    if args.snapshot_dir:
        freeze_recording(recording, args.snapshot_dir)
    profile, frames = recording["profile"], recording["frames"]
    profile_before = deepcopy(profile)
    backend = OfficialFSGloveBackend(recording["config"], recording["rest"],
                                    checkout=args.fsglove_source, mano_assets_root=args.mano_assets)
    geometry, display = ManoBackend(args.mano_assets), DisplayPoseCache()
    result = benchmark_frames(backend, geometry, display, frames, profile,
                              warmup=args.warmup, mesh_every=args.mesh_every,
                              deadline_ms=1000. / args.target_hz, trace_gc=args.trace_gc)
    result.update(schema_version=1, benchmark="official_fsglove_recorded_compute_capacity",
        input_kind="recorded_orientation_offline_benchmark", hardware_accessed=False,
        input=source_summary(frames[args.warmup:], recording["config"]),
        complete_window=source_summary(frames, recording["config"]),
        source_session=str(recording["session"]), source_profile=str(recording["profile_path"]),
        bounded_read=recording["tail"], target_compute_hz=args.target_hz,
        mesh_budget_hz=args.target_hz / args.mesh_every,
        calibrated_node_count=len(profile["nodes"]), functional_enabled=profile["functional_enabled"],
        functional_fingers=list(profile["functional_fingers"]), profile_version=profile.get("profile_version"),
        upstream=backend.core.provenance, implementation=implementation_provenance(),
        environment=dict(python=sys.version.split()[0], platform=platform.platform(),
                         cpu=platform.processor(), threads={key: os.environ[key] for key in THREAD_VARIABLES},
                         torch_threads=torch.get_num_threads(), torch_interop_threads=torch.get_num_interop_threads()),
        exclusions=["model/profile loading", "warmup", "serial acquisition", "VQF fusion",
                    "disk/JSON recording", "IPC and snapshot copies", "GUI/OpenGL", "OS scheduler deadlines"],
        limitations=["Replay publication cadence is reported separately from compute throughput.",
                     "No new sensor observations or timestamps are synthesized to claim 120 Hz.",
                     "A 120 Hz compute budget is not a 120 Hz hardware or hard-real-time guarantee.",
                     "Geometry decimation is a compute scenario, not a measured display refresh rate."])
    if args.cprofile_frames:
        result["cprofile"] = profile_frames(backend, geometry, display, frames, profile,
            warmup=args.warmup, count=args.cprofile_frames, mesh_every=args.mesh_every)
    if profile != profile_before:
        raise RuntimeError("Benchmark backend mutated the supplied profile")
    result["profile_unchanged"] = True
    rendered = json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + "\n"
    if args.output:
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(rendered)
        print(json.dumps(dict(output=str(args.output), frames=result["measured_frames"],
            throughput_fps=result["processing_throughput_fps"], solve=result["latency_ms"]["solve"],
            total=result["latency_ms"]["total"], recorded_hz=result["input"]["recorded_publication_hz"],
            hardware_accessed=False), ensure_ascii=False))
    else:
        print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
