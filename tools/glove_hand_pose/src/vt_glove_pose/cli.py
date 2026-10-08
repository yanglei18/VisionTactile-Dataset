"""Explicit entry points: no live port is opened merely by launching the GUI."""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

from .config import gyro_scale, load_config, write_json_exclusive
from .stream import iter_csv


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("Expected a finite positive number")
    return result


def inspect_csv(path, config):
    counts, first, last, sums, squares, norm_acc, near_limit = {}, {}, {}, {}, {}, {}, {}
    total = 0
    for sample in iter_csv(path):
        n, t = sample.node_id, sample.host_monotonic_s
        v = np.asarray(sample.values, dtype=float)
        if n not in counts:
            counts[n], first[n] = 0, t
            sums[n], squares[n], norm_acc[n], near_limit[n] = np.zeros(6), np.zeros(6), 0., 0
        counts[n] += 1
        last[n] = t
        sums[n] += v
        squares[n] += v*v
        norm_acc[n] += float(np.linalg.norm(v[3:]) * config["accel_g_per_lsb"])
        near_limit[n] += int(np.any(np.abs(v[:3]) >= 32700))
        total += 1
    if not total:
        raise ValueError("CSV contains no samples")
    nodes = {}
    warnings = ["Host parse timestamps are not hardware sample timestamps or proof of loss-free acquisition.",
                "Per-node receive rates do not establish synchronization between the twelve IMUs."]
    for n in range(1, 13):
        count = counts.get(n, 0)
        if not count:
            nodes[str(n)] = {"records": 0, "received_rate_hz": None}
            warnings.append(f"Node {n} is missing")
            continue
        span = last[n] - first[n]
        mean = sums[n] / count
        rate = (count - 1) / span if span > 0 else None
        nodes[str(n)] = dict(records=count, host_duration_s=span, received_rate_hz=rate,
                            raw_mean=mean.tolist(),
                            raw_std=np.sqrt(np.maximum(0, squares[n]/count - mean**2)).tolist(),
                            mean_accel_norm_g_assumed=norm_acc[n]/count,
                            near_int16_limit_gyro_records=near_limit[n])
        if "gyro_dps_per_lsb_by_node" in config:
            nodes[str(n)]["configured_gyro_dps_per_lsb"] = gyro_scale(config, n)
        if rate and abs(rate/config["sample_rate_hz"]-1) > .2:
            warnings.append(f"Node {n} receive rate differs more than 20% from the nominal fusion rate")
    report = dict(schema_version=1, source=str(path), total_records=total, nodes=nodes,
                configured_sample_rate_hz=config["sample_rate_hz"],
                gyro_dps_per_lsb=config["gyro_dps_per_lsb"], accel_g_per_lsb=config["accel_g_per_lsb"],
                units_verified=config["units_verified"], warnings=warnings)
    if "gyro_dps_per_lsb_by_node" in config:
        report["gyro_dps_per_lsb_by_node"] = dict(config["gyro_dps_per_lsb_by_node"])
    return report


def parser():
    result = argparse.ArgumentParser(description="Experimental 12-IMU glove hand-pose tool (no ROS required)")
    sub = result.add_subparsers(dest="command", required=True)
    for name, help_text in (("live", "Live viewer; serial connection is manual"),
                            ("replay", "Replay raw CSV; no hardware"),
                            ("demo", "Synthetic 3D hand and palm motion; no hardware"),
                            ("inspect", "Read-only raw CSV diagnostics")):
        command = sub.add_parser(name, help=help_text)
        command.add_argument("--config", type=Path)
        if name in ("replay", "inspect"):
            command.add_argument("csv", type=Path)
        if name in ("live", "replay"):
            command.add_argument("--calibration", type=Path)
        if name == "live":
            command.add_argument("--port", help="Prefill an explicit port; still requires clicking Connect")
        if name in ("demo", "replay"):
            command.add_argument("--headless", action="store_true")
            command.add_argument("--output", type=Path, help="Exclusive-create headless JSON result")
        if name != "inspect":
            command.add_argument("--smoke-seconds", type=positive, help="Close GUI after N seconds (testing only)")
        if name == "demo":
            command.add_argument("--seconds", type=positive, default=2.0, help="Synthetic headless duration")
        if name == "inspect":
            command.add_argument("--output", type=Path)
    command = sub.add_parser("config", help="Print or exclusive-create the example settings")
    command.add_argument("--output", type=Path)
    command = sub.add_parser("quick-live", help="Fast experimental full-hand preview with one open-hand reference; no strict A/B calibration")
    command.add_argument("--config", type=Path)
    command.add_argument("--port", default="", help="Prefill only; click Connect to open the selected port")
    command.add_argument("--smoke-seconds", type=positive, help="Close the unconnected GUI after N seconds")
    command = sub.add_parser("hinge-live", help="Manual dual-node natural calibration and experimental local-angle preview")
    command.add_argument("--config", type=Path)
    command.add_argument("--finger", choices=("thumb", "index", "middle", "ring", "little"), default="thumb")
    command.add_argument("--port", help="Prefill only; click Connect to open the selected port")
    command.add_argument("--smoke-seconds", type=positive, help="Close the unconnected GUI after N seconds")
    command = sub.add_parser("validate-hinge", help="Analyze recorded online output or replay a marked experiment; not absolute accuracy")
    command.add_argument("protocol", type=Path, help="Exported *_validation_protocol.json beside its raw CSV and frozen profile")
    command.add_argument("--source", choices=("auto", "online", "replay"), default="auto",
                         help="Prefer recorded full-rate online outputs when available; replay explicitly reinitializes")
    command.add_argument("--observer", choices=("baseline", "constrained"),
                         help="Replay algorithm (default constrained); use with --source replay for new protocols")
    command.add_argument("--max-pairs", type=int, default=30000)
    command.add_argument("--byte-limit", type=int)
    command.add_argument("--output", type=Path)
    command.add_argument("--html", type=Path, help="Exclusive-create an offline readable result table")
    command = sub.add_parser("diagnose-session", help="Read-only diagnostic JSONL analysis and exact-state replay")
    command.add_argument("session", type=Path)
    command.add_argument("--output", type=Path, help="Exclusive-create diagnostic JSON report")
    command.add_argument("--raw-only", action="store_true", help="Summarize captured data without restoring VQF state")
    command = sub.add_parser("fit-hinge", help="Offline natural-motion hinge-axis candidate; no live calibration writes")
    command.add_argument("capture", type=Path, help="Raw CSV, diagnostic JSONL or paired NPZ")
    command.add_argument("--config", type=Path)
    command.add_argument("--proximal", type=int, default=5)
    command.add_argument("--distal", type=int, default=6)
    command.add_argument("--archive-prefix", default="", help="NPZ capture prefix, e.g. a1_")
    command.add_argument("--byte-limit", type=int, help="Fixed input prefix limit for CSV/JSONL")
    command.add_argument("--start-seconds", type=float, default=0.)
    command.add_argument("--duration-seconds", type=positive)
    command.add_argument("--max-pairs", type=int, default=10000)
    command.add_argument("--proximal-bias-dps", type=float, nargs=3)
    command.add_argument("--distal-bias-dps", type=float, nargs=3)
    command.add_argument("--output", type=Path, help="New report file; never overwrite a calibration")
    command = sub.add_parser("calibrate-hinge", help="Offline natural-motion profile from separate declared neutral and motion captures")
    command.add_argument("neutral", type=Path, help="Raw CSV or diagnostic JSONL; explicitly declared reference pose")
    command.add_argument("capture", type=Path, help="Raw CSV or diagnostic JSONL containing natural hinge motion")
    command.add_argument("--config", type=Path)
    command.add_argument("--proximal", type=int, default=5)
    command.add_argument("--distal", type=int, default=6)
    command.add_argument("--neutral-start-seconds", type=float, default=0.)
    command.add_argument("--neutral-duration-seconds", type=positive, default=5.)
    command.add_argument("--motion-start-seconds", type=float, default=0.)
    command.add_argument("--motion-duration-seconds", type=positive, default=20.)
    command.add_argument("--max-pairs", type=int, default=10000)
    command.add_argument("--byte-limit", type=int)
    command.add_argument("--output", type=Path)
    for name, help_text in (("replay-hinge", "Offline relative-angle replay using a natural-motion candidate profile"),
                            ("compare-hinge", "Compare baseline and experimental constrained observers on identical data")):
        command = sub.add_parser(name, help=help_text)
        command.add_argument("capture", type=Path, help="Independent raw CSV or diagnostic JSONL motion")
        command.add_argument("--profile", type=Path, required=True)
        command.add_argument("--config", type=Path, help="Optional cross-check; otherwise use profile units and rate")
        command.add_argument("--start-seconds", type=float, default=0.)
        command.add_argument("--duration-seconds", type=positive)
        command.add_argument("--max-pairs", type=int, default=10000)
        command.add_argument("--byte-limit", type=int)
        command.add_argument("--output", type=Path)
        if name == "replay-hinge":
            command.add_argument("--observer", choices=("baseline", "constrained"), default="baseline")
        else:
            command.add_argument("--html", type=Path, help="Exclusive-create a self-contained comparison plot")
    command = sub.add_parser("benchmark-hinge", help="Synthetic observer stress tests; never opens hardware")
    command.add_argument("--cases", nargs="+", help="Scenario names; default all fixed scenarios")
    command.add_argument("--sample-rate-hz", type=positive, default=120.)
    command.add_argument("--seed", type=int, default=7)
    command.add_argument("--include-series", action="store_true")
    command.add_argument("--html", type=Path, help="Comparison plots; also includes traces in the JSON report")
    command.add_argument("--output", type=Path)
    return result


def _output(data, path=None):
    if path:
        write_json_exclusive(path, data)
    else:
        print(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))


def _comparison_output(data, args):
    html_path = getattr(args, "html", None)
    content = None
    if html_path:
        from .hinge_report import render_comparison_html
        if args.output and html_path.resolve() == args.output.resolve():
            raise ValueError("JSON and HTML outputs must be different paths")
        if html_path.exists():
            raise FileExistsError(f"output already exists: {html_path}")
        content = render_comparison_html(data)
    _output(data, args.output)
    if html_path:
        with html_path.open("x", encoding="utf-8") as stream:
            stream.write(content)


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        config = load_config(getattr(args, "config", None))
        if args.command == "config":
            _output(config, args.output)
        elif args.command == "quick-live":
            from .quick_hand_live import launch_quick_hand
            return launch_quick_hand(config, port=args.port, smoke_seconds=args.smoke_seconds)
        elif args.command == "hinge-live":
            from .hinge_gui import launch_hinge_gui
            return launch_hinge_gui(config, finger=args.finger, port=args.port,
                                    smoke_seconds=args.smoke_seconds)
        elif args.command == "validate-hinge":
            from .hinge_validation_io import validate_hinge_protocol
            from .hinge_validation import render_validation_html
            report = validate_hinge_protocol(args.protocol, observer=args.observer, source=args.source,
                                            max_pairs=args.max_pairs, byte_limit=args.byte_limit)
            content = None
            if args.html:
                if args.output and args.html.resolve() == args.output.resolve():
                    raise ValueError("JSON and HTML outputs must be different paths")
                if args.html.exists():
                    raise FileExistsError(f"output already exists: {args.html}")
                content = render_validation_html(report)
            _output(report, args.output)
            if args.html:
                with args.html.open("x", encoding="utf-8") as output:
                    output.write(content)
        elif args.command == "benchmark-hinge":
            from .hinge_benchmark import benchmark_observers
            report = benchmark_observers(cases=args.cases, sample_rate_hz=args.sample_rate_hz,
                                         seed=args.seed, include_series=args.include_series or bool(args.html))
            _comparison_output(report, args)
        elif args.command in ("calibrate-hinge", "replay-hinge", "compare-hinge"):
            from .hinge_replay import calibrate_hinge_pair, replay_hinge_pair, compare_hinge_pair, _profile
            from .pair_capture import load_pair_capture
            from .hinge_settings import hinge_settings_match, settings_from_units
            if not 1 <= args.max_pairs <= 30000:
                raise ValueError("max-pairs must be in [1, 30000]")
            options = dict(max_pairs=args.max_pairs, byte_limit=args.byte_limit)
            if args.command == "calibrate-hinge":
                options.update(proximal=args.proximal, distal=args.distal,
                               sample_rate_hz=config["sample_rate_hz"],
                               gyro_dps_per_lsb=config["gyro_dps_per_lsb"],
                               gyro_dps_per_lsb_by_node=config.get("gyro_dps_per_lsb_by_node"))
                neutral = load_pair_capture(args.neutral, **options,
                                           start_s=args.neutral_start_seconds,
                                           duration_s=args.neutral_duration_seconds)
                motion = load_pair_capture(args.capture, **options,
                                          start_s=args.motion_start_seconds,
                                          duration_s=args.motion_duration_seconds)
                report = calibrate_hinge_pair(neutral, motion, accel_g_per_lsb=config["accel_g_per_lsb"])
            else:
                if args.profile.stat().st_size > 4 * 1024 * 1024:
                    raise ValueError("offline hinge profile exceeds 4 MiB")
                with args.profile.open(encoding="utf-8") as source:
                    profile = json.load(source)
                settings, _, _ = _profile(profile)
                if args.config:
                    if not hinge_settings_match(settings_from_units(config, settings["node_ids"]), settings):
                        raise ValueError("explicit configuration differs from profile nodes, rate or effective units")
                options.update(proximal=settings["node_ids"][0], distal=settings["node_ids"][1],
                               sample_rate_hz=settings["sample_rate_hz"],
                               gyro_dps_per_lsb=settings["gyro_dps_per_lsb"],
                               gyro_dps_per_lsb_by_node=settings.get("gyro_dps_per_lsb_by_node"))
                motion = load_pair_capture(args.capture, **options,
                                          start_s=args.start_seconds, duration_s=args.duration_seconds)
                report = (compare_hinge_pair(profile, motion) if args.command == "compare-hinge"
                          else replay_hinge_pair(profile, motion, observer=args.observer))
            _comparison_output(report, args)
            # Offline diagnostics return a result even when a candidate is
            # rejected. Check profile_status / replay_status, not just exit 0.
        elif args.command == "fit-hinge":
            from .hinge_lab import run_hinge_experiment
            report = run_hinge_experiment(
                args.capture, config, proximal=args.proximal, distal=args.distal,
                archive_prefix=args.archive_prefix, byte_limit=args.byte_limit,
                start_s=args.start_seconds, duration_s=args.duration_seconds,
                max_pairs=args.max_pairs, proximal_bias_dps=args.proximal_bias_dps,
                distal_bias_dps=args.distal_bias_dps)
            _output(report, args.output)
            # Exit 0 means a report was produced, NOT calibration acceptance.
            # The report is deliberately not compatible with live calibration.
        elif args.command == "diagnose-session":
            from .diagnostics import diagnose_session
            report = diagnose_session(args.session, raw_only=args.raw_only)
            _output(report, args.output)
            # A valid prefix report is still useful, but must not be a success
            # verdict for an incomplete/nonreproducible experiment.
            return 0 if report["session_complete"] and (args.raw_only or report["reconstruction"]["reproducible"]) else 1
        elif args.command == "inspect":
            _output(inspect_csv(args.csv, config), args.output)
        elif getattr(args, "headless", False):
            from .engine import PoseEngine, demo_snapshot
            started = time.perf_counter()
            if args.command == "demo":
                if args.seconds > 60:
                    raise ValueError("Headless demo duration is limited to 60 seconds")
                frames = max(1, int(args.seconds * 30))
                for i in range(frames):
                    snapshot = demo_snapshot(i / 30, config)
                snapshot["processed_frames"] = frames
            else:
                engine = PoseEngine(config)
                if args.calibration:
                    engine.load_calibration(args.calibration)
                rows = 0
                for sample in iter_csv(args.csv):
                    engine.process(sample)
                    rows += 1
                if not rows:
                    raise ValueError("CSV contains no samples")
                snapshot = engine.get_snapshot(now=engine.last_time)
                snapshot.update(source="replay", processed_rows=rows, eof=True)
                snapshot["warnings"].append("EOF: this is a final historical snapshot, not a live stream. No calibration was silently inferred from the recording.")
            snapshot["processing_elapsed_s"] = time.perf_counter() - started
            _output(snapshot, args.output)
        else:
            if getattr(args, "output", None):
                raise ValueError("--output requires --headless for demo/replay")
            from .gui import launch_gui
            source = args.port if args.command == "live" else str(args.csv) if args.command == "replay" else None
            return launch_gui(config, mode=args.command, source=source,
                              calibration_path=str(args.calibration) if getattr(args, "calibration", None) else None,
                              smoke_seconds=args.smoke_seconds)
        return 0
    except (ValueError, OSError, RuntimeError, ImportError) as exc:
        print(f"vt-glove-pose: {exc}", file=sys.stderr)
        return 2
