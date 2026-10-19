from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from .aligner import align_dataset
from .bag_reader import read_unified_bag
from .config import load_config
from .export import TOOL_VERSION, export_result, validate_output
from .extrinsics import load_extrinsics
from .tactile_adapter import DEFAULT_TACTILE_TOPIC
from .tactile_export import export_tactile


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="vt-multisensor-align",
        description="Audit and align a unified VisionTactile ROS 2 bag",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    inspect_parser = commands.add_parser(
        "inspect", help="audit the unified bag contract without writing output"
    )
    inspect_parser.add_argument("--bag", required=True, type=Path)
    inspect_parser.add_argument("--config", required=True, type=Path)
    align_parser = commands.add_parser(
        "align", help="align one unified bag and create a result directory"
    )
    align_parser.add_argument("--bag", required=True, type=Path)
    align_parser.add_argument("--config", required=True, type=Path)
    align_parser.add_argument("--extrinsics", required=True, type=Path)
    align_parser.add_argument("--output", required=True, type=Path)
    validate_parser = commands.add_parser(
        "validate", help="verify hashes, row count, and quality verdict"
    )
    validate_parser.add_argument("--output", required=True, type=Path)
    tactile_parser = commands.add_parser(
        "export-tactile", help="stream sparse tactile batches into per-camera-frame rolling NPZ grids"
    )
    tactile_parser.add_argument("--bag", required=True, type=Path)
    tactile_parser.add_argument("--aligned-frames", required=True, type=Path)
    tactile_parser.add_argument("--output", required=True, type=Path)
    tactile_parser.add_argument("--topic", default=DEFAULT_TACTILE_TOPIC)
    tactile_parser.add_argument("--expected-hand", choices=("left", "right"),
                                help="identity assertion for custom topics; canonical topics infer the hand")
    tactile_parser.add_argument("--expected-frame-id", help="explicit frame identity for non-default frames")
    tactile_parser.add_argument("--max-age-ms", type=float, default=100.0)
    tactile_parser.add_argument("--max-clock-step-ms", type=float, default=5.0)
    tactile_parser.add_argument(
        "--require-provenance", action="store_true",
        help="require an adjacent alignment manifest; mismatches are always rejected",
    )
    report_parser = commands.add_parser(
        "report-capture", help="write a post-recording quality report without controlling capture"
    )
    report_parser.add_argument("--bag", required=True, type=Path)
    report_parser.add_argument("--output", required=True, type=Path,
                               help="new JSON report path; existing files are never overwritten")
    report_parser.add_argument("--config", type=Path,
                               help="optional alignment config to report missing expected streams; no extrinsics needed")
    profiles_parser = commands.add_parser(
        "make-profiles", help="generate paired unified capture/alignment profiles for selected hands")
    profiles_parser.add_argument("--capture-template", required=True, type=Path)
    profiles_parser.add_argument("--alignment-template", required=True, type=Path)
    profiles_parser.add_argument("--output", required=True, type=Path,
                                 help="new directory containing cameras.yaml and alignment.yaml")
    profiles_parser.add_argument("--pose-hands", default="none", help="none, left, right, both, or left,right")
    profiles_parser.add_argument("--tactile-hands", default="none", help="none, left, right, both, or left,right")
    profiles_parser.add_argument("--require-tactile", action="store_true")
    for hand in ("left", "right"):
        profiles_parser.add_argument(f"--{hand}-reference-node-id", type=int,
                                     help="optional palm node 1..12; otherwise verify the message's own reference")
    return parser


def _inspect(arguments: argparse.Namespace) -> int:
    config = load_config(arguments.config)
    dataset = read_unified_bag(arguments.bag, config)
    document = {
        "bag_name": dataset.bag_path.name,
        "storage_identifier": dataset.storage_identifier,
        "camera_complete_frames": {
            name: len(values) for name, values in dataset.camera_frames.items()
        },
        "valid_tracker_poses": {
            role: len(values) for role, values in dataset.tracker_poses.items()
        },
        "additional_stream_samples": {
            name: len(values)
            for name, values in dataset.additional_samples.items()
        },
        "tracker_identity_stable": {
            role: bool(value) for role, value in dataset.tracker_ids.items()
        },
    }
    print(json.dumps(document, indent=2, ensure_ascii=False))
    return 0


def _align(arguments: argparse.Namespace) -> int:
    config = load_config(arguments.config)
    dataset = read_unified_bag(arguments.bag, config)
    extrinsics = load_extrinsics(
        arguments.extrinsics, config, dataset.tracker_ids
    )
    result = align_dataset(dataset, config, extrinsics)
    output = export_result(
        output_directory=arguments.output,
        config_path=arguments.config,
        config=config,
        dataset=dataset,
        extrinsics=extrinsics,
        result=result,
    )
    print(
        json.dumps(
            {
                "output": str(output),
                "verdict": result.quality["verdict"],
                "aligned_frame_count": len(result.records),
                "rejection_reasons": list(result.rejection_reasons),
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    return 0 if result.accepted else 2


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if arguments == ["--version"]:
        print(f"vt-multisensor-alignment {TOOL_VERSION}")
        return 0
    try:
        parsed = _parser().parse_args(arguments)
        if parsed.command == "inspect":
            return _inspect(parsed)
        if parsed.command == "align":
            return _align(parsed)
        if parsed.command == "make-profiles":
            from .profiles import make_profiles, parse_hands

            output = make_profiles(
                capture_template=parsed.capture_template, alignment_template=parsed.alignment_template,
                output_directory=parsed.output, pose_hands=parse_hands(parsed.pose_hands),
                tactile_hands=parse_hands(parsed.tactile_hands), require_tactile=parsed.require_tactile,
                reference_node_ids={hand: getattr(parsed, f"{hand}_reference_node_id")
                                    for hand in ("left", "right")
                                    if getattr(parsed, f"{hand}_reference_node_id") is not None},
            )
            print(json.dumps({"capture_config": str(output / "cameras.yaml"),
                              "alignment_config": str(output / "alignment.yaml")}, indent=2))
            return 0
        if parsed.command == "report-capture":
            from .capture_quality import generate_capture_quality

            report = generate_capture_quality(
                bag_path=parsed.bag, output_path=parsed.output,
                config=load_config(parsed.config) if parsed.config else None,
            )
            print(json.dumps({
                "output": str(parsed.output.resolve()),
                "status": report["status"], "summary": report["summary"],
                "warnings": report["warnings"],
            }, indent=2, ensure_ascii=False))
            # Quality warnings describe the captured data; they are not an
            # acquisition gate or a failure to write the requested report.
            return 0
        if parsed.command == "export-tactile":
            output = export_tactile(
                bag_path=parsed.bag, aligned_frames=parsed.aligned_frames,
                output_directory=parsed.output, topic=parsed.topic,
                max_age_ms=parsed.max_age_ms, max_clock_step_ms=parsed.max_clock_step_ms,
                require_provenance=parsed.require_provenance,
                expected_hand=parsed.expected_hand, expected_frame_id=parsed.expected_frame_id,
            )
            manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
            print(json.dumps({
                "output": str(output), "provenance": manifest["provenance"],
                "quality": manifest["quality_summary"],
            }, indent=2, ensure_ascii=False))
            return 0
        validation = validate_output(parsed.output)
        print(json.dumps(validation, indent=2, ensure_ascii=False))
        return 0 if validation["verdict"] == "ACCEPTED" else 2
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        print(f"ERROR: {error}")
        return 1


def entrypoint() -> None:
    raise SystemExit(main())
