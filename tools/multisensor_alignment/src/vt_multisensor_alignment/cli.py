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
        validation = validate_output(parsed.output)
        print(json.dumps(validation, indent=2, ensure_ascii=False))
        return 0 if validation["verdict"] == "ACCEPTED" else 2
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        print(f"ERROR: {error}")
        return 1


def entrypoint() -> None:
    raise SystemExit(main())
