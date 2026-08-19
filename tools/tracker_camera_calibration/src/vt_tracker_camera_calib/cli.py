from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .bag_reader import read_calibration_bag
from .charuco import render_board
from .config import load_config
from .config_writer import (
    REFERENCE_CAMERAS,
    TRACKER_ROLES,
    build_config_document,
    write_config,
)
from .export import export_result
from .handeye import solve_hand_eye
from .pairing import pair_static_observations
from .repeatability import compare_extrinsics, write_repeatability_report


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="vt-tracker-camera-calibrate",
        description=(
            "Offline eye-in-hand calibration from a rosbag2 recording. "
            "This tool never opens or controls camera or Tracker hardware."
        ),
    )
    parser.add_argument("--version", action="version", version="%(prog)s 0.3.0")
    subparsers = parser.add_subparsers(dest="command", required=True)
    configure = subparsers.add_parser(
        "configure", help="create one identity-bound calibration config"
    )
    configure.add_argument(
        "--camera", required=True, choices=tuple(REFERENCE_CAMERAS)
    )
    configure.add_argument(
        "--tracker-role", required=True, choices=TRACKER_ROLES
    )
    configure.add_argument("--square-length-mm", required=True, type=float)
    configure.add_argument("--marker-length-mm", required=True, type=float)
    configure.add_argument("--output", required=True, type=Path)
    board = subparsers.add_parser("board", help="render a printable ChArUco board")
    board.add_argument("--config", required=True, type=Path)
    board.add_argument("--output", required=True, type=Path)
    board.add_argument("--dpi", type=int, default=300)

    calibrate = subparsers.add_parser(
        "calibrate", help="solve and validate Tracker-to-camera extrinsics"
    )
    calibrate.add_argument("--bag", required=True, type=Path)
    calibrate.add_argument("--config", required=True, type=Path)
    calibrate.add_argument("--output", required=True, type=Path)
    compare = subparsers.add_parser(
        "compare", help="compare at least three valid calibration runs"
    )
    compare.add_argument("--inputs", required=True, nargs="+", type=Path)
    compare.add_argument("--output", required=True, type=Path)
    compare.add_argument("--max-translation-mm", type=float, default=5.0)
    compare.add_argument("--max-rotation-deg", type=float, default=0.5)
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    if arguments.command == "configure":
        document = build_config_document(
            camera_name=arguments.camera,
            tracker_role=arguments.tracker_role,
            square_length_mm=arguments.square_length_mm,
            marker_length_mm=arguments.marker_length_mm,
        )
        output = write_config(arguments.output, document)
        print(
            f"camera={arguments.camera} tracker_role={arguments.tracker_role} "
            f"output={output}"
        )
        return 0
    if arguments.command == "compare":
        report = compare_extrinsics(
            arguments.inputs,
            threshold_translation_m=arguments.max_translation_mm / 1000.0,
            threshold_rotation_deg=arguments.max_rotation_deg,
        )
        output = write_repeatability_report(arguments.output, report)
        print(
            f"status={report.status} runs={report.run_count} "
            f"maximum_translation_mm={report.maximum_translation_m * 1000:.3f} "
            f"maximum_rotation_deg={report.maximum_rotation_deg:.3f} "
            f"recommended_input={report.recommended_input} output={output}"
        )
        return 0 if report.status == "PASS" else 2
    config = load_config(arguments.config)
    if arguments.command == "board":
        path = render_board(config.board, arguments.output, arguments.dpi)
        print(f"board={path} dpi={arguments.dpi}")
        return 0
    observations = read_calibration_bag(arguments.bag, config)
    pairs = pair_static_observations(
        observations.board_observations,
        observations.tracker_samples,
        config.pairing,
    )
    print(
        f"images={observations.image_count} "
        f"timed_images={observations.timed_image_count} "
        f"board_observations={len(observations.board_observations)} "
        f"tracker_samples={len(observations.tracker_samples)} "
        f"selected_static_pairs={len(pairs)}",
        file=sys.stderr,
    )
    solution = solve_hand_eye(pairs, config.acceptance)
    output = export_result(
        output_directory=arguments.output,
        config_path=arguments.config,
        bag_path=arguments.bag,
        config=config,
        observations=observations,
        selected_pairs=pairs,
        solution=solution,
    )
    print(
        f"quality={solution.quality} method={solution.method} "
        f"pairs={solution.pair_count} "
        f"validation_translation_mm={solution.validation_translation_rms_m * 1000:.3f} "
        f"validation_rotation_deg={solution.validation_rotation_rms_deg:.3f} "
        f"output={output}"
    )
    return 0 if solution.quality != "REJECTED" else 2


def entrypoint() -> None:
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("ERROR: interrupted", file=sys.stderr)
        raise SystemExit(130) from None
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1) from None


if __name__ == "__main__":
    entrypoint()
