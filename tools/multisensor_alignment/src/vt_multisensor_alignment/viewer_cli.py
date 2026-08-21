from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Callable

from .dataset import AlignedDataset
from .errors import DatasetError
from .export import TOOL_VERSION
from .viewer_model import ViewerConfig


DatasetOpener = Callable[..., AlignedDataset]
InteractiveRunner = Callable[..., None]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="vt-multisensor-view",
        description="Visualize an aligned VisionTactile dataset offline",
    )
    parser.add_argument("--alignment", required=True, type=Path)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=900)
    parser.add_argument("--depth-min-m", type=float, default=0.2)
    parser.add_argument("--depth-max-m", type=float, default=3.0)
    parser.add_argument("--tracker-range-m", type=float, default=2.0)
    parser.add_argument("--allow-rejected", action="store_true")
    parser.add_argument("--skip-integrity", action="store_true")
    parser.add_argument(
        "--export-frame",
        type=Path,
        help="write the selected dashboard frame as PNG and exit without a window",
    )
    return parser


def _normalized_start(index: int, count: int) -> int:
    normalized = index + count if index < 0 else index
    if not 0 <= normalized < count:
        raise ValueError(f"start frame is outside dataset: {index}")
    return normalized


def _snapshot_target(path: Path) -> Path:
    target = path.expanduser().resolve()
    if target.suffix.lower() != ".png":
        raise ValueError("--export-frame must use a .png path")
    if target.exists():
        raise FileExistsError(f"refusing to overwrite snapshot: {target}")
    if not target.parent.is_dir():
        raise FileNotFoundError(
            f"snapshot parent directory does not exist: {target.parent}"
        )
    return target


def _export_snapshot(
    dataset: AlignedDataset,
    *,
    index: int,
    speed: float,
    config: ViewerConfig,
    target: Path,
) -> dict[str, object]:
    from .viewer_render import render_aligned_frame

    frame = dataset.frame(
        index,
        include_timing=False,
        additional_streams=(),
    )
    image = render_aligned_frame(
        frame,
        camera_names=dataset.camera_names,
        total_frames=len(dataset),
        playing=False,
        speed=speed,
        config=config,
    )
    image.save(target, format="PNG")
    return {
        "output": str(target),
        "frame_index": frame.frame_index,
        "reference_time_ns": frame.reference_time_ns,
    }


def main(
    argv: list[str] | None = None,
    *,
    _dataset_opener: DatasetOpener = AlignedDataset.open,
    _interactive_runner: InteractiveRunner | None = None,
) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if arguments == ["--version"]:
        print(f"vt-multisensor-view {TOOL_VERSION}")
        return 0
    try:
        parsed = _parser().parse_args(arguments)
        config = ViewerConfig(
            canvas_width=parsed.width,
            canvas_height=parsed.height,
            depth_min_m=parsed.depth_min_m,
            depth_max_m=parsed.depth_max_m,
            tracker_range_m=parsed.tracker_range_m,
        )
        target = (
            _snapshot_target(parsed.export_frame)
            if parsed.export_frame is not None
            else None
        )
        with _dataset_opener(
            parsed.alignment,
            parsed.bag,
            allow_rejected=parsed.allow_rejected,
            verify_integrity=not parsed.skip_integrity,
        ) as dataset:
            start_index = _normalized_start(parsed.start, len(dataset))
            if target is not None:
                document = _export_snapshot(
                    dataset,
                    index=start_index,
                    speed=parsed.speed,
                    config=config,
                    target=target,
                )
                print(json.dumps(document, indent=2, ensure_ascii=False))
                return 0
            runner = _interactive_runner
            if runner is None:
                from .viewer_app import run_interactive

                runner = run_interactive
            runner(
                dataset,
                start_index=start_index,
                speed=parsed.speed,
                config=config,
            )
        return 0
    except (DatasetError, ImportError, OSError, RuntimeError, TypeError, ValueError) as error:
        print(f"ERROR: {error}")
        return 1


def entrypoint() -> None:
    raise SystemExit(main())
