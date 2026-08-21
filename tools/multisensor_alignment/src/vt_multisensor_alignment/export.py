from __future__ import annotations

import csv
from datetime import datetime, timezone
import hashlib
import html
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Mapping

from .aligner import AlignmentResult
from .bag_reader import configured_topic_types
from .config import AlignmentConfig
from .extrinsics import ExtrinsicBinding
from .model import BagDataset


TOOL_VERSION = "0.2.0"
OUTPUT_FILES = frozenset(
    {
        "manifest.json",
        "stream_catalog.json",
        "aligned_frames.jsonl",
        "timing_residuals.csv",
        "quality_report.json",
        "diagnostics.svg",
    }
)
_INTEGRITY_FILES = OUTPUT_FILES - {"manifest.json"}


def _json_bytes(document: object) -> bytes:
    return (
        json.dumps(
            document,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=False,
        )
        + "\n"
    ).encode("utf-8")


def _write_private(path: Path, payload: bytes) -> None:
    path.write_bytes(payload)
    path.chmod(0o600)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _stream_catalog(
    config: AlignmentConfig, dataset: BagDataset
) -> dict[str, object]:
    core_topics: set[str] = set()
    for camera in config.cameras:
        core_topics.update(
            {
                camera.color_topic,
                camera.depth_topic,
                camera.camera_info_topic,
                camera.timing_topic,
            }
        )
    core_topics.update(
        tracker.sample_topic for tracker in config.trackers
    )
    additional_by_topic = {
        stream.topic: stream for stream in config.additional_streams
    }
    streams = []
    for topic, type_name in configured_topic_types(config).items():
        extension = additional_by_topic.get(topic)
        entry: dict[str, object] = {
            "topic": topic,
            "type": type_name,
            "contract": "core" if topic in core_topics else "extension",
            "required": topic in config.required_topics,
            "present": topic in dataset.topic_types,
            "message_count": dataset.message_counts.get(topic, 0),
            "accepted_count": dataset.accepted_counts.get(topic, 0),
        }
        if extension is not None:
            entry.update(
                {
                    "stream_name": extension.name,
                    "timestamp_field": extension.timestamp_field,
                    "selection_strategy": extension.strategy,
                    "max_delta_ms": extension.max_delta_ms,
                }
            )
        streams.append(entry)
    return {
        "schema_version": 1,
        "bag_contract": "unified-dataset-v1",
        "streams": streams,
        "incomplete_camera_groups": dict(dataset.incomplete_camera_groups),
    }


def _diagnostics_svg(
    quality: Mapping[str, object], config: AlignmentConfig
) -> str:
    camera_ratios = quality["camera_match_ratio"]
    tracker_ratios = quality["tracker_reference_coverage_ratio"]
    attached_ratios = quality["attached_tracker_coverage_ratio"]
    rows: list[tuple[str, float, float]] = []
    for camera in config.cameras:
        rows.append(
            (
                f"camera {camera.name}",
                float(camera_ratios[camera.name]),
                config.thresholds.min_camera_match_ratio,
            )
        )
        rows.append(
            (
                f"attached Tracker {camera.name}",
                float(attached_ratios[camera.name]),
                config.thresholds.min_tracker_coverage_ratio,
            )
        )
    for tracker in config.trackers:
        rows.append(
            (
                f"Tracker {tracker.role} @ reference",
                float(tracker_ratios[tracker.role]),
                config.thresholds.min_tracker_coverage_ratio,
            )
        )
    width = 1000
    height = 100 + len(rows) * 42
    bars = []
    for index, (label, ratio, threshold) in enumerate(rows):
        y = 80 + index * 42
        bar_width = max(0.0, min(ratio, 1.0)) * 500.0
        threshold_x = 390.0 + max(0.0, min(threshold, 1.0)) * 500.0
        color = "#1a7f37" if ratio >= threshold else "#cf222e"
        bars.append(
            f'<text x="30" y="{y + 15}" font-family="sans-serif" '
            f'font-size="14">{html.escape(label)}</text>'
            f'<rect x="390" y="{y}" width="500" height="20" '
            f'fill="#eaeef2"/>'
            f'<rect x="390" y="{y}" width="{bar_width:.1f}" height="20" '
            f'fill="{color}"/>'
            f'<line x1="{threshold_x:.1f}" x2="{threshold_x:.1f}" '
            f'y1="{y - 3}" y2="{y + 23}" stroke="#24292f"/>'
            f'<text x="905" y="{y + 15}" font-family="monospace" '
            f'font-size="14">{ratio:.3f}</text>'
        )
    verdict = html.escape(str(quality["verdict"]))
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}">\n'
        '<rect width="100%" height="100%" fill="white"/>\n'
        f'<text x="30" y="38" font-family="sans-serif" font-size="24">'
        f'Unified alignment — {verdict}</text>\n'
        + "\n".join(bars)
        + "\n</svg>\n"
    )


def export_result(
    *,
    output_directory: str | Path,
    config_path: str | Path,
    config: AlignmentConfig,
    dataset: BagDataset,
    extrinsics: Mapping[str, ExtrinsicBinding],
    result: AlignmentResult,
) -> Path:
    target = Path(output_directory).resolve()
    if target.exists():
        raise FileExistsError(f"refusing to overwrite output directory: {target}")
    if not target.parent.is_dir():
        raise FileNotFoundError(f"output parent does not exist: {target.parent}")
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{target.name}.", dir=target.parent)
    )
    temporary.chmod(0o700)
    try:
        catalog = _stream_catalog(config, dataset)
        _write_private(
            temporary / "stream_catalog.json", _json_bytes(catalog)
        )
        frame_path = temporary / "aligned_frames.jsonl"
        with frame_path.open("w", encoding="utf-8") as stream:
            for record in result.records:
                json.dump(
                    record,
                    stream,
                    ensure_ascii=False,
                    allow_nan=False,
                    separators=(",", ":"),
                )
                stream.write("\n")
        frame_path.chmod(0o600)
        residual_path = temporary / "timing_residuals.csv"
        with residual_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=(
                    "frame_index",
                    "camera",
                    "camera_delta_ns",
                    "attached_tracker_gap_ns",
                ),
            )
            writer.writeheader()
            writer.writerows(result.timing_residuals)
        residual_path.chmod(0o600)
        _write_private(
            temporary / "quality_report.json", _json_bytes(result.quality)
        )
        _write_private(
            temporary / "diagnostics.svg",
            _diagnostics_svg(result.quality, config).encode("utf-8"),
        )
        inventory = {
            name: {
                "size_bytes": (temporary / name).stat().st_size,
                "sha256": _sha256(temporary / name),
            }
            for name in sorted(_INTEGRITY_FILES)
        }
        config_source = Path(config_path).resolve()
        metadata_path = dataset.bag_path / "metadata.yaml"
        manifest = {
            "schema_version": 1,
            "tool": "vt-multisensor-alignment",
            "tool_version": TOOL_VERSION,
            "created_at": datetime.now(timezone.utc).astimezone().isoformat(
                timespec="seconds"
            ),
            "bag_contract": "unified-dataset-v1",
            "source_bag": {
                "name": dataset.bag_path.name,
                "storage_identifier": dataset.storage_identifier,
                "metadata_sha256": _sha256(metadata_path),
            },
            "configuration": {
                "file_name": config_source.name,
                "sha256": _sha256(config_source),
                "reference_camera": config.reference_camera,
                "world_frame": config.world_frame,
            },
            "tracker_ids": dict(dataset.tracker_ids),
            "extrinsics": {
                name: value.as_manifest_document()
                for name, value in sorted(extrinsics.items())
            },
            "aligned_frame_count": len(result.records),
            "verdict": result.quality["verdict"],
            "files": inventory,
        }
        _write_private(temporary / "manifest.json", _json_bytes(manifest))
        os.rename(temporary, target)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return target


def validate_output(output_directory: str | Path) -> dict[str, object]:
    target = Path(output_directory).resolve()
    if not target.is_dir():
        raise FileNotFoundError(f"alignment output directory does not exist: {target}")
    observed_files = {path.name for path in target.iterdir() if path.is_file()}
    if observed_files != OUTPUT_FILES:
        raise ValueError(
            "alignment output file set mismatch: "
            f"expected={sorted(OUTPUT_FILES)} observed={sorted(observed_files)}"
        )
    manifest = json.loads(
        (target / "manifest.json").read_text(encoding="utf-8")
    )
    if manifest.get("schema_version") != 1:
        raise ValueError("unsupported alignment manifest schema")
    inventory = manifest.get("files")
    if type(inventory) is not dict or set(inventory) != _INTEGRITY_FILES:
        raise ValueError("alignment manifest file inventory is malformed")
    for name, expected in inventory.items():
        path = target / name
        if (
            expected.get("size_bytes") != path.stat().st_size
            or expected.get("sha256") != _sha256(path)
        ):
            raise ValueError(f"output integrity mismatch: {name}")
    quality = json.loads(
        (target / "quality_report.json").read_text(encoding="utf-8")
    )
    frame_count = 0
    with (target / "aligned_frames.jsonl").open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                raise ValueError(f"blank aligned record at line {line_number}")
            json.loads(line)
            frame_count += 1
    if frame_count != manifest.get("aligned_frame_count"):
        raise ValueError("aligned frame count differs from manifest")
    if quality.get("verdict") != manifest.get("verdict"):
        raise ValueError("quality verdict differs from manifest")
    return {
        "verdict": quality["verdict"],
        "aligned_frame_count": frame_count,
        "tool_version": manifest.get("tool_version"),
    }
