from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from .clock_audit import ClockAudit, audit_clock_stream
from .config import AlignmentConfig
from .extrinsics import ExtrinsicBinding
from .matcher import (
    interpolate_pose,
    match_nearest_unique,
    select_generic_sample,
)
from .model import BagDataset, InterpolatedPose


@dataclass(frozen=True)
class AlignmentResult:
    records: tuple[dict[str, object], ...]
    timing_residuals: tuple[dict[str, object], ...]
    quality: dict[str, object]
    clock_audits: tuple[ClockAudit, ...]
    rejection_reasons: tuple[str, ...]

    @property
    def accepted(self) -> bool:
        return not self.rejection_reasons


def _milliseconds_to_nanoseconds(value: float) -> int:
    return int(round(value * 1_000_000.0))


def _pose_document(
    pose: InterpolatedPose,
    *,
    role: str,
    tracker_id: str,
) -> dict[str, object]:
    return {
        "role": role,
        "tracker_id": tracker_id,
        "timestamp_ns": pose.timestamp_ns,
        "bracket_gap_ns": pose.bracket_gap_ns,
        "before_sequence": pose.before_sequence,
        "after_sequence": pose.after_sequence,
        "world_from_tracker": pose.transform.as_document(),
    }


def _ratio(numerator: int, denominator: int) -> float:
    return numerator / denominator if denominator else 0.0


def align_dataset(
    dataset: BagDataset,
    config: AlignmentConfig,
    extrinsics: Mapping[str, ExtrinsicBinding],
) -> AlignmentResult:
    expected_cameras = {camera.name for camera in config.cameras}
    if set(extrinsics) != expected_cameras:
        raise ValueError("extrinsics must contain every configured camera exactly once")
    reference_frames = dataset.camera_frames[config.reference_camera]
    if not reference_frames:
        raise ValueError("reference camera has no complete frames")
    max_camera_delta_ns = _milliseconds_to_nanoseconds(
        config.thresholds.max_camera_delta_ms
    )
    max_tracker_gap_ns = _milliseconds_to_nanoseconds(
        config.thresholds.max_tracker_gap_ms
    )
    max_clock_step_ns = _milliseconds_to_nanoseconds(
        config.thresholds.max_clock_step_ms
    )
    reference_times = tuple(
        frame.host_realtime_ns for frame in reference_frames
    )
    matches: dict[str, tuple[int | None, ...]] = {}
    for camera in config.cameras:
        values = dataset.camera_frames[camera.name]
        if camera.name == config.reference_camera:
            matches[camera.name] = tuple(range(len(reference_frames)))
        else:
            matches[camera.name] = match_nearest_unique(
                reference_times,
                tuple(frame.host_realtime_ns for frame in values),
                max_delta_ns=max_camera_delta_ns,
            )

    clock_audits: list[ClockAudit] = []
    for stream_name in (
        *(f"camera:{camera.name}" for camera in config.cameras),
        *(f"tracker:{tracker.role}" for tracker in config.trackers),
    ):
        clock_audits.append(
            audit_clock_stream(
                stream_name,
                dataset.clock_observations[stream_name],
                max_step_ns=max_clock_step_ns,
            )
        )

    camera_matches = {camera.name: 0 for camera in config.cameras}
    tracker_reference = {tracker.role: 0 for tracker in config.trackers}
    attached_tracker = {camera.name: 0 for camera in config.cameras}
    additional_matches = {
        stream.name: 0 for stream in config.additional_streams
    }
    records: list[dict[str, object]] = []
    residuals: list[dict[str, object]] = []
    for frame_index, reference_frame in enumerate(reference_frames):
        reference_time = reference_frame.host_realtime_ns
        flags: list[str] = []
        tracker_documents: dict[str, object] = {}
        for tracker in config.trackers:
            pose = interpolate_pose(
                dataset.tracker_poses[tracker.role],
                reference_time,
                max_tracker_gap_ns,
            )
            if pose is None:
                tracker_documents[tracker.role] = None
                flags.append(f"missing_tracker_at_reference:{tracker.role}")
            else:
                tracker_reference[tracker.role] += 1
                tracker_documents[tracker.role] = _pose_document(
                    pose,
                    role=tracker.role,
                    tracker_id=dataset.tracker_ids[tracker.role],
                )

        camera_documents: dict[str, object] = {}
        for camera in config.cameras:
            selected_index = matches[camera.name][frame_index]
            if selected_index is None:
                camera_documents[camera.name] = None
                flags.append(f"missing_camera:{camera.name}")
                residuals.append(
                    {
                        "frame_index": frame_index,
                        "camera": camera.name,
                        "camera_delta_ns": None,
                        "attached_tracker_gap_ns": None,
                    }
                )
                continue
            camera_matches[camera.name] += 1
            frame = dataset.camera_frames[camera.name][selected_index]
            delta = frame.host_realtime_ns - reference_time
            attached = interpolate_pose(
                dataset.tracker_poses[camera.tracker_role],
                frame.host_realtime_ns,
                max_tracker_gap_ns,
            )
            attached_document = None
            world_from_camera = None
            if attached is None:
                flags.append(f"missing_attached_tracker:{camera.name}")
            else:
                attached_tracker[camera.name] += 1
                attached_document = _pose_document(
                    attached,
                    role=camera.tracker_role,
                    tracker_id=dataset.tracker_ids[camera.tracker_role],
                )
                world_from_camera = attached.transform.compose(
                    extrinsics[camera.name].tracker_from_camera
                ).as_document()
            camera_documents[camera.name] = {
                "host_realtime_ns": frame.host_realtime_ns,
                "source_timestamp_ns": frame.source_timestamp_ns,
                "delta_ns": delta,
                "color": frame.color.as_document(),
                "depth": frame.depth.as_document(),
                "timing": frame.timing.as_document(),
                "attached_tracker": attached_document,
                "world_from_camera": world_from_camera,
            }
            residuals.append(
                {
                    "frame_index": frame_index,
                    "camera": camera.name,
                    "camera_delta_ns": delta,
                    "attached_tracker_gap_ns": (
                        attached.bracket_gap_ns if attached is not None else None
                    ),
                }
            )

        additional_documents: dict[str, object] = {}
        for stream in config.additional_streams:
            samples = dataset.additional_samples[stream.name]
            selected = select_generic_sample(
                samples,
                timestamp_ns=reference_time,
                strategy=stream.strategy,
                max_delta_ns=_milliseconds_to_nanoseconds(stream.max_delta_ms),
            )
            if selected is None:
                additional_documents[stream.name] = None
                if stream.required:
                    flags.append(f"missing_required_stream:{stream.name}")
                continue
            additional_matches[stream.name] += 1
            sample = samples[selected]
            additional_documents[stream.name] = {
                "timestamp_ns": sample.timestamp_ns,
                "delta_ns": sample.timestamp_ns - reference_time,
                "message": sample.reference.as_document(),
            }
        records.append(
            {
                "frame_index": frame_index,
                "reference_camera": config.reference_camera,
                "reference_time_ns": reference_time,
                "cameras": camera_documents,
                "trackers": tracker_documents,
                "additional_streams": additional_documents,
                "quality_flags": flags,
            }
        )

    count = len(reference_frames)
    camera_ratios = {
        name: _ratio(value, count) for name, value in camera_matches.items()
    }
    tracker_ratios = {
        role: _ratio(value, count) for role, value in tracker_reference.items()
    }
    attached_ratios = {
        name: _ratio(value, count) for name, value in attached_tracker.items()
    }
    additional_ratios = {
        name: _ratio(value, count) for name, value in additional_matches.items()
    }
    reasons: list[str] = []
    for audit in clock_audits:
        if not audit.valid:
            reasons.append(f"clock_audit_failed:{audit.stream_name}")
    for name, ratio in camera_ratios.items():
        if ratio < config.thresholds.min_camera_match_ratio:
            reasons.append(f"camera_coverage_below_threshold:{name}")
    for role, ratio in tracker_ratios.items():
        if ratio < config.thresholds.min_tracker_coverage_ratio:
            reasons.append(f"tracker_coverage_below_threshold:{role}")
    for name, ratio in attached_ratios.items():
        if ratio < config.thresholds.min_tracker_coverage_ratio:
            reasons.append(f"attached_tracker_coverage_below_threshold:{name}")
    for stream in config.additional_streams:
        if (
            stream.required
            and additional_ratios[stream.name]
            < config.thresholds.min_required_stream_coverage_ratio
        ):
            reasons.append(
                f"required_stream_coverage_below_threshold:{stream.name}"
            )
    quality = {
        "verdict": "ACCEPTED" if not reasons else "REJECTED",
        "reference_frame_count": count,
        "camera_match_ratio": camera_ratios,
        "tracker_reference_coverage_ratio": tracker_ratios,
        "attached_tracker_coverage_ratio": attached_ratios,
        "additional_stream_coverage_ratio": additional_ratios,
        "thresholds": {
            "max_camera_delta_ms": config.thresholds.max_camera_delta_ms,
            "max_tracker_gap_ms": config.thresholds.max_tracker_gap_ms,
            "max_clock_step_ms": config.thresholds.max_clock_step_ms,
            "min_camera_match_ratio": config.thresholds.min_camera_match_ratio,
            "min_tracker_coverage_ratio": (
                config.thresholds.min_tracker_coverage_ratio
            ),
            "min_required_stream_coverage_ratio": (
                config.thresholds.min_required_stream_coverage_ratio
            ),
        },
        "clock_audits": [audit.as_document() for audit in clock_audits],
        "rejection_reasons": reasons,
    }
    return AlignmentResult(
        records=tuple(records),
        timing_residuals=tuple(residuals),
        quality=quality,
        clock_audits=tuple(clock_audits),
        rejection_reasons=tuple(reasons),
    )
