"""Manual, exclusive export of a dual-node experiment; no acquisition ownership."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from .config import write_json_exclusive
from .stream import RawRecorder


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def export_hinge_session(directory, bundle, *, config, validation_attempts=()):
    """Write immutable raw attempts and a final manifest into a NEW directory.

    Failed/cancelled captures remain evidence, not usable calibrations. A failed
    export deliberately leaves its partial directory; only manifest.json with
    export_complete=true is a completed export. Never replace existing files.
    """
    directory = Path(directory)
    directory.mkdir(exist_ok=False)
    manifest = dict(schema_version=1, kind="vt_glove_pose.experimental_hinge_session",
                    export_complete=False, production_acceptance="not_evaluated",
                    hardware_synchronization_verified=False,
                    scope="selected_six_node_wire_group_not_all_twelve_nodes",
                    config=config, attempts=[], files={})
    write_json_exclusive(directory / "export_started.json", {
        "export_complete": False, "note": "Completion requires manifest.json."})
    attempts = list(bundle.get("attempts", ())) + list(validation_attempts)
    for index, attempt in enumerate(attempts, 1):
        # Names are generated here, never taken from external path metadata.
        kind = attempt.get("kind", "unknown")
        kind = kind if kind in {"neutral", "motion", "direction", "validation"} else "unknown"
        name = f"{index:03d}_{kind}.csv"
        samples = attempt.get("samples", ())
        with RawRecorder(directory / name) as recorder:
            recorder.write(samples)
        item = {key: value for key, value in attempt.items() if key not in {"samples", "series", "profile", "online"}}
        item.update(raw_file=name, raw_sample_count=len(samples))
        if kind == "validation" and attempt.get("profile") is not None:
            profile_name = f"{index:03d}_validation_profile.json"
            protocol_name = f"{index:03d}_validation_protocol.json"
            write_json_exclusive(directory/profile_name, attempt["profile"])
            # The raw CSV starts at its first received sample, not the button
            # press. Preserve declared windows even if data arrived too late.
            origin = samples[0].host_monotonic_s if samples else attempt["started_s"]
            windows = []
            for marker in attempt.get("markers", []):
                window = dict(marker)
                window["start_s"] = marker["start_host_monotonic_s"]-origin
                window["end_s"] = marker["end_host_monotonic_s"]-origin
                windows.append(window)
            protocol = dict(kind="vt_glove_pose.hinge_validation_protocol", schema_version=1,
                time_basis="seconds_from_first_csv_host_parse_time_not_physical_motion_time",
                capture_file=name, profile_file=profile_name,
                capture_sha256=file_sha256(directory/name), profile_sha256=file_sha256(directory/profile_name),
                capture_status=attempt.get("status"), csv_origin_host_monotonic_s=origin,
                windows=windows, direction=attempt.get("direction"),
                guided_quickcheck=attempt.get("guided_quickcheck", False),
                annotation_source=("timed_guidance_not_detected_pose_or_independent_truth"
                                   if attempt.get("guided_quickcheck") else
                                   "operator_declared_pose_not_independent_angle_truth"),
                independent_angle_truth_available=False)
            if "online" in attempt:
                online_name = f"{index:03d}_validation_online.json"
                trace = dict(kind="vt_glove_pose.hinge_online_trace", schema_version=1,
                    settings=attempt["profile"].get("settings"),
                    observer="recorded_constrained_hinge_angle_bias_v1_experimental",
                    state_basis="actual_per_pair_online_output_not_reinitialized_replay_or_ui_snapshots",
                    capture_sha256=protocol["capture_sha256"], profile_sha256=protocol["profile_sha256"],
                    discarded_rows=attempt.get("online_discarded_rows", 0),
                    capture_status=attempt.get("status"), series=attempt["online"])
                # Compact streaming JSON avoids duplicating a large pretty-
                # printed string in memory during a 180-second export.
                with (directory/online_name).open("x", encoding="utf-8") as output:
                    json.dump(trace, output, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
                    output.write("\n")
                protocol.update(online_file=online_name, online_sha256=file_sha256(directory/online_name))
                item["online_file"] = online_name
            write_json_exclusive(directory/protocol_name, protocol)
            item.update(profile_file=profile_name, protocol_file=protocol_name)
        if "series" in attempt:
            series_name = f"{index:03d}_{kind}_preview.json"
            write_json_exclusive(directory / series_name, {
                "scope": "display_rate_snapshots_not_every_imu_sample",
                "series": attempt["series"]})
            item["preview_file"] = series_name
        manifest["attempts"].append(item)
    if bundle.get("profile") is not None:
        write_json_exclusive(directory / "candidate_profile.json", bundle["profile"])
    manifest["session"] = {key: value for key, value in bundle.items()
                           if key not in {"attempts", "neutral_samples", "motion_samples", "direction_samples", "profile"}}
    for path in sorted(directory.iterdir()):
        if path.is_file():
            manifest["files"][path.name] = dict(bytes=path.stat().st_size, sha256=file_sha256(path))
    manifest["export_complete"] = True
    write_json_exclusive(directory / "manifest.json", manifest)
    return str(directory)
