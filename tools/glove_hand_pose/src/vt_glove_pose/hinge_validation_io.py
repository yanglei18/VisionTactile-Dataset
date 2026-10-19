"""Analyze actual recorded online angles or explicitly replay a marked capture."""
from __future__ import annotations

import json
import math
from pathlib import Path

from .hinge_export import file_sha256
from .hinge_replay import _profile, replay_hinge_pair
from .pair_capture import load_pair_capture


def _read_json(path, limit):
    if path.stat().st_size > limit:
        raise ValueError(f"JSON exceeds {limit} bytes: {path}")
    with path.open(encoding="utf-8") as source:
        value = json.load(source)
    if not isinstance(value, dict):
        raise ValueError("expected a JSON object")
    return value


def _local_file(directory, name):
    if not isinstance(name, str) or not name or name in {".", ".."} or Path(name).name != name or "\\" in name:
        raise ValueError("protocol capture/profile must be filenames in the protocol directory")
    return directory/name


def validate_hinge_protocol(path, *, observer=None, source="auto", max_pairs=30000, byte_limit=None):
    """Pose repeatability/stability is not absolute accuracy or physical latency."""
    from .hinge_validation import analyze_hinge_validation
    if source not in {"auto", "online", "replay"}:
        raise ValueError("source must be auto, online or replay")
    path = Path(path)
    protocol = _read_json(path, 1024*1024)
    if protocol.get("kind") != "vt_glove_pose.hinge_validation_protocol" or protocol.get("schema_version") != 1:
        raise ValueError("expected a hinge_validation_protocol schema_version=1")
    if protocol.get("time_basis") != "seconds_from_first_csv_host_parse_time_not_physical_motion_time":
        raise ValueError("unsupported validation window time basis")
    mode = ("online" if protocol.get("online_file") else "replay") if source == "auto" else source
    if mode == "online" and not protocol.get("online_file"):
        raise ValueError("protocol has no full-rate online output; use --source replay explicitly for older recordings")
    if mode == "online" and observer is not None:
        raise ValueError("--observer applies to --source replay; recorded online outputs cannot change algorithms")
    capture_path = _local_file(path.parent, protocol.get("capture_file"))
    profile_path = _local_file(path.parent, protocol.get("profile_file"))
    if capture_path.stat().st_size > 64*1024*1024:
        raise ValueError("validation CSV exceeds 64 MiB; split explicitly into smaller experiments")
    profile = _read_json(profile_path, 4*1024*1024)
    hashes = dict(capture=file_sha256(capture_path), profile=file_sha256(profile_path))
    verified = {}
    for label in ("capture", "profile"):
        expected = protocol.get(label+"_sha256")
        verified[label] = expected is not None and expected == hashes[label]
        if expected is not None and not verified[label]:
            raise ValueError(f"{label} SHA256 differs from the exported protocol")
    settings, _, _ = _profile(profile)
    capture = load_pair_capture(capture_path, proximal=settings["node_ids"][0], distal=settings["node_ids"][1],
        sample_rate_hz=settings["sample_rate_hz"], gyro_dps_per_lsb=settings["gyro_dps_per_lsb"],
        gyro_dps_per_lsb_by_node=settings.get("gyro_dps_per_lsb_by_node"),
        max_pairs=max_pairs, byte_limit=byte_limit)
    trace_path = None
    if mode == "online":
        from .hinge_online import online_hinge_report
        trace_path = _local_file(path.parent, protocol["online_file"])
        trace = _read_json(trace_path, 64*1024*1024)
        hashes["online"] = file_sha256(trace_path)
        expected = protocol.get("online_sha256")
        verified["online"] = expected is not None and expected == hashes["online"]
        if expected is not None and not verified["online"]:
            raise ValueError("online trace SHA256 differs from the exported protocol")
        if trace.get("capture_sha256") != hashes["capture"] or trace.get("profile_sha256") != hashes["profile"]:
            raise ValueError("online trace is not bound to this raw CSV and frozen profile")
        replay = online_hinge_report(trace, capture, settings)
    else:
        replay = replay_hinge_pair(profile, capture, observer=observer or "constrained")
    windows = protocol.get("windows")
    if not isinstance(windows, list) or len(windows) > 256:
        raise ValueError("protocol windows must be a list with at most 256 entries")
    offset = 0.
    if len(capture["host_monotonic_s"]):
        offset = float(capture["host_monotonic_s"][0, 0])-capture["metadata"]["first_input_host_monotonic_s"]
    shifted = []
    for original in windows:
        if not isinstance(original, dict):
            raise ValueError("each validation window must be an object")
        window = dict(original)
        for key in ("start_s", "end_s"):
            value = window.get(key)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError("window start_s/end_s must be finite numbers")
            window[key] = value-offset
        shifted.append(window)
    report = analyze_hinge_validation(replay, shifted)
    report["provenance"] = dict(protocol=str(path), capture=str(capture_path), profile=str(profile_path),
        protocol_sha256=file_sha256(path), source_sha256=hashes, exported_hash_verified=verified,
        profile_settings=settings, capture_status=protocol.get("capture_status"),
        csv_first_to_first_pair_s=offset, source=capture["metadata"],
        windows_input_time_basis=protocol["time_basis"],
        windows_report_time_basis="seconds_from_first_matched_pair_host_parse_time",
        replay_status=replay["replay_status"], replay_summary=replay["summary"],
        observer=replay["observer"], direction=protocol.get("direction"),
        evaluation_source=mode, online_file=str(trace_path) if trace_path else None,
        observer_state_basis=("actual_recorded_per_pair_online_output" if mode == "online" else
                              "offline_reinitialized_from_frozen_profile_and_first_pair_not_live_state_checkpoint"))
    report.setdefault("limitations", []).append(
        "本报告评估保存的实际在线逐配对输出，保留当时状态产生的角度，不重新初始化、插值或用离线结果填补缺失；这仍不是独立角度真值或显示端延迟测量。"
        if mode == "online" else
        "本报告从录制时的候选和验证首帧重新初始化观察器，不恢复实时观察器已累计的偏置等内部状态；它不是实时输出的逐样本复现。")
    return report
