"""Read-only evidence from a running hand, with explicit replay limitations.

Only documents whose canonical hashes match the child's active bindings are
returned. Reading the configured resource paths alone is not evidence that
those bytes are in use. No models are deserialized and no device is opened.
"""
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import time

MAX_ARTIFACT_BYTES = 1024 * 1024


def _digest(document):
    return hashlib.sha256(json.dumps(document, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def software_binding(root):
    """Capture implementation bytes once when the service is constructed."""
    root = Path(root).resolve()
    checksum = hashlib.sha256()
    for path in sorted(root.rglob("*.py")):
        checksum.update(path.relative_to(root).as_posix().encode() + b"\0")
        checksum.update(hashlib.sha256(path.read_bytes()).digest())
    return dict(package_root=str(root), source_sha256=checksum.hexdigest(),
                scope="Hand service startup source bytes; dependency binaries are not archived")


def _artifact(path, expected):
    if not isinstance(expected, str) or len(expected) != 64:
        raise ValueError("Running child did not publish an active artifact hash")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(descriptor, "rb") as stream:
        before = os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode) or before.st_size > MAX_ARTIFACT_BYTES:
            raise ValueError("Active artifact must be a bounded regular JSON file")
        raw = stream.read(MAX_ARTIFACT_BYTES + 1)
        after = os.fstat(stream.fileno())
    if len(raw) > MAX_ARTIFACT_BYTES or (before.st_size, before.st_mtime_ns, before.st_ctime_ns) != (
            after.st_size, after.st_mtime_ns, after.st_ctime_ns):
        raise ValueError("Active artifact changed during observation")
    document = json.loads(raw)
    if not isinstance(document, dict) or _digest(document) != expected:
        raise ValueError("Artifact bytes do not match the running child's active hash")
    return dict(sha256=expected, hash_semantics="canonical_json_sha256", document=document)


def collect_hand_evidence(snapshot, *, hand, service_instance_id, software, include_artifacts=True):
    """Freeze source indices and runtime bindings without claiming exact edges."""
    runtime = snapshot.get("runtime") or {}
    input_snapshot = snapshot.get("input_snapshot") or {}
    input_runtime = input_snapshot.get("runtime") or runtime
    orientation = input_snapshot.get("orientation") or snapshot.get("orientation") or {}
    calibration = snapshot.get("calibration") or {}
    pose = snapshot.get("pose") or {}
    session = input_runtime.get("session_id")
    published = input_runtime.get("published_monotonic_s")
    producer_stamp = (int(published * 1_000_000_000) if type(published) in (int, float)
                      and math.isfinite(published) and published > 0 else 0)
    result = dict(schema_version=1, status="verified_snapshot", hand=hand,
        service_instance_id=service_instance_id, session_id=session or "",
        producer_observed_monotonic_ns=producer_stamp, service_observed_monotonic_ns=time.monotonic_ns(),
        boundary_exact=False,
        boundary_semantics="Asynchronous producer observations; not synchronized recorder edges or exact trial counts",
        profile_version=calibration.get("profile_version"),
        config_sha256=calibration.get("config_sha256", ""),
        rest_sha256=calibration.get("rest_sha256", ""),
        profile_sha256=calibration.get("profile_sha256", ""),
        upstream_state_sha256=calibration.get("upstream_state_sha256", ""),
        input_kind=orientation.get("input_kind", "unknown"),
        source_mode=("live" if orientation.get("input_kind") == "raw_serial" else
                     "replay" if orientation.get("input_kind") == "csv_replay" else "unknown"),
        software=deepcopy(software), source_indices={}, artifacts={},
        producer_counters=dict(received_samples=input_runtime.get("received_samples"),
            ros_raw_export=deepcopy(input_runtime.get("ros_raw_export") or {}),
            ros_export=deepcopy(runtime.get("ros_export") or {})),
        replay_scope=dict(vqf_state_captured=False, exact_raw_replay_from_trial_start=False,
            detail="Warm VQF state and producer/recorder boundary synchronization are not captured. "
                   "Use recorded fused pose directly; raw replay needs prehistory or an explicit cold-start scope."))
    problems = []
    if not session or runtime.get("session_id") != session or orientation.get("session_id") != session:
        problems.append("Pose/input/session bindings are missing or disagree")
    if input_runtime.get("connected") is not True:
        problems.append("Producer is not connected")
    if result["source_mode"] == "unknown":
        problems.append("Producer source mode is unknown")
    if not producer_stamp:
        problems.append("Producer snapshot observation clock is unavailable")
    if orientation.get("config_sha256") != result["config_sha256"]:
        problems.append("Input and calibration configuration identities disagree")
    if pose.get("profile_version") is not None and pose["profile_version"] != result["profile_version"]:
        problems.append("Pose and calibration profile versions disagree")
    for node, row in (orientation.get("nodes") or {}).items():
        result["source_indices"][str(node)] = dict(sample_index=row.get("sample_index"),
            filter_epoch=row.get("epoch"), host_monotonic_ns=row.get("host_monotonic_ns"),
            host_realtime_ns=row.get("host_realtime_ns"))
    if not result["source_indices"] or any(
            type(row.get(key)) is not int or row[key] < (0 if key == "filter_epoch" else 1)
            for row in result["source_indices"].values()
            for key in ("sample_index", "filter_epoch", "host_monotonic_ns", "host_realtime_ns")):
        problems.append("Producer sensor identities are unavailable")
    if include_artifacts:
        directory = Path(runtime.get("output_dir") or "")
        paths = {"config": directory / "config.json", "rest": directory / "rest.json"}
        if calibration.get("profile_artifact_path"):
            paths["profile"] = Path(calibration["profile_artifact_path"])
        else:
            problems.append("Running child has no saved active profile path")
        for name, path in paths.items():
            try:
                result["artifacts"][name] = _artifact(path, result[name + "_sha256"])
            except (OSError, ValueError, UnicodeError) as exc:
                problems.append(f"{name}: {exc}")
    if problems:
        result.update(status="unavailable", problems=problems)
    return result
