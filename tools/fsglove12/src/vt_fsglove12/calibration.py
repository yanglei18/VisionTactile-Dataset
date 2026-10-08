"""Session-bound mounting/frame calibration from explicitly labelled poses.

This is an adapter algorithm, NOT FSGlove's unmodified DiffHCal. For each
observed bone it solves Y * R_sensor(t) * X = R_label(t). X is the constant
sensor-from-bone mounting rotation; Y aligns that filter's initial heading to
the labelled model frame. Calibration cannot prevent subsequent 6-axis yaw
drift. Label fit is not an independent accuracy measurement.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation

from .config import PARENTS, check_model_hand, check_proposed_mount, digest, validate_config


def quat(value, name="quaternion") -> Rotation:
    array = np.asarray(value, dtype=float)
    if array.shape != (4,) or not np.isfinite(array).all() or abs(np.linalg.norm(array) - 1) > .01:
        raise ValueError(f"{name} must be a normalized finite [x,y,z,w] quaternion")
    return Rotation.from_quat(array)


def validate_rest(rest: dict) -> np.ndarray:
    if not isinstance(rest, dict) or rest.get("parents") != list(PARENTS):
        raise ValueError("rest model must contain the 16-slot MANO parent hierarchy")
    rotations = np.asarray(rest.get("rest_global_xyzw"), dtype=float)
    if rotations.shape != (16, 4):
        raise ValueError("rest_global_xyzw must have shape (16,4)")
    for slot, q in enumerate(rotations):
        quat(q, f"rest slot {slot}")
    return rotations


def _validated_poses(manifest):
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        raise ValueError("calibration manifest requires schema_version=1")
    if manifest.get("time_origin") != "first_frame":
        raise ValueError("manifest time_origin must explicitly be first_frame")
    poses = manifest.get("poses")
    if not isinstance(poses, list) or len(poses) < 3:
        raise ValueError("at least three labelled rigid-hand poses are needed")
    previous_end = -math.inf
    result = []
    for index, pose in enumerate(poses):
        if not isinstance(pose, dict):
            raise ValueError("each calibration pose must be an object")
        start, end = pose.get("start_s"), pose.get("end_s")
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (start, end)):
            raise ValueError("pose intervals must be finite numbers")
        if not 0 <= start < end or start < previous_end:
            raise ValueError("pose intervals must be ordered, nonoverlapping and positive")
        previous_end = end
        result.append((float(start), float(end), quat(pose.get("root_quat_xyzw"), f"pose {index} root")))
    return result


def _fit(sensor_rotations: Rotation, target_rotations: Rotation):
    # From A0 X and B0, derive A0^-1 At X = X B0^-1 Bt.
    a = (sensor_rotations[0].inv() * sensor_rotations[1:]).as_rotvec()
    b = (target_rotations[0].inv() * target_rotations[1:]).as_rotvec()
    for values in (a, b):
        singular = np.linalg.svd(values, compute_uv=False)
        if singular[0] < np.deg2rad(10) or singular[1] < .01 * singular[0]:
            raise ValueError("poses need rotations about two distinct axes; one axis cannot identify the mount")
    sensor_from_bone, _ = Rotation.align_vectors(a, b)
    alignments = target_rotations * (sensor_rotations * sensor_from_bone).inv()
    world_alignment = alignments.mean()
    prediction = world_alignment * sensor_rotations * sensor_from_bone
    residual = np.rad2deg((target_rotations.inv() * prediction).magnitude())
    return sensor_from_bone, world_alignment, residual


def calibrate(frames, manifest: dict, rest: dict, config: dict, *, stream_sha256: str) -> dict:
    """Fit only physically mapped nodes; no virtual quaternion is calibrated.

    ``manifest`` labels rigid, neutral-hand intervals by commanded root
    rotations. All intervals must belong to the SAME uninterrupted filter
    epoch for each fitted node. Missing/bad finger nodes are reported locally;
    an absent palm cannot produce a usable whole-hand profile.
    """
    config = validate_config(config)
    check_model_hand(config, rest)
    check_proposed_mount(config)
    rest_rotations = Rotation.from_quat(validate_rest(rest))
    poses = _validated_poses(manifest)
    if not isinstance(stream_sha256, str) or len(stream_sha256) != 64:
        raise ValueError("stream_sha256 is required to bind the independent filter headings")
    bins = {node: [[] for _ in poses] for node in config["node_to_slot"]}
    epochs = {node: set() for node in bins}
    first_time = None
    frame_count = 0
    for frame in frames:
        timestamp = float(frame["timestamp_s"])
        if not math.isfinite(timestamp):
            raise ValueError("frame timestamp must be finite")
        if first_time is None:
            first_time = timestamp
        frame_count += 1
        relative_time = timestamp - first_time
        matching = next((index for index, (start, end, _) in enumerate(poses) if start <= relative_time < end), None)
        if matching is None:
            continue
        for node, data in frame["nodes"].items():
            if node not in bins:
                continue
            protection = data.get("input_filter", {})
            if (data.get("orientation_valid") is False or protection.get("gyro_rejected")
                    or protection.get("accel_rejected")):
                continue
            age = float(data.get("age_s", math.inf))
            if not math.isfinite(age) or not 0 <= age <= config["stale_after_s"]:
                continue
            epoch = data.get("epoch")
            if type(epoch) is not int or epoch < 0:
                raise ValueError("orientation frames must include nonnegative filter epochs")
            rotation = quat(data["quat_xyzw"], f"node {node}")
            bins[node][matching].append(rotation.as_quat())
            epochs[node].add(epoch)
    if not frame_count:
        raise ValueError("orientation stream is empty")
    nodes, rejected, warnings = {}, {}, []
    for node, slot in config["node_to_slot"].items():
        try:
            if len(epochs[node]) != 1:
                raise ValueError("calibration crosses a filter reset or contains no samples")
            counts = [len(rows) for rows in bins[node]]
            if min(counts) < 3:
                raise ValueError(f"each labelled pose needs at least 3 fresh frames; got {counts}")
            samples = [Rotation.from_quat(rows) for rows in bins[node]]
            means = Rotation.from_quat([sample.mean().as_quat() for sample in samples])
            target = Rotation.from_quat([(root * rest_rotations[slot]).as_quat() for _, _, root in poses])
            mount, alignment, errors = _fit(means, target)
            within_pose_p95 = [float(np.percentile(np.rad2deg((sample.mean().inv() * sample).magnitude()), 95))
                               for sample in samples]
            rms = float(np.sqrt(np.mean(errors ** 2)))
            # Warn instead of demanding a perfect human pose. Consumers see
            # the measured fit and provisional status, not a false PASS.
            if rms > 5 or max(within_pose_p95) > 5:
                warnings.append(f"node {node}: label fit or pose motion exceeds 5 degrees; review before interpreting joint angles")
            nodes[node] = {
                "slot": slot, "epoch": next(iter(epochs[node])),
                "sensor_from_bone_xyzw": mount.as_quat().tolist(),
                "world_alignment_xyzw": alignment.as_quat().tolist(),
                "label_fit_rms_deg": rms, "label_fit_per_pose_deg": errors.tolist(),
                "within_pose_p95_deg": within_pose_p95, "frames_per_pose": counts,
            }
        except ValueError as exc:
            rejected[node] = str(exc)
    palm_node = next(node for node, slot in config["node_to_slot"].items() if slot == 0)
    if palm_node not in nodes:
        raise ValueError(f"palm node {palm_node} could not be calibrated: {rejected.get(palm_node)}")
    return {
        "schema_version": 1, "method": "labelled_rigid_pose_hand_eye_adapter_v1",
        "status": "provisional_not_accuracy_validated", "hand": config["hand"],
        "config_sha256": digest(config), "rest_sha256": digest(rest),
        "stream_sha256": stream_sha256, "stream_first_timestamp_s": first_time,
        "stale_after_s": config["stale_after_s"], "manifest": manifest,
        "nodes": nodes, "rejected_nodes": rejected, "warnings": warnings,
        "limitations": ["Commanded poses are labels, not independently measured ground truth.",
                        "Valid only for this replay/filter run and matching node epochs.",
                        "Initial heading alignment does not correct subsequent relative heading drift.",
                        "Unmeasured bones are never calibrated using synthetic data."],
    }


def apply_calibration(frame: dict, profile: dict) -> tuple[dict, dict]:
    """Return current bone orientations keyed by node; omit invalid nodes."""
    observations, rejected = {}, {}
    for node, params in profile["nodes"].items():
        data = frame["nodes"].get(node)
        if data is None:
            rejected[node] = "missing_or_stale"
            continue
        if data.get("orientation_valid") is False:
            rejected[node] = "input_filter_prolonged_invalid"
            continue
        if data.get("epoch") != params["epoch"]:
            rejected[node] = "filter_epoch_changed_recalibration_required"
            continue
        age = float(data.get("age_s", math.inf))
        if not math.isfinite(age) or not 0 <= age <= profile["stale_after_s"]:
            rejected[node] = "stale"
            continue
        sensor = quat(data["quat_xyzw"], f"node {node}")
        orientation = quat(params["world_alignment_xyzw"]) * sensor * quat(params["sensor_from_bone_xyzw"])
        observations[node] = orientation.as_quat().tolist()
    return observations, rejected


def check_profile_binding(profile: dict, config: dict, rest: dict, stream_sha256: str):
    check_model_hand(config, rest)
    if profile.get("hand", "right") != config["hand"]:
        raise ValueError("calibration hand does not match the configured hand")
    for field, value in (("config_sha256", digest(config)), ("rest_sha256", digest(rest)),
                         ("stream_sha256", stream_sha256)):
        if profile.get(field) != value:
            raise ValueError(f"calibration {field} does not match: do not reuse independent filter headings across sessions")
