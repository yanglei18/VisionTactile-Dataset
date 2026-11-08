"""Gravity observed in palm-sensor coordinates; no heading or hand pose."""
import numpy as np

from .calibration import quat


def observed_up(node, stale_after_s):
    """Return the fused world-up vector expressed in this physical IMU."""
    if not isinstance(node, dict):
        return None
    guard = node.get("input_filter") or {}
    age = node.get("age_s")
    if (node.get("orientation_valid") is False or guard.get("gyro_rejected") or guard.get("accel_rejected")
            or type(node.get("epoch")) is not int or node["epoch"] < 0
            or type(node.get("sample_index")) is not int or node["sample_index"] < 1
            or type(age) not in (int, float) or not 0 <= age <= stale_after_s):
        return None
    try:
        return quat(node.get("quat_xyzw")).inv().apply([0., 0., 1.])
    except (TypeError, ValueError):
        return None


def capture_up_reference(rows, node_id, stale_after_s):
    """Summarize a committed hold only; never gate or change a calibration."""
    samples = {}
    for row in rows:
        node = row.get("nodes", {}).get(node_id)
        up = observed_up(node, stale_after_s)
        if up is not None:
            samples[(node["epoch"], node["sample_index"])] = up
    if len(samples) < 3 or len({epoch for epoch, _ in samples}) != 1:
        return None
    values = np.asarray(list(samples.values()))
    mean = values.mean(axis=0)
    length = np.linalg.norm(mean)
    # A widely dispersed hold cannot define a stable visual reference.
    if length < 1e-6 or np.percentile(values @ (mean / length), 5) < np.cos(np.deg2rad(10.)):
        return None
    return dict(up=(mean / length).tolist(), epoch=next(iter(samples))[0])
