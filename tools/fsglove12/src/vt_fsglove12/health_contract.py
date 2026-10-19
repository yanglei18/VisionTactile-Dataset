"""Small results-only input-health contract shared by GUI and ROS export.

Arrival timestamps and increasing counters do not establish fresh sensor data.
Never export raw payloads, signatures or status bytes in these diagnostics.
"""
from __future__ import annotations

from copy import deepcopy


def blocked_input_nodes(frame, mapped=None):
    health = (frame or {}).get("input_health") or {}
    if not health.get("enabled"):
        return []
    nodes = set(health.get("faulted_nodes", ())) | set(health.get("recovering_nodes", ()))
    nodes.update(str(node) for node, item in health.get("nodes", {}).items()
                 if item.get("healthy") is False)
    nodes = {str(node) for node in nodes}
    if mapped is not None:
        nodes.intersection_update(map(str, mapped))
    return sorted(nodes, key=lambda node: int(node))


def calibration_input_ready(frame, config):
    """Absence of a required report is not evidence of a healthy source."""
    if not config.get("input_health", {}).get("enabled", False):
        return True
    health = (frame or {}).get("input_health") or {}
    return bool(health.get("enabled") and all(
        health.get("nodes", {}).get(node, {}).get("state") == "healthy"
        and health["nodes"][node].get("healthy") is True
        and (frame or {}).get("nodes", {}).get(node, {}).get("orientation_valid") is not False
        and node in (frame or {}).get("nodes", {})
        for node in config["node_to_slot"]))


def result_input_health(frame):
    """Whitelist diagnostics so wire captures can never leak into ROS JSON."""
    health = (frame or {}).get("input_health") or {}
    if not health.get("enabled"):
        return {"enabled": False}
    fields = ("healthy", "faulted", "state", "reasons", "observed_receive_rate_hz",
              "nominal_sample_rate_hz")
    return dict(enabled=True, faulted_nodes=list(health.get("faulted_nodes", ())),
                recovering_nodes=list(health.get("recovering_nodes", ())),
                nodes={str(node): {key: deepcopy(item[key]) for key in fields if key in item}
                       for node, item in health.get("nodes", {}).items()},
                semantics="receive_health_not_device_odr_or_absolute_pose_accuracy")
