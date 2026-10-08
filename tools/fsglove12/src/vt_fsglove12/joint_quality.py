"""Results-only, immediate per-joint validity shared by display and export.

These masks describe estimator validity, not sensor coverage or anatomical
accuracy. Missing fields retain the legacy contract; a partial/malformed new
contract fails closed instead of converting an unavailable joint to measured.
"""
from __future__ import annotations


JOINT_QUALITY_FIELDS = ("joint_valid_mask", "joint_display_available_mask", "joint_quality")
_FLAGS = {"valid": (True, True), "degraded": (False, True), "unavailable": (False, False)}


def has_joint_quality(pose):
    return any(key in pose for key in JOINT_QUALITY_FIELDS)


def joint_quality_state(pose):
    """Return independent lists; never mutate or hysteretically filter results."""
    if not has_joint_quality(pose):
        return [True] * 16, [True] * 16, ["valid"] * 16
    valid, available, quality = (pose.get(key) for key in JOINT_QUALITY_FIELDS)
    if (not all(isinstance(values, (list, tuple)) and len(values) == 16
                for values in (valid, available, quality))
            or any(type(value) is not bool for value in (*valid, *available))
            or any(not isinstance(value, str) or value not in _FLAGS for value in quality)):
        return [False] * 16, [False] * 16, ["unavailable"] * 16
    # Fail closed per slot if a producer supplies contradictory flags.
    states = [state if (v, a) == _FLAGS[state] else "unavailable"
              for v, a, state in zip(valid, available, quality)]
    return ([_FLAGS[state][0] for state in states],
            [_FLAGS[state][1] for state in states], states)
