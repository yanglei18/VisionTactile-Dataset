"""Bounded, optional per-frame glove provenance shared by live and bag previews.

The sender's masks describe this result, not anatomical accuracy or the current
workstation layout. Missing/malformed historical metadata remains unknown.
"""
from collections.abc import Mapping


_MASKS = ("segment_observed_mask", "joint_observed_mask", "inferred_mask")


def glove_sources(value):
    """Copy a message or preview mapping without importing ROS or a solver."""
    get = value.get if isinstance(value, Mapping) else lambda key: getattr(value, key, None)
    masks = {}
    for name in _MASKS:
        raw = get(name)
        if raw is None or not hasattr(raw, "__len__") or len(raw) != 16:
            return {}
        # ROS bool[] can arrive as bools or an array of 0/1 integers.
        if any(type(item) not in (bool, int) or item not in (0, 1) for item in raw):
            return {}
        masks[name] = [bool(item) for item in raw]
    observed = masks["joint_observed_mask"]
    if (not any(masks["segment_observed_mask"])
            or any(joint and (inferred or not segment) for joint, inferred, segment in
                   zip(observed, masks["inferred_mask"], masks["segment_observed_mask"]))):
        return {}
    reference = get("reference_node_id")
    if type(reference) is int and 1 <= reference <= 12:
        masks["reference_node_id"] = reference
    return masks
