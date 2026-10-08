import json
import math

import numpy as np
import pytest

from vt_glove_pose.kinematics import HandModel
from vt_glove_pose.math3d import conj, from_axis_angle, from_matrix, mean_quaternion, mul, normalize, rotate, to_matrix


def config(hand="right", **kwargs):
    return {
        "hand": hand, "palm_node": 12, "dip_ratio": 0.65,
        "fingers": {
            "index": {"proximal": 2, "distal": 3, "lengths_mm": [45, 25, 18],
                      "base_mm": [25, 65, 0], "neutral_splay_deg": 8,
                      "anatomy_verified": True},
            "thumb": {"proximal": 0, "distal": 1, "lengths_mm": [35, 25],
                      "base_mm": [38, 16, 0], "neutral_splay_deg": -50,
                      "anatomy_verified": False},
        }, **kwargs,
    }


def q(axis, degrees):
    return from_axis_angle(axis, math.radians(degrees))


def orientations(mcp=30, pip=50, splay=12, common=None):
    proximal = mul(q([0, 0, 1], 8 + splay), q([-1, 0, 0], mcp))
    thumb = mul(q([0, 0, 1], -50), q([-1, 0, 0], 20))
    values = {12: np.array([1, 0, 0, 0]), 2: proximal,
              3: mul(proximal, q([-1, 0, 0], pip)), 0: thumb,
              1: mul(thumb, q([-1, 0, 0], 35))}
    return {node: mul(common, value) for node, value in values.items()} if common is not None else values


def solve(model=None, values=None, valid=None, axes=None, neutral=True):
    values = orientations() if values is None else values
    nodes = set(values)
    return (model or HandModel(config())).solve(values, nodes if valid is None else valid,
                                               nodes if axes is None else axes, neutral)


def test_known_bend_splay_and_inferred_dip():
    result = solve()
    finger = result["fingers"]["index"]
    assert finger["valid"] and finger["quality"] == "ok"
    assert finger["mcp_deg"] == pytest.approx(30)
    assert finger["pip_deg"] == pytest.approx(50)
    assert finger["splay_deg"] == pytest.approx(12)
    assert finger["dip_deg"] == pytest.approx(32.5)
    assert finger["dip_source"] == "model_estimate"
    assert not finger["dip_measured"]
    assert finger["estimated_segments"] == [2]
    points = np.array(finger["points"])
    assert np.linalg.norm(np.diff(points, axis=0), axis=1) == pytest.approx([45, 25, 18])
    assert points[1, 2] < 0
    json.dumps(result, allow_nan=False)


def test_common_rotation_and_quaternion_sign_do_not_change_solution():
    baseline = solve()
    values = orientations(common=q([1, -2, 3], 123))
    for node in (0, 2, 12):
        values[node] *= -1
    result = solve(values=values)
    for name in ("thumb", "index"):
        assert np.array(result["fingers"][name]["points"]) == pytest.approx(np.array(baseline["fingers"][name]["points"]))
        for field in ("mcp_deg", "pip_deg", "ip_deg", "splay_deg"):
            assert result["fingers"][name][field] == pytest.approx(baseline["fingers"][name][field])


def test_left_hand_mirrors_geometry_not_flexion():
    right = solve()
    left = solve(model=HandModel(config("left")))
    for name in right["fingers"]:
        mirrored = np.array(right["fingers"][name]["points"]) * [-1, 1, 1]
        assert np.array(left["fingers"][name]["points"]) == pytest.approx(mirrored)
        assert left["fingers"][name]["mcp_deg"] == right["fingers"][name]["mcp_deg"]
    assert np.array(left["palm_points"]) == pytest.approx(np.array(right["palm_points"]) * [-1, 1, 1])


def test_stale_palm_keeps_pair_angle_but_never_live_geometry():
    finger = solve(valid={0, 1, 2, 3})["fingers"]["index"]
    assert finger["pip_valid"] and finger["pip_deg"] == pytest.approx(50)
    assert finger["mcp_deg"] is None and finger["splay_deg"] is None
    assert not finger["valid"] and not finger["geometry_live"]
    assert finger["quality"] == "palm_stale"
    assert np.array(finger["points"])[:, 2] == pytest.approx([0, 0, 0, 0])


@pytest.mark.parametrize("axes,quality", [({0, 1, 2, 3}, "palm_axes_unverified"), ({0, 1, 12}, "axes_unverified")])
def test_axes_are_separate_from_neutral(axes, quality):
    finger = solve(axes=axes)["fingers"]["index"]
    assert not finger["valid"]
    assert finger["quality"] == quality
    assert np.array(finger["points"])[:, 2] == pytest.approx([0, 0, 0, 0])
    assert finger["pip_valid"] == (quality == "palm_axes_unverified")


def test_no_neutral_and_stale_distal_never_reuse_last_frame():
    model = HandModel(config())
    assert solve(model=model)["fingers"]["index"]["valid"]
    for result, quality in ((solve(model=model, neutral=False), "no_neutral"),
                            (solve(model=model, valid={0, 1, 2, 12}), "stale")):
        finger = result["fingers"]["index"]
        assert not finger["valid"] and finger["pip_deg"] is None
        assert finger["quality"] == quality
        assert np.array(finger["points"])[:, 2] == pytest.approx([0, 0, 0, 0])


def test_unmapped_and_invalid_quaternion_are_explicit():
    settings = config()
    settings["fingers"]["index"]["distal"] = None
    finger = solve(model=HandModel(settings))["fingers"]["index"]
    assert not finger["valid"] and finger["quality"] == "unmapped"
    values = orientations()
    values[3] = np.array([np.nan, 0, 0, 0])
    finger = solve(values=values)["fingers"]["index"]
    assert not finger["valid"] and finger["quality"] == "invalid_orientation"
    assert finger["pip_deg"] is None


def test_thumb_has_two_bones_and_explicitly_provisional_ip():
    thumb = solve()["fingers"]["thumb"]
    assert len(thumb["points"]) == 3
    assert thumb["ip_deg"] == pytest.approx(35)
    assert thumb["pip_deg"] is None and thumb["dip_deg"] is None
    assert thumb["anatomical_provisional"]
    assert thumb["estimated_segments"] == []


def test_disabled_dip_is_not_reported_as_measurement():
    finger = solve(model=HandModel(config(estimate_dip=False)))["fingers"]["index"]
    assert finger["dip_deg"] is None and not finger["dip_valid"]
    assert finger["dip_source"] == "disabled"
    assert finger["estimated_segments"] == [2]
    points = np.array(finger["points"])
    assert np.diff(points, axis=0)[1] / 25 == pytest.approx(np.diff(points, axis=0)[2] / 18)


def test_out_of_range_angles_are_flagged_and_clipped():
    finger = solve(values=orientations(mcp=-40, pip=145, splay=50))["fingers"]["index"]
    assert finger["quality"] == "clipped" and finger["warnings"]
    assert finger["mcp_deg"] == -20 and finger["pip_deg"] == 120
    assert finger["splay_deg"] == 35
    assert finger["raw_angles_deg"]["pip_deg"] == pytest.approx(145)


def test_quaternion_rotation_matrix_and_inverse():
    rotation = q([1, 2, -3], 71)
    assert to_matrix(from_matrix(to_matrix(rotation))) == pytest.approx(to_matrix(rotation))
    assert mul(conj(rotation), rotation) == pytest.approx([1, 0, 0, 0])
    assert rotate(q([0, 0, 1], 90), [1, 0, 0]) == pytest.approx([0, 1, 0])
    for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 2, 3]):
        rotation = q(axis, 180)
        assert to_matrix(from_matrix(to_matrix(rotation))) == pytest.approx(to_matrix(rotation))


def test_quaternion_mean_is_sign_invariant_and_normalization_robust():
    rotation = q([1, -2, 3], 80)
    assert to_matrix(mean_quaternion([rotation, -rotation, rotation * 7])) == pytest.approx(to_matrix(rotation))
    assert normalize([1e308, 1e308, 0, 0]) == pytest.approx([2**-0.5, 2**-0.5, 0, 0])
    assert normalize([1e-300, 0, 0, 0]) == pytest.approx([1, 0, 0, 0])


@pytest.mark.parametrize("value", [[0, 0, 0, 0], [float("nan"), 0, 0, 0], [1, 0, 0], [1, float("inf"), 0, 0]])
def test_bad_quaternions_are_rejected(value):
    with pytest.raises(ValueError):
        normalize(value)


@pytest.mark.parametrize("matrix", [np.diag([-1, 1, 1]), np.eye(3) * 2, np.zeros((4, 4)), np.full((3, 3), np.nan)])
def test_only_proper_rotation_matrices_are_accepted(matrix):
    with pytest.raises(ValueError):
        from_matrix(matrix)


def test_empty_mean_and_zero_axis_rejected():
    with pytest.raises(ValueError):
        mean_quaternion([])
    with pytest.raises(ValueError):
        from_axis_angle([0, 0, 0], 0)
