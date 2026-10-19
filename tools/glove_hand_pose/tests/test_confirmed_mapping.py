"""Regression coverage for the user-confirmed 2026-09-28 glove mapping.

Synthetic orientations verify routing and mirroring, not hardware pose accuracy.
"""
import copy
import math

import numpy as np
import pytest

from vt_glove_pose.config import fingerprint, load_config
from vt_glove_pose.kinematics import HandModel
from vt_glove_pose.math3d import from_axis_angle, mul


CONFIRMED_PAIRS = {
    "thumb": (5, 6),
    "index": (3, 4),
    "middle": (1, 2),
    "ring": (7, 8),
    "little": (9, 10),
}


def _q(axis, degrees):
    return from_axis_angle(axis, math.radians(degrees))


def _neutral_orientations(config):
    """HandModel receives the engine's canonical segment orientations."""
    values = {config["palm_node"]: _q([0, 0, 1], 0),
              config["auxiliary_node"]: _q([0, 0, 1], 0)}
    for finger in config["fingers"].values():
        neutral = _q([0, 0, 1], finger["neutral_splay_deg"])
        values[finger["proximal"]] = neutral.copy()
        values[finger["distal"]] = neutral.copy()
    return values


def _solve(config, values):
    return HandModel(config).solve(values, set(values), set(values), True)


def test_default_mapping_keeps_confirmation_and_thumb_anatomy_separate():
    config = load_config()
    assert {name: (item["proximal"], item["distal"])
            for name, item in config["fingers"].items()} == CONFIRMED_PAIRS
    assert all(proximal < distal for proximal, distal in CONFIRMED_PAIRS.values())
    assert config["hand"] == "right"
    assert config["hand_side_confirmed"] is False
    assert config["fingers"]["thumb"]["anatomy_verified"] is False
    assert config["palm_node"] == 12
    assert config["auxiliary_node"] == 11


@pytest.mark.parametrize("hand", ["right", "left"])
@pytest.mark.parametrize("target,proximal,distal", [("index", 3, 4), ("ring", 7, 8)])
def test_only_confirmed_finger_moves_for_each_node_pair(hand, target, proximal, distal):
    config = load_config()
    config["hand"] = hand
    values = _neutral_orientations(config)
    baseline = _solve(config, values)
    # Explicit node IDs ensure a reverted index/ring mapping cannot pass this test.
    values[proximal] = mul(_q([0, 0, 1], 7), _q([-1, 0, 0], 25))
    values[distal] = mul(values[proximal], _q([-1, 0, 0], 40))
    result = _solve(config, values)
    for name, item in result["fingers"].items():
        assert item["valid"]
        if name == target:
            assert item["mcp_deg"] == pytest.approx(25)
            assert item["pip_deg"] == pytest.approx(40)
            assert item["splay_deg"] == pytest.approx(7)
            assert item["dip_deg"] == pytest.approx(26)
            assert not np.allclose(item["points"], baseline["fingers"][name]["points"])
        else:
            np.testing.assert_allclose(item["points"], baseline["fingers"][name]["points"])
            for key in ("mcp_deg", "pip_deg", "ip_deg", "splay_deg", "dip_deg"):
                assert item[key] == pytest.approx(baseline["fingers"][name][key])


def test_left_hand_mirrors_geometry_without_swapping_finger_names():
    config = load_config()
    values = _neutral_orientations(config)
    values[3] = _q([-1, 0, 0], 20)
    values[4] = mul(values[3], _q([-1, 0, 0], 35))
    values[7] = _q([-1, 0, 0], 45)
    values[8] = mul(values[7], _q([-1, 0, 0], 60))
    right = _solve(config, values)
    config["hand"] = "left"
    left = _solve(config, values)
    for name in CONFIRMED_PAIRS:
        np.testing.assert_allclose(left["fingers"][name]["points"],
                                   np.asarray(right["fingers"][name]["points"]) * [-1, 1, 1])
        for key in ("mcp_deg", "pip_deg", "ip_deg", "splay_deg", "dip_deg"):
            assert left["fingers"][name][key] == pytest.approx(right["fingers"][name][key])
    assert left["fingers"]["index"]["pip_deg"] == pytest.approx(35)
    assert left["fingers"]["ring"]["pip_deg"] == pytest.approx(60)


def test_old_mapping_fingerprint_is_not_reused_by_new_mapping():
    config = load_config()
    legacy = copy.deepcopy(config)
    legacy["fingers"]["index"].update(proximal=7, distal=8)
    legacy["fingers"]["ring"].update(proximal=3, distal=4)
    assert fingerprint(config) != fingerprint(legacy)
    confirmed_session = copy.deepcopy(config)
    confirmed_session["hand_side_confirmed"] = True
    assert fingerprint(confirmed_session) == fingerprint(config)
