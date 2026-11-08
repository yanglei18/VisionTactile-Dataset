from copy import deepcopy

import numpy as np
from scipy.spatial.transform import Rotation
import pytest

from vt_fsglove12.config import digest
from vt_fsglove12.live_display import display_basis
from vt_fsglove12.palm_inclination import capture_up_reference
from vt_hand_tactile_viewer.palm_guide import PalmGuide


NOW = 2_000_000_000
MOUNT = Rotation.from_euler("xyz", [26., -38., 61.], degrees=True)


def sample(pitch, *, yaw=0., index=1):
    orientation = Rotation.from_euler("z", yaw, degrees=True) * Rotation.from_euler("x", pitch, degrees=True) * MOUNT
    return dict(quat_xyzw=orientation.as_quat(), epoch=2, sample_index=index, age_s=.01,
                orientation_valid=True, host_monotonic_ns=NOW - 10_000_000)


def hold(angle, config, key):
    rows = [dict(nodes={"11": sample(angle, yaw=i * 23., index=i)}) for i in range(1, 8)]
    return dict(capture_up_reference(rows, "11", .15), session_id="s", hand=config["hand"],
                node="11", config_sha256=digest(config), clear_generation=1, capture_id=key)


def fixture(hand="left"):
    config = dict(node_to_slot={"11": 0}, hand=hand, stale_after_s=.15)
    axes = dict(hand=hand, roll_axis=[0., 1., 0.], pitch_axis=[1. if hand == "left" else -1., 0., 0.])
    geom = dict(vertices_m=[[.01, .05, 0.]], joints_m=[[0., 0., 0.], [0., .1, 0.]],
                faces=[], provenance=dict(hand=hand))
    refs = {k: hold(a, config, k) for k, a in (("neutral", 0.), ("fingers_down", 47.))}
    snap = dict(runtime=dict(connected=True, session_id="s"),
        calibration=dict(clear_generation=1, palm_tilt_references=refs),
        input_snapshot=dict(runtime=dict(connected=True, session_id="s", clear_generation=1),
            orientation=dict(config_sha256=digest(config), session_id="s", nodes={"11": sample(-42., yaw=109.)})))
    return PalmGuide(dict(config=config, neutral_geometry=geom, model_axes=axes, hand=hand)), snap


@pytest.mark.parametrize("hand", ["left", "right"])
def test_observable_signed_tilt_is_independent_of_mount_and_heading(hand):
    guide, snap = fixture(hand)
    pose = guide.update(snap, now_ns=NOW)
    assert pose.pitch_deg == pytest.approx(-42.)
    assert pose.angle_deg == pytest.approx(42.)
    assert pose.geometry["display_only"] and pose.geometry["provenance"]["hand"] == hand
    basis = display_basis(guide.resources["model_axes"])
    assert (pose.geometry["joints_m"][1] @ basis.T)[2] == pytest.approx(-.1 * np.sin(np.deg2rad(42.)))
    assert np.linalg.norm(pose.geometry["joints_m"][1]) == pytest.approx(.1)
    assert guide.update(snap, now_ns=NOW) is pose


def test_without_up_axis_only_show_measured_angle_and_with_no_reference_no_mesh():
    guide, snap = fixture()
    refs = snap["calibration"]["palm_tilt_references"]
    up = refs.pop("fingers_down")
    p = guide.update(snap, now_ns=NOW)
    assert p.state == "inclination" and p.angle_deg == pytest.approx(42.) and p.geometry is None
    refs["fingers_down"] = dict(up, up=refs["neutral"]["up"])
    assert guide.update(snap, now_ns=NOW).geometry is None
    refs.clear()
    assert guide.update(snap, now_ns=NOW).state == "await_neutral"


def test_session_clear_epoch_source_age_and_hand_cannot_reuse_cached_mesh():
    guide, snap = fixture()
    assert guide.update(snap, now_ns=NOW).geometry
    for path, value in (
        (("runtime", "session_id"), "new"),
        (("input_snapshot", "runtime", "connected"), False),
        (("input_snapshot", "runtime", "clear_generation"), 4),
        (("input_snapshot", "orientation", "nodes", "11", "epoch"), 3),
        (("input_snapshot", "orientation", "nodes", "11", "orientation_valid"), False),
        (("input_snapshot", "orientation", "nodes", "11", "host_monotonic_ns"), NOW-500_000_000),
        (("input_snapshot", "orientation", "config_sha256"), "other"),
        (("calibration", "palm_tilt_references", "neutral", "hand"), "right"),
    ):
        s = deepcopy(snap)
        d = s
        for part in path[:-1]:
            d = d[part]
        d[path[-1]] = value
        assert guide.update(s, now_ns=NOW).geometry is None, path


def test_capture_reference_requires_distinct_same_epoch_stable_observations():
    def ref(nodes):
        return capture_up_reference([dict(nodes={"11": n}) for n in nodes], "11", .15)
    assert ref([sample(0.)] * 8) is None
    assert ref([sample(45., index=i) for i in range(1, 4)] + [dict(sample(45., index=5), epoch=3)]) is None
    assert ref([sample(i * 20., index=i) for i in range(1, 9)]) is None
