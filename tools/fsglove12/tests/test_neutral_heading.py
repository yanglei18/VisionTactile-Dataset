"""An explicit straight-hand observation may correct yaw, never measured tilt."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import PARENTS
from vt_fsglove12.neutral_heading import apply_heading, fit_confirmed_heading


def scene():
    slots = (0, 1, 2)
    config = dict(session_id="session", config_sha256="config", rest_sha256="rest", upstream_state_sha256="state",
                  nodes={str(i): {"slot": i, "epoch": 2} for i in slots})
    model = SimpleNamespace(_base_quat=Rotation.identity(16).as_quat(),
                            _global_installation_error=np.zeros((16, 3)),
                            _local_installation_error=np.zeros((16, 3)),
                            _delta_rot_fa_rotvec=np.zeros((16, 3)),
                            _joint_fa=[0, *PARENTS[1:]], _fk_seq=range(1, 16))
    hand = Rotation.from_euler("XY", [15., 7.], degrees=True)
    drift = Rotation.from_euler("z", [0., 55., -30.], degrees=True)
    def frame(index, rotations):
        return dict(session_id="session", timestamp_s=33. + .1 * index,
                    nodes={str(slot): dict(quat_xyzw=r.as_quat().tolist(), epoch=2,
                                           age_s=.001, sample_index=index + 1) for slot, r in zip(slots, rotations)})
    rows = [frame(i, drift * hand) for i in range(20)]
    return config, model, rows, frame, drift


def test_anchored_heading_preserves_a_subsequent_real_spread_bend_and_wrist_rotation():
    profile, model, samples, frame_for, drift = scene()
    saved = deepcopy(samples)
    anchored = fit_confirmed_heading(samples, profile, model, {}, .15)
    assert samples == saved and "neutral_heading" not in profile
    assert fit_confirmed_heading(samples, anchored, model, {}, .15) == anchored
    assert anchored["neutral_heading"]["offsets_rad_by_node"]["0"] == 0.
    wrist = Rotation.from_euler("ZYX", [67., -28., 32.], degrees=True)
    spread_bend = wrist * Rotation.from_euler("YZ", [20., 60.], degrees=True)
    pip = spread_bend * Rotation.from_euler("z", 54., degrees=True)
    truth = Rotation.from_quat([r.as_quat() for r in (wrist, spread_bend, pip)])
    live = frame_for(21, drift * truth)
    original = deepcopy(live)
    corrected = apply_heading(live, anchored)
    assert live == original and corrected["nodes"]["0"] == live["nodes"]["0"]
    actual = Rotation.from_quat([n["quat_xyzw"] for n in corrected["nodes"].values()])
    np.testing.assert_allclose((truth.inv() * actual).magnitude(), 0., atol=1e-12)
    np.testing.assert_allclose(actual.inv().apply([0., 0., 1.]), (drift * truth).inv().apply([0., 0., 1.]),
                               atol=1e-12)
    assert apply_heading(live, profile) is live


@pytest.mark.parametrize("bad", ["session", "epoch", "missing", "stale", "unsynchronized", "duplicate",
                                  "short", "gap", "moving", "bent", "protected"])
def test_invalid_neutral_window_cannot_replace_an_applied_reference(bad):
    profile, model, rows, _, _ = scene()
    previous = fit_confirmed_heading(rows, profile, model, {}, .15)
    saved = deepcopy(previous)
    for i, row in enumerate(rows):
        data = row["nodes"]["2"]
        if bad == "session":
            row["session_id"] = "another"
        elif bad == "epoch":
            if i == 3:
                data["epoch"] += 1
        elif bad == "missing":
            del row["nodes"]["1"]
        elif bad in ("stale", "unsynchronized"):
            data["age_s"] = .5 if bad == "stale" else .06
        elif bad == "duplicate":
            data["sample_index"] = 1
        elif bad in ("short", "gap"):
            row["timestamp_s"] = i * (.005 if bad == "short" else .5)
        elif bad in ("moving", "bent"):
            angle = 60 if bad == "bent" else i * 2
            data["quat_xyzw"] = (Rotation.from_euler("x", angle, degrees=True)
                                 * Rotation.from_quat(data["quat_xyzw"])).as_quat().tolist()
        elif bad == "protected":
            data["input_filter"] = {"gyro_rejected": True}
    with pytest.raises(ValueError):
        fit_confirmed_heading(rows, previous, model, {}, .15)
    assert previous == saved


def test_saved_reference_cannot_cross_installation_and_stale_nodes_keep_their_invalid_epoch():
    profile, model, rows, _, _ = scene()
    anchored = fit_confirmed_heading(rows, profile, model, {}, .15)
    live = deepcopy(rows[0])
    live["nodes"]["2"]["epoch"] += 1
    assert apply_heading(live, anchored)["nodes"]["2"] == live["nodes"]["2"]
    anchored["upstream_state_sha256"] = "another"
    with pytest.raises(ValueError):
        apply_heading(rows[0], anchored)
