"""Check the orientation of the *rendered* palm, independently of a camera or UI labels."""
from copy import deepcopy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_upstream_backend import frame_for_anatomical_angles, make_data, official_core
from vt_fsglove12.config import PARENTS, load_config
from vt_fsglove12.guided_calibration import model_motion_axes
from vt_fsglove12.live_display import display_basis
from vt_fsglove12.mano import ManoBackend
from vt_fsglove12.upstream_backend import OfficialFSGloveBackend


@pytest.mark.parametrize("hand,expected_axis", [("left", [-1., 2., -3.]), ("right", [1., 2., 3.])])
def test_model_root_and_quaternion_agree_for_mixed_axis_motion(hand, expected_axis, monkeypatch):
    # The left anatomical wrist points opposite the native X and Z axes,
    # while the Y (vertical) axis and the right rest frame are unchanged.
    rest = Rotation.identity(16).as_quat()
    if hand == "left":
        rest[0] = [0., 1., 0., 0.]
    backend = OfficialFSGloveBackend(load_config(hand=hand),
        {"parents": list(PARENTS), "rest_global_xyzw": rest.tolist(), "provenance": {"hand": hand}},
        checkout=None, mano_assets_root=None, core=object())
    anatomical = .2 * np.array([1., 2., 3.])
    pose = np.random.default_rng(28).normal(0., .1, size=(16, 3))
    pose[0] = anatomical
    raw = dict(pose_available=True, mano_pose_rotvec=pose.tolist(),
               root_quat_xyzw=Rotation.from_rotvec(anatomical).as_quat().tolist(),
               upstream_raw_mano_pose_rotvec=pose.tolist())
    monkeypatch.setattr(backend, "_solve_frame", lambda *args, **kwargs: deepcopy(raw))
    result = backend.solve({}, {})
    expected = Rotation.from_rotvec(.2 * np.array(expected_axis))
    np.testing.assert_allclose(Rotation.from_quat(result["root_quat_xyzw"]).as_matrix(), expected.as_matrix(), atol=1e-7)
    np.testing.assert_allclose(Rotation.from_rotvec(result["mano_pose_rotvec"][0]).as_matrix(), expected.as_matrix(), atol=1e-7)
    np.testing.assert_array_equal(result["mano_pose_rotvec"][1:], pose[1:])
    np.testing.assert_array_equal(result["upstream_raw_mano_pose_rotvec"], pose)


@pytest.mark.parametrize("hand", ["left", "right"])
def test_rendered_palm_follows_the_physical_three_pose_tilts_and_turn(official_core, hand):
    assets = official_core.mano_assets_root
    if not (assets / "models" / f"MANO_{hand.upper()}.pkl").is_file():
        pytest.skip(f"Licensed {hand} MANO model not supplied")
    model = ManoBackend(assets, hand=hand)
    neutral = model.render_mano(np.zeros((16, 3)))
    basis = display_basis(model_motion_axes(neutral["joints_m"], hand=hand))
    backend = OfficialFSGloveBackend(load_config(hand=hand), model.export_rest(),
                                     checkout=None, mano_assets_root=None, core=official_core)
    # The three captures in make_data are rigid physical rotations about world
    # X; world +Y points towards the fingers and +Z is the direction of lift.
    _, captures = make_data(official_core)
    session = "test-session"
    profile = backend.fit_calibration(captures, session_id=session)
    template = captures["base_pose"][0]

    def bend(theta, sequence):
        local = np.zeros((16, 3))
        local[[1, 2], 2] = theta
        frame = frame_for_anatomical_angles(backend, profile, template, local)
        frame["timestamp_s"] = 200. + sequence / 30.
        for node in frame["nodes"].values():
            node["sample_index"] = sequence
        return frame

    neutral_frames = [bend(0., 100 + n) for n in range(6)]
    motion = [bend(angle, 110 + n) for n, angle in enumerate(np.tile(np.linspace(.1, 1.2, 24), 3))]
    entry = backend.fit_functional_finger(neutral_frames, motion, profile, finger="index", session_id=session)
    profile["functional_fingers"] = {"index": entry}
    joints = np.array(neutral["joints_m"])
    # Track independent palm landmarks relative to the wrist (translations,
    # distal finger flexion and the camera cannot change this criterion).
    mcps = [5, 9, 17]
    flat_directions = (joints[mcps] - joints[0]) @ basis.T
    flat_directions /= np.linalg.norm(flat_directions, axis=1)[:, None]
    motions = [(Rotation.identity(), 0.), (Rotation.from_rotvec([.6, 0., 0.]), 0.),
               (Rotation.from_rotvec([-.6, 0., 0.]), 0.), (Rotation.from_euler("xyz", [.5, -.2, .4]), .65)]
    for enabled in (False, True):
        backend.reset_calibration()
        for seq, (physical, index_flexion) in enumerate(motions, 400):
            # A curled index during a compound wrist turn must retain its local flexion.
            sample = bend(index_flexion, seq)
            for node in sample["nodes"].values():
                node["quat_xyzw"] = (physical * Rotation.from_quat(node["quat_xyzw"])).as_quat().tolist()
            pose = backend.solve(sample, {**profile, "functional_enabled": enabled})
            if enabled:
                assert 1 in pose["functional_adapter"]["applied_slots"]
            np.testing.assert_allclose(np.asarray(pose["anatomical_euler_rad"])[[1, 2], 2],
                                       index_flexion, atol=1e-6)
            mano = pose["mano_pose_rotvec"]
            pts = np.asarray(model.render_mano(mano)["joints_m"])
            directions = (pts[mcps] - pts[0]) @ basis.T
            directions /= np.linalg.norm(directions, axis=1)[:, None]
            expected = physical.apply(flat_directions)
            # Finger landmarks of an actual MANO mesh need not coincide exactly
            # with the canonical world axes. Sign reversal gives >60° error.
            angles = np.rad2deg(np.arccos(np.clip(np.sum(expected * directions, axis=1), -1., 1.)))
            assert max(angles) < 6., (hand, enabled, angles)
