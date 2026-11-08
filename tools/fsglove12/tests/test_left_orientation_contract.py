"""Opt-in LEFT 12-node -> sparse FSGlove -> QMT -> MANO orientation contract.

Known joint kinematics generate sensor observations with independent constant
headings and mounts. This covers software direction invariance, not physical
glove accuracy or time-varying bias. Explicit source/model opt-in is required.
"""
from copy import deepcopy
from importlib.resources import files
from pathlib import Path
import os

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_installation_frame import poses
from test_installation_frame_upstream import cumulative_rest
from test_upstream_backend import official_core
from vt_fsglove12.config import PARENTS, load_config
from vt_fsglove12.joint_quality import joint_quality_state
from vt_fsglove12.mano import ManoBackend
from vt_fsglove12.upstream_backend import OfficialFSGloveBackend, STEP_KEYS
from vt_fsglove12.upstream_functional import FINGER_SLOTS


class SyntheticLeftHand:
    """Forward physical kinematics; never invert fitted installation errors."""
    def __init__(self, config, mount, heading, rest, axis):
        self.config, self.mount, self.heading, self.rest = config, mount, heading, rest
        self.virtual = Rotation.from_matrix(np.stack(
            [np.cross([0., 0., 1.], axis), [0., 0., 1.], axis], axis=1))

    def packet(self, raw, sequence):
        nodes = {node: dict(quat_xyzw=raw[slot].as_quat().tolist(), age_s=.005,
                            epoch=0, sample_index=sequence)
                 for node, slot in self.config["node_to_slot"].items()}
        # An intentionally unrelated but fresh accessory reading ensures it
        # cannot accidentally become the palm/reference after the permutation.
        nodes["12"] = dict(quat_xyzw=[0., 0., 0., 1.], age_s=0., epoch=0,
                           sample_index=sequence + 100)
        return dict(timestamp_s=1000. + sequence / 30., session_id="left-orientation",
                    nodes=nodes)

    def frame(self, sequence, root, local):
        segments = [root]
        for slot, parent in enumerate(PARENTS[1:], 1):
            delta = self.rest[parent].inv() * self.rest[slot]
            segments.append(segments[parent] * delta * local[slot])
        increment = (self.virtual * Rotation.concatenate(segments)
                     * self.rest.inv() * self.virtual.inv())
        return self.packet(self.heading * increment * self.mount, sequence)


def finger_rotations(*, fingers=0., thumb=0., neutral=False):
    values = [Rotation.identity() for _ in range(16)]
    reference = Rotation.from_euler("XYZ", [.08, -.03, .06])
    for finger, slots in FINGER_SLOTS.items():
        for slot in slots:
            basis = Rotation.from_rotvec([.15 + slot * .03, -.22, 0.])
            angle = 0. if neutral else (thumb if finger == "thumb" else fingers * (1. + slot * .01))
            values[slot] = reference * basis * Rotation.from_rotvec([0., 0., angle]) * basis.inv()
    return Rotation.concatenate(values)


@pytest.fixture(scope="module")
def left_sparse_data(official_core):
    if not (Path(os.environ["FSGLOVE12_MANO_ASSETS"]) / "models/MANO_LEFT.pkl").is_file():
        pytest.skip("Licensed left-hand MANO model not supplied")
    model = ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"], hand="left")
    config = load_config(files("vt_fsglove12").joinpath("configs/left_from_right_initial.json"))
    backend = OfficialFSGloveBackend(config, model.export_rest(), checkout=None,
                                    mano_assets_root=None, core=official_core)
    axis = np.array([-.6, .8, 0.])
    mount, heading, inputs = poses(axis, np.linspace(-2.8, 2.9, 16))
    source = SyntheticLeftHand(config, mount, heading, None, axis)
    captures = {key: [source.packet(Rotation.from_quat(raw), 1 + step * 10 + index)
                      for index in range(5)] for step, (key, raw) in enumerate(zip(STEP_KEYS, inputs))}
    profile = backend.fit_calibration(captures, session_id="left-orientation")
    assert len(profile["measured_calibration_slots"]) == 11
    assert "12" not in profile["nodes"]
    assert not profile["rejected_nodes"]
    source.rest = cumulative_rest(backend._calibrator(profile))
    return backend, profile, source, model


@pytest.fixture(scope="module")
def left_functional_data(left_sparse_data):
    pytest.importorskip("qmt")
    backend, profile, source, model = left_sparse_data
    neutral = [source.frame(100 + i, Rotation.identity(), finger_rotations(neutral=True))
               for i in range(6)]
    cycle = np.tile(np.r_[np.linspace(.05, .95, 6), np.linspace(.95, .05, 6)], 2)
    fingers = [source.frame(110 + i, Rotation.identity(), finger_rotations(fingers=float(value)))
               for i, value in enumerate(cycle)]
    thumb = [source.frame(160 + i, Rotation.identity(), finger_rotations(thumb=float(value)))
             for i, value in enumerate(cycle)]
    fit = backend.fit_functional_hand(neutral, fingers, thumb, profile,
                                       session_id=profile["session_id"])
    assert fit["errors"] == {}, fit["errors"]
    assert set(fit["entries"]) == set(FINGER_SLOTS)
    assert all(set(entry["slot_refs"]) == {str(slot) for slot in FINGER_SLOTS[finger]}
               for finger, entry in fit["entries"].items())
    return backend, {**deepcopy(profile), "functional_enabled": True,
                     "functional_fingers": fit["entries"]}, source, model


def local_output(result):
    assert result["pose_available"] and result["solver_success"]
    assert result["hand"] == "left"
    assert not result["calibration_rejections"]
    display = joint_quality_state(result)[1]
    assert all(display[slot] for slot in range(13))
    # In the standalone viewer the uninstrumented first metacarpal is a
    # fixed prior. The measured proximal segment drives the displayed joint.
    assert display[13] and display[14]
    assert not any(result["joint_observed_mask"][13:15])
    assert result["source_labels"][14] == "official_measured_segment_with_inferred_parent"
    anatomy = Rotation.from_euler("XYZ", np.asarray(result["anatomical_euler_rad"])[1:])
    mano = np.asarray(result["mano_pose_rotvec"]).copy()
    mano[0] = 0.
    return anatomy, mano


@pytest.mark.parametrize("functional", [False, True])
@pytest.mark.parametrize("bent", [False, True])
def test_left_sparse_and_functional_outputs_ignore_whole_hand_direction(
        request, left_sparse_data, functional, bent):
    data = request.getfixturevalue("left_functional_data") if functional else left_sparse_data
    backend, profile, source, model = data
    shape = finger_rotations(fingers=.55, thumb=.45) if bent else finger_rotations(neutral=True)
    expected = None
    for root in ([0., 0., 0.], [.2, -.4, .3], [1.4, -.8, 2.0], [-1., .7, -.8]):
        # Equal fresh owner/history at each direction: no hidden comparison
        # between a warmed heading window and a newly initialized one.
        owner = backend.with_config(backend.config)
        result = owner.solve(source.frame(1000, Rotation.from_rotvec(root), shape), profile)
        anatomy, mano = local_output(result)
        if functional:
            assert result["functional_adapter"]["applied_slots"] == [1, 2, 4, 5, 7, 8, 10, 11, 15]
        vertices = np.asarray(model.render_mano(mano, result["shape_params"])["vertices_m"])
        if expected is not None:
            assert np.max((expected[0].inv() * anatomy).magnitude()) < 1e-10
            assert np.max(np.abs(expected[1] - mano)) < 1e-6
            assert np.max(np.linalg.norm(expected[2] - vertices, axis=1)) < 1e-6
        expected = anatomy, mano, vertices


def test_left_functional_same_owner_retains_shape_during_continuous_direction_change(left_functional_data):
    backend, profile, source, _ = left_functional_data
    owner = backend.with_config(backend.config)
    shape = finger_rotations(fingers=.55, thumb=.45)
    reference = None
    for index in range(151):
        phase = index / 150.
        root = Rotation.from_rotvec([.5 * np.sin(2 * np.pi * phase), -.35 * np.sin(np.pi * phase), 1.2 * phase])
        result = owner.solve(source.frame(2000 + index, root, shape), profile)
        anatomy, mano = local_output(result)
        if reference is None:
            reference = anatomy, mano
        assert np.max((reference[0].inv() * anatomy).magnitude()) < 1e-8
        assert np.max(np.abs(reference[1] - mano)) < 1e-6

def test_left_thumb_distal_and_proximal_inputs_move_their_own_display_segments(left_sparse_data):
    backend, profile, source, _ = left_sparse_data
    identity = Rotation.identity()
    at_rest = [identity] * 16
    neutral = backend.solve(source.frame(1, identity, at_rest), profile)
    for slot in (14, 15):
        local = list(at_rest)
        local[slot] = Rotation.from_euler("Z", 35., degrees=True)
        moved = backend.solve(source.frame(2, identity, local), profile)
        before = Rotation.from_rotvec(neutral["mano_pose_rotvec"])
        after = Rotation.from_rotvec(moved["mano_pose_rotvec"])
        delta = np.rad2deg((before.inv() * after).magnitude())
        assert delta[slot] == pytest.approx(35., abs=.001)
        assert np.delete(delta, slot).max() < .001
    assert source.config["node_to_slot"]["9"] == 15
    assert source.config["node_to_slot"]["10"] == 14


def test_left_profile_requires_directed_frame_even_after_cache_hit(left_sparse_data):
    backend, profile, source, _ = left_sparse_data
    frame = source.frame(4, Rotation.identity(), [Rotation.identity()] * 16)
    assert backend.solve(frame, profile)["pose_available"]
    legacy = deepcopy(profile)
    del legacy["installation_coordinate_policy"]
    with pytest.raises(ValueError, match="coordinate policy"):
        backend.solve(frame, legacy)
