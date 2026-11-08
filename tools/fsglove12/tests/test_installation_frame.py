"""Directed installation frames under independent IMU yaw references.

Synthetic rigid/articulated kinematics establish coordinate consistency, not
anatomical accuracy or six-axis drift removal on a physical glove.
"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import PARENTS
from vt_fsglove12.installation_frame import directed_installation_error


def poses(axis, yaw=None):
    mount = Rotation.random(16, random_state=np.random.default_rng(830))
    heading = Rotation.from_euler("z", np.zeros(16) if yaw is None else yaw)
    return mount, heading, tuple((heading * turn * mount).as_quat() for turn in (
        Rotation.identity(), Rotation.from_rotvec(-.55 * axis), Rotation.from_rotvec(.65 * axis)))


def inferred_frames(inputs, function):
    base = Rotation.from_quat(inputs[0])
    return base * Rotation.from_rotvec(function(*inputs)).inv()


@pytest.mark.parametrize("axis", [[-.6, .8, 0.], [.8, .6, 0.], [-.6, -.8, 0.], [0., -1., 0.]])
def test_independent_heading_changes_only_each_world_reference(axis):
    axis = np.asarray(axis)
    _, _, reference = poses(axis)
    _, heading, changed = poses(axis, np.linspace(-2.8, 2.9, 16))
    frames = inferred_frames(reference, directed_installation_error)
    changed_frames = inferred_frames(changed, directed_installation_error)
    assert np.max(((heading * frames).inv() * changed_frames).magnitude()) < 1e-12
    matrices = changed_frames.as_matrix()
    np.testing.assert_allclose(np.linalg.det(matrices), 1., atol=1e-12)
    np.testing.assert_allclose(matrices.transpose(0, 2, 1) @ matrices,
                               np.tile(np.eye(3), (16, 1, 1)), atol=1e-12)
    np.testing.assert_allclose(changed_frames.apply([0., 1., 0.]),
                               np.tile([0., 0., 1.], (16, 1)), atol=1e-12)


def synthetic_kinematics(global_error, inputs, raw, rest_frames):
    """Original global/local installation equations, independent of MANO.

    S_i is the cumulative neutral bone frame. G_i=V_i^-1 B_i and
    L_i=S_i^-1 G_i. A_i=(F_parent D_i)^-1 F_i with F_i=G_i B_i^-1 Q_i L_i^-1.
    """
    global_error = Rotation.from_rotvec(global_error)
    local_error = rest_frames.inv() * global_error
    segment = global_error * Rotation.from_quat(inputs[0]).inv() * raw * local_error.inv()
    result = [segment[0]]
    for slot, parent in enumerate(PARENTS[1:], 1):
        delta = rest_frames[parent].inv() * rest_frames[slot]
        result.append((segment[parent] * delta).inv() * segment[slot])
    return Rotation.concatenate(result)


def moving_hand(axis, mount, heading, root, articulated, *, rest=None):
    """Forward kinematics of a known hand, followed by fixed sensor mounts.

    Global hand direction changes at root; local joint angles remain known.
    Every IMU sees an independent constant heading E_i and mounting M_i.
    """
    rng = np.random.default_rng(440)
    if rest is None:
        rest = Rotation.concatenate([Rotation.identity(), Rotation.random(15, random_state=rng)])
    local = Rotation.from_rotvec(rng.uniform(-.4, .4, (16, 3)) if articulated else np.zeros((16, 3)))
    local_rotations = [root]
    segments = [root]
    for slot, parent in enumerate(PARENTS[1:], 1):
        delta = rest[parent].inv() * rest[slot]
        segments.append(segments[parent] * delta * local[slot])
        local_rotations.append(local[slot])
    virtual = Rotation.from_matrix(np.stack([np.cross([0., 0., 1.], axis), [0., 0., 1.], axis], axis=1))
    physical_increment = virtual * Rotation.concatenate(segments) * rest.inv() * virtual.inv()
    return heading * physical_increment * mount, rest, Rotation.concatenate(local_rotations)


@pytest.mark.parametrize("root", [[0., 0., 0.], [.2, -.4, .3], [1.4, -.8, 2.0], [-1.0, .7, -.8]])
@pytest.mark.parametrize("articulated", [False, True])
def test_known_joint_motion_is_independent_of_whole_hand_orientation(root, articulated):
    axis = np.array([-.6, .8, 0.])
    mount, heading, inputs = poses(axis, np.linspace(-2.8, 2.9, 16))
    raw, rest, expected = moving_hand(axis, mount, heading, Rotation.from_rotvec(root), articulated)
    error = directed_installation_error(*inputs)
    actual = synthetic_kinematics(error, inputs, raw, rest)
    assert np.max((expected.inv() * actual).magnitude()) < 1e-12


def test_sweep_direction_is_retained_and_does_not_become_an_unsigned_axis():
    _, _, inputs = poses(np.array([-.6, .8, 0.]))
    function = directed_installation_error
    forward = inferred_frames(inputs, function)
    reverse = inferred_frames((inputs[0], inputs[2], inputs[1]), function)
    np.testing.assert_allclose(reverse.as_matrix()[:, :, 2], -forward.as_matrix()[:, :, 2], atol=1e-12)
    np.testing.assert_allclose((forward.inv() * reverse).magnitude(), np.pi, atol=1e-12)


@pytest.mark.parametrize("kind", ["zero_sweep", "vertical_sweep", "half_turn", "nan", "not_unit", "wrong_shape"])
def test_undefined_or_invalid_installation_input_is_rejected(kind):
    _, _, original = poses(np.array([1., 0., 0.]))
    inputs = [value.copy() for value in original]
    if kind == "zero_sweep":
        inputs[2] = inputs[1].copy()
    elif kind in ("vertical_sweep", "half_turn"):
        delta = [0., 0., .8] if kind == "vertical_sweep" else [np.pi, 0., 0.]
        inputs[2] = (Rotation.from_rotvec(delta) * Rotation.from_quat(inputs[1])).as_quat()
    elif kind == "nan":
        inputs[1][2, 1] = np.nan
    elif kind == "not_unit":
        inputs[0][3] *= 2.
    else:
        inputs[0] = inputs[0][:15]
    with pytest.raises(ValueError):
        directed_installation_error(*inputs)


def test_unit_quaternion_sign_equivalence_and_input_arrays_unchanged():
    _, _, inputs = poses(np.array([-.6, .8, 0.]), np.linspace(-2., 2., 16))
    before = [value.copy() for value in inputs]
    error = directed_installation_error(*inputs)
    signed = [value.copy() for value in inputs]
    signed[0][::2] *= -1.; signed[1][1::2] *= -1.; signed[2] *= -1.
    other = directed_installation_error(*signed)
    assert np.max((Rotation.from_rotvec(error).inv() * Rotation.from_rotvec(other)).magnitude()) < 1e-12
    for actual, expected in zip(inputs, before):
        np.testing.assert_array_equal(actual, expected)


def test_small_but_defined_sweep_is_not_rejected_as_an_action_threshold():
    _, _, inputs = poses(np.array([1., 0., 0.]))
    inputs = list(inputs)
    inputs[2] = (Rotation.from_rotvec([1e-5, 0., 0.]) * Rotation.from_quat(inputs[1])).as_quat()
    assert np.isfinite(directed_installation_error(*inputs)).all()
