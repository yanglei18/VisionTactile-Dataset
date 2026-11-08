import numpy as np
import pytest
from PySide6 import QtWidgets
from scipy.spatial.transform import Rotation

from vt_hand_tactile_viewer.mounting_demo import MountingDemo, _finger_clips


@pytest.fixture(params=["left", "right"])
def demo(request, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    side = 1 if request.param == "right" else -1
    basis = Rotation.from_euler("zy", [13, 24], degrees=True).as_matrix()
    axes = dict(hand=request.param, pitch_axis=-side * basis[0], roll_axis=basis[1],
                palm_up_axis=basis[2])
    joints = np.zeros((21, 3))
    for finger in range(5):
        joints[1 + 4 * finger:5 + 4 * finger, 0] = side * (finger - 2) * .015
        joints[1 + 4 * finger:5 + 4 * finger, 1] = [.05, .075, .095, .12]
    # Nonzero wrist and oblique model coordinates catch accidental rotation of
    # the mesh about the origin or about a world axis.
    joints = joints @ basis + [.06, .02, .04]
    geometry = dict(vertices_m=joints[[0, 8, 20]].copy(), faces=[[0, 1, 2]],
                    joints_m=joints, provenance=dict(hand=request.param))
    view = MountingDemo(geometry, axes)
    yield view, joints
    view.close()
    app.processEvents()


def test_demonstration_tips_tilt_in_opposite_directions_about_a_fixed_wrist(demo):
    view, joints = demo
    original = joints @ view.basis.T
    assert view.step == "neutral"
    for step, degrees in (("fingers_down", 50.), ("thumb_up", -50.), ("neutral", 0.)):
        view.set_step(step, hold=True)
        posed = view.mesh.transform().map(original.T).T
        np.testing.assert_allclose(posed[0], original[0], atol=1e-7)
        tip = posed[12] - posed[0]
        assert np.degrees(np.arctan2(tip[2], tip[1])) == pytest.approx(degrees, abs=1e-4)
        np.testing.assert_allclose(np.linalg.norm(np.diff(posed, axis=0), axis=1),
                                   np.linalg.norm(np.diff(original, axis=0), axis=1), atol=1e-7)
    np.testing.assert_allclose(view.mesh_data.vertexes(), original[[0, 8, 20]], atol=1e-7)


def test_selection_animates_but_an_active_capture_holds_the_exact_target(demo, monkeypatch):
    view, joints = demo
    monkeypatch.setattr(view, "isVisible", lambda: True)
    view.set_step("fingers_down")
    view._animation.setCurrentTime(view._animation.duration() // 2)
    half = view.mesh.transform().matrix()
    view.set_step("fingers_down")
    np.testing.assert_array_equal(view.mesh.transform().matrix(), half)
    view.set_step("fingers_down", hold=True)
    assert view._animation.state() == view._animation.State.Stopped
    assert view._pitch == 50.
    assert not np.allclose(view.mesh.transform().matrix(), half)


def test_functional_demo_cycles_only_target_fingers_and_leaves_capture_targets_open(demo, monkeypatch):
    view, joints = demo
    poses = []

    class Renderer:
        def render(self, pose):
            poses.append(pose.copy())
            posed = joints.copy()
            for slot, tip in ((1, 8), (4, 12), (7, 20), (10, 16), (15, 4)):
                posed[tip, 1] -= .03 * pose[slot, 2]
            return dict(joints_m=posed, vertices_m=posed[[0, 8, 20]])

    view._clips = _finger_clips(Renderer(), view._neutral)
    for action, (_, joint_frames) in view._clips.items():
        tip_distances = np.linalg.norm(joint_frames[-1] - joint_frames[0], axis=1)
        assert set(np.flatnonzero(tip_distances > .001)) == (
            {8, 12, 16, 20} if action == "flex_fingers" else {4})
    np.testing.assert_allclose(poses[15][1:4, 2], np.deg2rad([65., 80., 52.]))
    assert set(np.flatnonzero(poses[-1][:, 2])) == {15}
    monkeypatch.setattr(view, "isVisible", lambda: True)
    view.set_step("thumb_up", hold=True)
    duration = view._flexion.duration()
    for action in ("flex_fingers", "flex_thumb"):
        view.set_step(action)
        assert view._pitch == 0 and view._flexion.state() == view._flexion.State.Running
        view._flexion.setCurrentTime(duration // 2)
        expected = view._clips[action][0][-1] @ view.basis.T
        np.testing.assert_allclose(view.mesh_data.vertexes(), expected, atol=1e-7)
        view.set_step(action)
        assert view._flexion.currentTime() == duration // 2
        view._flexion.setCurrentTime(duration)
        np.testing.assert_allclose(view.mesh_data.vertexes(), view._neutral["vertices_m"] @ view.basis.T, atol=1e-7)
    assert len(poses) == 32  # The animation samples the cache without invoking the numerical backend.
    view.set_step("neutral", hold=True)
    assert view._flexion.state() == view._flexion.State.Stopped
    np.testing.assert_allclose(view.mesh_data.vertexes(), view._neutral["vertices_m"] @ view.basis.T, atol=1e-7)
