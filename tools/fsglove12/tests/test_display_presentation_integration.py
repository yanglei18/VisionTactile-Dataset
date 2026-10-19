"""Real MANO presentation-only holds, never hardware or measurement repair.

These tests deliberately distinguish the measurement pose, display history and
ROS representation. External source and licensed assets require explicit opt-in.
"""
from copy import deepcopy
import io
import json
import os
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_upstream_backend import fit_data, official_core
from vt_fsglove12.mano import ManoBackend
from vt_fsglove12.pose_export import build_state


pytestmark = pytest.mark.skipif(
    not os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT") or not os.environ.get("FSGLOVE12_MANO_ASSETS"),
    reason="requires explicitly configured official source and licensed MANO model",
)


def slot_value(values, slot):
    return values[str(slot)] if isinstance(values, dict) and str(slot) in values else values[slot]


@pytest.fixture(scope="module")
def rig(official_core):
    backend, profile, _ = fit_data(official_core)
    return backend, profile, ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"])


@pytest.fixture
def cache():
    from vt_fsglove12.display_pose import DisplayPoseCache
    return DisplayPoseCache()


def compose(rig, angles, root):
    backend, profile, _ = rig
    with backend.core.torch.no_grad():
        result = backend._calibrator(profile)._axislayer.compose(
            backend.core.torch.from_numpy(np.asarray(angles).reshape(1, 16, 3)).float())
    mano = result[0].detach().cpu().numpy().astype(float)
    mano[0] = root.as_rotvec()
    return mano


def measured_pose(rig, angles, *, stamp=100., root=None, invalid=(), degraded=()):
    backend, profile, _ = rig
    root = root or Rotation.identity()
    valid, available, quality = [True] * 16, [True] * 16, ["valid"] * 16
    for slot in invalid:
        valid[slot], available[slot], quality[slot] = False, False, "unavailable"
    for slot in degraded:
        valid[slot], available[slot], quality[slot] = False, True, "degraded"
    return dict(timestamp_s=stamp, pose_available=True, solver_success=True,
        algorithm="official_fsglove_with_qmt_functional_adapter_v1", quality="not_accuracy_validated",
        reason="official_inference_complete", upstream_fsglove_solver_executed=True,
        anatomical_euler_rad=np.asarray(angles).tolist(), mano_pose_rotvec=compose(rig, angles, root).tolist(),
        root_quat_xyzw=root.as_quat().tolist(), shape_params=np.linspace(-.3, .4, 10).tolist(),
        session_id=profile["session_id"], profile_version=4, frame_index=int(stamp*30),
        config_sha256=backend._config_sha, model_rest_sha256=backend._rest_sha,
        upstream_state_sha256=profile["upstream_state_sha256"], input_kind="raw_serial",
        joint_valid_mask=valid, joint_display_available_mask=available, joint_quality=quality,
        segment_observed_mask=[True] * 16, joint_observed_mask=[False] * 16,
        inferred_mask=[True] * 16, source_labels=["synthetic_functional_constraint"] * 16)


def source_frame(pose):
    stamp = pose["timestamp_s"]
    return dict(timestamp_s=stamp, session_id=pose["session_id"], frame_index=pose["frame_index"],
                nodes={"12": dict(host_realtime_ns=1_800_000_000_000_000_000+int(stamp*1e9),
                    host_monotonic_ns=int(stamp*1e9), sample_index=pose["frame_index"], epoch=2)})


def test_held_local_rotations_follow_current_parent_root_and_shape_in_real_mano(rig, cache):
    _, _, mesh = rig
    initial_angles = np.zeros((16, 3))
    initial_angles[7] = [0., .1, .25]
    initial_angles[8, 2], initial_angles[9, 2] = .7, .45
    initial = measured_pose(rig, initial_angles)
    initial_before = deepcopy(initial)
    cache.update(initial, context="profile-4", now=100.01)
    current_angles = initial_angles.copy()
    current_angles[7] = [0., -.2, .65]
    current_angles[[8, 9]] = 0.  # Measurement placeholders must remain placeholders.
    root = Rotation.from_euler("XYZ", [.3, -.4, .55])
    current = measured_pose(rig, current_angles, stamp=100.1, root=root, invalid=(8, 9))
    current_before = deepcopy(current)
    displayed = cache.update(current, context="profile-4", now=100.11)
    assert displayed["display_only"] is True
    assert set(displayed["held_slots"]) == {8, 9} and not displayed["unknown_slots"]
    np.testing.assert_array_equal(np.asarray(displayed["mano_pose_rotvec"])[[8, 9]],
                                  np.asarray(initial["mano_pose_rotvec"])[[8, 9]])
    np.testing.assert_array_equal(displayed["mano_pose_rotvec"][0], current["mano_pose_rotvec"][0])
    assert displayed["shape_params"] == current["shape_params"]
    mixed = current_angles.copy()
    mixed[[8, 9]] = initial_angles[[8, 9]]
    expected_pose = compose(rig, mixed, root)
    np.testing.assert_allclose(displayed["mano_pose_rotvec"], expected_pose, atol=3e-6)
    actual = mesh.render_mano(displayed["mano_pose_rotvec"], displayed["shape_params"])
    expected = mesh.render_mano(expected_pose, current["shape_params"])
    for field in ("vertices_m", "joints_m"):
        np.testing.assert_allclose(actual[field], expected[field], atol=2e-6)
    assert len(actual["vertices_m"]) == 778 and len(actual["joints_m"]) == 21
    assert len(actual["faces"]) == len(expected["faces"])
    old_geometry = mesh.render_mano(initial["mano_pose_rotvec"], initial["shape_params"])
    assert not np.allclose(actual["joints_m"], old_geometry["joints_m"])
    assert slot_value(displayed["source_timestamp_by_slot"], 8) == 100.
    assert slot_value(displayed["source_age_by_slot"], 8) == pytest.approx(.11)
    assert (initial, current) == (initial_before, current_before)


def test_degraded_to_unavailable_keeps_last_displayed_local_pose_not_earlier_valid(rig, cache):
    angles = np.zeros((16, 3)); angles[8, 2] = .2
    good = measured_pose(rig, angles)
    cache.update(good, context="same", now=100.)
    angles[8, 2] = .8
    provisional = measured_pose(rig, angles, stamp=100.05, degraded=(8,))
    shown = cache.update(provisional, context="same", now=100.06)
    assert 8 in shown["provisional_slots"]
    angles[8] = 0.
    missing = measured_pose(rig, angles, stamp=100.1, invalid=(8,))
    before = deepcopy((good, provisional, missing))
    held = cache.update(missing, context="same", now=100.12)
    assert 8 in held["held_slots"]
    np.testing.assert_array_equal(held["mano_pose_rotvec"][8], provisional["mano_pose_rotvec"][8])
    assert slot_value(held["held_source_quality_by_slot"], 8) == "degraded"
    assert slot_value(held["source_timestamp_by_slot"], 8) == 100.05
    assert slot_value(held["source_age_by_slot"], 8) == pytest.approx(.07)
    assert (good, provisional, missing) == before
    # Consumer writes must not corrupt the cache's retained local rotation.
    held["mano_pose_rotvec"][8] = [9., 8., 7.]
    again = cache.update(missing, context="same", now=100.13)
    np.testing.assert_array_equal(again["mano_pose_rotvec"][8], provisional["mano_pose_rotvec"][8])


@pytest.mark.parametrize("change", range(8), ids=["session", "clear", "profile", "config", "rest",
                                                 "upstream-state", "revision", "enabled"])
def test_context_change_does_not_carry_local_display_history(rig, cache, change):
    angles = np.zeros((16, 3)); angles[8, 2] = .8
    initial = measured_pose(rig, angles)
    context = tuple(range(8))
    cache.update(initial, context=context, now=100.)
    changed = list(context); changed[change] += 100
    missing = measured_pose(rig, angles, stamp=100.1, invalid=(8,))
    shown = cache.update(missing, context=tuple(changed), now=100.1)
    assert 8 in shown["unknown_slots"] and 8 not in shown["held_slots"]
    np.testing.assert_allclose(shown["mano_pose_rotvec"][8], [0., 0., 0.], atol=1e-8)
    assert missing["joint_quality"][8] == "unavailable"
    assert missing["anatomical_euler_rad"][8][2] == .8  # No in-place zeroing.


def test_invalid_root_never_creates_a_new_current_geometry_from_old_root(rig, cache):
    angles = np.zeros((16, 3)); angles[8, 2] = .6
    good = measured_pose(rig, angles)
    cache.update(good, context="one", now=100.)
    root_missing = measured_pose(rig, angles, stamp=100.1, invalid=(0,))
    before = deepcopy(root_missing)
    assert cache.update(root_missing, context="one", now=100.1) is None
    assert root_missing == before
    unavailable = {**good, "pose_available": False, "solver_success": False,
                   "reason": "missing_or_stale_root", "timestamp_s": 100.2}
    assert cache.update(unavailable, context="one", now=100.2) is None


def test_held_real_mano_geometry_is_not_exported_as_measurement(rig, cache):
    backend, _, mesh = rig
    angles = np.zeros((16, 3)); angles[8, 2], angles[9, 2] = .7, .45
    good = measured_pose(rig, angles)
    cache.update(good, context="same", now=100.)
    angles[[8, 9]] = 0.
    missing = measured_pose(rig, angles, stamp=100.1, invalid=(8, 9))
    frame = source_frame(missing)
    before = deepcopy((frame, missing))
    # Runtime must use measurement geometry=None when the one render is a held display.
    expected = build_state(frame, missing, None, backend.config, root_node="12",
                           solve_completed_monotonic_ns=100_120_000_000)
    display = cache.update(missing, context="same", now=100.11)
    rendered = mesh.render_mano(display["mano_pose_rotvec"], display["shape_params"])
    assert rendered["mano_pose_rotvec"][8] != missing["mano_pose_rotvec"][8]
    actual = build_state(frame, missing, None, backend.config, root_node="12",
                         solve_completed_monotonic_ns=100_120_000_000)
    assert actual == expected
    assert not actual["pose_valid"] and actual["joint_positions_m"] == []
    np.testing.assert_array_equal(np.asarray(actual["mano_pose_rotvec"]).reshape(16, 3),
                                  missing["mano_pose_rotvec"])
    quality = json.loads(actual["quality_json"])
    assert quality["joint_quality"][8] == quality["joint_quality"][9] == "unavailable"
    assert "held_slots" not in json.dumps(actual)
    assert (frame, missing) == before


def test_runtime_single_render_routes_held_geometry_only_to_display_not_recording_or_ros(rig, tmp_path):
    """Exercise the actual tick/export wiring without starting any reader."""
    from vt_fsglove12.live_runtime import LiveController, _Session

    backend, profile, mesh = rig
    clock = SimpleNamespace(value=100.)
    control = SimpleNamespace(pose=None)
    rendered, exported = [], []

    class PoseBackend:
        def solve(self, frame, profile, *, timestamp_s):
            assert timestamp_s == control.pose["timestamp_s"]
            return deepcopy(control.pose)

    class Geometry:
        def render_mano(self, pose, shape):
            rendered.append((deepcopy(pose), deepcopy(shape)))
            return mesh.render_mano(pose, shape)

        def render(self, *args):
            raise AssertionError("official display must not use legacy Euler rendering")

    class Fusion:
        def snapshot(self, now):
            frame = source_frame(control.pose)
            frame["timestamp_s"] = now
            frame["nodes"]["12"].update(age_s=.01, orientation_valid=True,
                quat_xyzw=control.pose["root_quat_xyzw"])
            return frame

        def reset_for_calibration(self):
            return {"method": "test_no_device_reset"}

    class Exporter:
        def publish(self, value):
            exported.append(deepcopy(value))

        def record_error(self, error):
            raise AssertionError(f"export failed: {error}")

        def snapshot(self):
            return {"enabled": True, "sent": len(exported)}

    controller = LiveController(backend.config, backend.rest, {}, output_root=tmp_path,
        clock=lambda: clock.value, geometry_backend=Geometry(), pose_backend=PoseBackend(),
        pose_exporter=Exporter())
    controller._profile = {**deepcopy(profile), "profile_version": 4}
    session = _Session(profile["session_id"], tmp_path, "NO_DEVICE",
        SimpleNamespace(input_kind="csv_replay"), Fusion(), object(), clock.value)
    session.opened = session.metadata_written = True
    session.last_sample = clock.value
    session.streams = {name: io.StringIO() for name in ("orientations", "poses", "events")}
    controller._session = session

    angles = np.zeros((16, 3)); angles[8, 2] = .7
    control.pose = measured_pose(rig, angles)
    controller._tick(session)
    assert len(rendered) == 1 and exported[-1]["pose_valid"]
    assert len(exported[-1]["joint_positions_m"]) == 63

    clock.value = session.last_sample = 100.1
    angles[8] = 0.
    control.pose = measured_pose(rig, angles, stamp=100.1, invalid=(8,),
                                 root=Rotation.from_euler("XYZ", [.2, -.1, .3]))
    measurement = deepcopy(control.pose)
    controller._tick(session)
    snapshot = controller.latest_snapshot()
    assert len(rendered) == 2, "holding must not require a second MANO evaluation"
    assert snapshot["geometry"] is None
    assert snapshot["display_pose"]["held_slots"] == [8]
    assert len(snapshot["display_geometry"]["vertices_m"]) == 778
    assert len(snapshot["display_geometry"]["faces"]) > 0
    assert snapshot["display_pose"]["mano_pose_rotvec"][8] != measurement["mano_pose_rotvec"][8]
    assert not exported[-1]["pose_valid"] and exported[-1]["joint_positions_m"] == []
    np.testing.assert_array_equal(np.asarray(exported[-1]["mano_pose_rotvec"]).reshape(16, 3),
                                  measurement["mano_pose_rotvec"])
    saved = [json.loads(line) for line in session.streams["poses"].getvalue().splitlines()][-1]
    assert saved["mano_pose_rotvec"] == measurement["mano_pose_rotvec"]
    assert saved["anatomical_euler_rad"] == measurement["anatomical_euler_rad"]
    assert saved["joint_quality"][8] == "unavailable" and "display_only" not in saved
    assert control.pose == measurement

    clock.value = session.last_sample = 100.2
    controller._profile["profile_version"] = 5
    control.pose = measured_pose(rig, angles, stamp=100.2, invalid=(8,))
    controller._tick(session)
    snapshot = controller.latest_snapshot()
    assert len(rendered) == 3
    assert snapshot["display_pose"]["held_slots"] == []
    assert snapshot["display_pose"]["unknown_slots"] == [8]
    controller._clear_calibration()
    assert controller.latest_snapshot()["display_pose"] is None
    assert controller.latest_snapshot()["display_geometry"] is None
