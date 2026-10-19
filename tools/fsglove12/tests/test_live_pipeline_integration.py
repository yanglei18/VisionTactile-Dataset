"""Opt-in real VQF/MANO numerical regression, not a physical glove benchmark.

The virtual IMUs have exact rigid mounts, no bias/noise, and gravity-only
accelerometer readings (no translational/centripetal acceleration). Raw int16
quantization and the configured node-specific scales are real parts of this
test. It exercises neither serial hardware nor GUI rendering/scheduling.
"""
import json
import os
import io
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.calibration import apply_calibration
from vt_fsglove12.config import PARENTS, load_config
from vt_fsglove12.guided_calibration import fit_guided_calibration, model_motion_axes
from vt_fsglove12.live_stream import OnlineFusion, RawSample
from vt_fsglove12.mano import ManoBackend
from vt_fsglove12.solver import SparseHandSolver


pytestmark = pytest.mark.skipif(
    not os.environ.get("FSGLOVE12_MANO_ASSETS"),
    reason="Real VQF/MANO integration requires explicitly supplied FSGLOVE12_MANO_ASSETS",
)


class ContinuousVirtualRig:
    def __init__(self, config, rest, axes):
        self.config, self.period = config, 1. / config["sample_rate_hz"]
        self.fusion = OnlineFusion(config)  # Real, incremental upstream VQF.
        self.rest = Rotation.from_quat(rest["rest_global_xyzw"])
        self.local = np.stack([self.rest[0].as_matrix()] + [
            (self.rest[PARENTS[slot]].inv() * self.rest[slot]).as_matrix() for slot in range(1, 16)])
        model_to_world, _ = Rotation.align_vectors([[0., 0., 1.]], [axes["palm_up_axis"]])
        self.model_to_world = Rotation.from_euler("z", .6) * model_to_world
        self.nodes = [str(node) for node in range(1, 13)]
        # Auxiliary 11 follows the palm but remains unmapped/unobserved by the
        # hand solver. All twelve real filter inputs still receive raw samples.
        self.slots = [config["node_to_slot"].get(node, 0) for node in self.nodes]
        self.mounts = Rotation.random(12, random_state=np.random.default_rng(301209))
        self.gyro_scales = np.array([config.get("gyro_dps_per_lsb_by_node", {}).get(
            node, config["gyro_dps_per_lsb"]) for node in self.nodes])
        self.root, self.angles = Rotation.identity(), np.zeros((16, 3))
        self.previous_sensors = self.sensor_rotations(self.root, self.angles)
        self.samples_per_node = 0
        self.profile = self.solver = None
        self.max_raw_abs = 0

    def sensor_rotations(self, root, angles):
        local = self.local @ Rotation.from_euler("XYZ", angles).as_matrix()
        global_matrices = np.empty((16, 3, 3))
        global_matrices[0] = (root * self.rest[0]).as_matrix()
        for slot in range(1, 16):
            global_matrices[slot] = global_matrices[PARENTS[slot]] @ local[slot]
        # sensor -> bone -> neutral MANO -> physical world, with +Z up.
        return self.model_to_world * Rotation.from_matrix(global_matrices[self.slots]) * self.mounts.inv()

    def feed(self, root, angles):
        sensors = self.sensor_rotations(root, angles)
        # This is the body-frame increment consumed by q_next=q*Exp(gyro*dt).
        gyro = (self.previous_sensors.inv() * sensors).as_rotvec() / self.period
        gravity_g = sensors.inv().apply(np.tile([0., 0., 1.], (12, 1)))
        gyro_raw = np.rint(np.rad2deg(gyro) / self.gyro_scales[:, None])
        accel_raw = np.rint(gravity_g / self.config["accel_g_per_lsb"])
        raw = np.concatenate((gyro_raw, accel_raw), axis=1).astype(np.int64)
        assert np.all((-32768 <= raw) & (raw <= 32767)), "Fixture must not clip sensor range"
        self.max_raw_abs = max(self.max_raw_abs, int(np.abs(raw).max()))
        self.samples_per_node += 1
        stamp = 100. + self.samples_per_node * self.period
        batch = [RawSample(int(node), tuple(int(value) for value in values), stamp,
                           1700000000. + stamp, self.samples_per_node % 256)
                 for node, values in zip(self.nodes, raw)]
        self.fusion.update(batch)
        self.previous_sensors = sensors
        self.root, self.angles = root, angles.copy()
        return stamp

    def advance(self, root, angles, seconds, *, capture=None, monitor=False):
        start_root, start_angles = self.root, self.angles.copy()
        root_increment = (start_root.inv() * root).as_rotvec()
        records = []
        count = round(seconds / self.period)
        for index in range(1, count + 1):
            # Smooth rest-to-rest motion; no discontinuous pose jumps.
            fraction = .5 - .5 * np.cos(np.pi * index / count)
            current_root = start_root * Rotation.from_rotvec(root_increment * fraction)
            current_angles = start_angles + fraction * (angles - start_angles)
            stamp = self.feed(current_root, current_angles)
            if index % 4 == 0 or index == count:  # 30 Hz presentation/capture, 120 Hz fusion.
                frame = self.fusion.snapshot(stamp)
                assert len(frame["nodes"]) == 12
                assert all(item["epoch"] == 0 for item in frame["nodes"].values())
                if capture is not None:
                    capture.append(frame)
                if monitor:
                    observations, rejected = apply_calibration(frame, self.profile)
                    assert not rejected
                    result = self.solver.solve(observations, stamp)
                    assert result["pose_available"] and result["solver_success"]
                    joint_error = np.rad2deg(np.asarray(result["anatomical_euler_rad"]) - current_angles)
                    root_error = np.rad2deg((current_root.inv() * Rotation.from_quat(result["root_quat_xyzw"])).magnitude())
                    records.append(dict(result=result, max_joint_error_deg=float(np.abs(joint_error).max()),
                                        root_error_deg=float(root_error)))
        return records


def test_continuous_raw_vqf_guided_calibration_and_hand_return(record_property):
    backend = ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"])
    rest = backend.export_rest()
    axes = model_motion_axes(backend.render(np.zeros((16, 3)))["joints_m"])
    config = load_config()
    rig = ContinuousVirtualRig(config, rest, axes)
    neutral, identity = np.zeros((16, 3)), Rotation.identity()
    captures = {key: [] for key in ("neutral", "fingers_down", "thumb_up")}
    rig.advance(identity, neutral, 1.)
    rig.advance(identity, neutral, 2., capture=captures["neutral"])
    pitch = Rotation.from_rotvec(np.asarray(axes["pitch_axis"]) * np.deg2rad(37.))
    roll = Rotation.from_rotvec(np.asarray(axes["roll_axis"]) * np.deg2rad(46.))
    rig.advance(pitch, neutral, 1.)
    rig.advance(pitch, neutral, 2., capture=captures["fingers_down"])
    rig.advance(identity, neutral, 1.)
    rig.advance(roll, neutral, 1.)
    rig.advance(roll, neutral, 2., capture=captures["thumb_up"])
    rig.profile = fit_guided_calibration(captures, rest, config,
                                        session_id="synthetic-continuous-vqf", model_axes=axes)
    assert not rig.profile["rejected_nodes"]
    assert "11" not in rig.profile["nodes"]
    rig.solver = SparseHandSolver(config, rest["rest_global_xyzw"])
    rig.advance(identity, neutral, 1.)

    bent = neutral.copy()
    for index, proximal in enumerate((1, 4, 7, 10)):
        bent[proximal, 1:] = [.08 * (-1) ** index, .35 + .06 * index]
        bent[proximal + 1, 2] = .65 + .06 * index
        bent[proximal + 2, 2] = config["dip_ratio"] * bent[proximal + 1, 2]
    bent[14, 1:] = [.12, .45]
    bent[15, 2] = .7
    bend_records = rig.advance(identity, bent, 1., monitor=True)
    bent_hold = rig.advance(identity, bent, .5, monitor=True)
    bent_before_root = np.asarray(bent_hold[-1]["result"]["anatomical_euler_rad"])
    arbitrary_root = Rotation.from_euler("XYZ", [.55, -.3, .65])
    rotate_records = rig.advance(arbitrary_root, bent, 1., monitor=True)
    rotated_hold = rig.advance(arbitrary_root, bent, .5, monitor=True)
    root_relative_change = max(float(np.rad2deg(np.abs(
        np.asarray(record["result"]["anatomical_euler_rad"]) - bent_before_root)).max())
        for record in rotate_records + rotated_hold)
    rig.advance(identity, bent, 1.)
    return_records = rig.advance(identity, neutral, 1., monitor=True)
    return_hold = rig.advance(identity, neutral, .5, monitor=True)

    # No delayed solver neutral-reset is needed: evaluate the first actual
    # returned-to-neutral sample, not a frame selected after a settling timeout.
    first_return_error = return_records[-1]["max_joint_error_deg"]
    records = bend_records + bent_hold + rotate_records + rotated_hold + return_records + return_hold
    mesh_errors_mm = []
    for record, expected_angles, expected_root in (
            (bent_hold[-1], bent, identity),
            (rotated_hold[-1], bent, arbitrary_root),
            (return_hold[-1], neutral, identity)):
        result = record["result"]
        actual = backend.render(result["anatomical_euler_rad"], result["root_quat_xyzw"])
        expected = backend.render(expected_angles, expected_root.as_quat())
        vertex_error = np.asarray(actual["vertices_m"]) - np.asarray(expected["vertices_m"])
        mesh_errors_mm.append(float(np.linalg.norm(vertex_error, axis=1).max() * 1000.))
    metrics = dict(
        kind="ideal_int16_vqf_mano_numerical_regression_not_human_accuracy",
        samples_per_node=rig.samples_per_node, nodes=12, nominal_sample_rate_hz=config["sample_rate_hz"],
        frames_solved=len(records), max_raw_abs=rig.max_raw_abs,
        max_joint_error_deg=max(record["max_joint_error_deg"] for record in records),
        max_root_error_deg=max(record["root_error_deg"] for record in records),
        common_root_max_relative_change_deg=root_relative_change,
        first_return_joint_error_deg=first_return_error,
        final_return_joint_error_deg=return_hold[-1]["max_joint_error_deg"],
        max_calibration_fit_deg=max(node["label_fit_rms_deg"] for node in rig.profile["nodes"].values()),
        max_mesh_vertex_error_mm=max(mesh_errors_mm), numerical_angle_tolerance_deg=.5,
        hardware_or_gui_tested=False,
    )
    print("LIVE_PIPELINE_METRICS=" + json.dumps(metrics, sort_keys=True))
    record_property("numerical_metrics_json", json.dumps(metrics, sort_keys=True))
    # At the coarsest configured gyro scale, even accumulated half-LSB error
    # over 16.5 s is 0.145 deg per board. A 0.5-deg regression margin covers
    # differential quantization/calibration and numerical filtering, without
    # serving as an accuracy requirement for real hands or sensors.
    assert metrics["max_joint_error_deg"] < .5
    assert metrics["max_root_error_deg"] < .5
    assert root_relative_change < .5
    assert first_return_error < .5
    assert metrics["final_return_joint_error_deg"] < .5
    assert metrics["max_mesh_vertex_error_mm"] < 2.
    assert rig.fusion.counts == {str(node): rig.samples_per_node for node in range(1, 13)}
    # The exact same current orientation yields the same solver pose: the
    # stateless solver has no additional UI-dependent integration/smoothing.
    frame = rig.fusion.snapshot(100. + rig.samples_per_node * rig.period)
    observations, _ = apply_calibration(frame, rig.profile)
    again = rig.solver.solve(observations, frame["timestamp_s"])
    np.testing.assert_allclose(again["anatomical_euler_rad"],
                               return_hold[-1]["result"]["anatomical_euler_rad"], atol=1e-10)
    final_mesh = backend.render(again["anatomical_euler_rad"], again["root_quat_xyzw"])
    assert np.isfinite(final_mesh["vertices_m"]).all()
    assert not again["anatomical_accuracy_verified"] and not again["yaw_drift_corrected"]


@pytest.mark.skipif(not os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT"),
                    reason="explicit official source checkout required")
def test_official_continuous_raw_vqf_runtime_direct_mano_and_records(tmp_path, monkeypatch, record_property):
    """Real VQF + original FSGlove + MANO, ideal inputs, no hardware accuracy claim."""
    from vt_fsglove12 import live_runtime
    from vt_fsglove12.upstream_backend import OfficialFSGloveBackend, ALGORITHM
    from vt_fsglove12.live_display import measured_pose_visible

    def forbidden(*args, **kwargs):
        raise AssertionError("official pipeline must never call a legacy fitting algorithm")

    for name in ("SparseHandSolver", "apply_calibration", "apply_functional_axes"):
        monkeypatch.setattr(live_runtime, name, forbidden)
    mesh = ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"])
    rest = mesh.export_rest()
    axes = model_motion_axes(mesh.render(np.zeros((16, 3)))["joints_m"])
    config = load_config()
    rig = ContinuousVirtualRig(config, rest, axes)
    neutral, identity = np.zeros((16, 3)), Rotation.identity()
    captures = {key: [] for key in live_runtime.STEP_KEYS}
    rig.advance(identity, neutral, 1.)
    rig.advance(identity, neutral, 2., capture=captures["neutral"])
    up = Rotation.from_rotvec(np.asarray(axes["pitch_axis"]) * np.deg2rad(-40.))
    down = Rotation.from_rotvec(np.asarray(axes["pitch_axis"]) * np.deg2rad(40.))
    rig.advance(up, neutral, 1.)
    rig.advance(up, neutral, 2., capture=captures["fingers_down"])
    rig.advance(down, neutral, 1.)
    rig.advance(down, neutral, 2., capture=captures["thumb_up"])

    official = OfficialFSGloveBackend(config, rest,
        checkout=os.environ["FSGLOVE12_UPSTREAM_CHECKOUT"], mano_assets_root=mesh.assets_root)
    official.core.torch.set_num_threads(1)
    clock = lambda: 100. + rig.samples_per_node * rig.period
    controller = live_runtime.LiveController(config, rest, axes, output_root=tmp_path,
        clock=clock, pose_backend=official, geometry_backend=mesh, calibration_fn=forbidden)
    session = live_runtime._Session("synthetic-official-pipeline", tmp_path, "NO_DEVICE",
        SimpleNamespace(input_kind="synthetic_raw_no_hardware"), rig.fusion, None, clock(),
        streams={name: io.StringIO() for name in ("events", "orientations", "poses")}, opened=True)
    controller._session = session
    controller._captures = captures
    controller._fit_profile(session)
    assert controller._profile and not controller._profile["rejected_nodes"]
    assert controller._solver is None

    snapshots = []
    bent = neutral.copy()
    for proximal in (1, 4, 7, 10):
        bent[proximal, 2], bent[proximal + 1, 2] = .4, .7
    bent[14, 2], bent[15, 2] = .4, .6
    for root, angles in ((identity, neutral), (identity, bent),
                         (Rotation.from_euler("XYZ", [.2, -.3, .4]), bent), (identity, neutral)):
        rig.advance(root, angles, 1.)
        for _ in range(8):
            rig.advance(root, angles, .1)
            session.last_sample = clock()
            controller._tick(session)
            snap = controller.latest_snapshot()
            pose, geometry = snap["pose"], snap["geometry"]
            assert pose["algorithm"] == ALGORITHM and pose["upstream_fsglove_solver_executed"]
            assert pose["pose_available"] and pose["solver_success"]
            assert measured_pose_visible(snap, clock())
            assert not pose["hardware_accessed"] and pose["input_kind"] == "synthetic_raw_no_hardware"
            assert sum(pose["segment_observed_mask"]) == 11
            assert geometry["provenance"]["pose_input"] == "upstream_mano_axis_angle_direct"
            assert np.asarray(geometry["vertices_m"]).shape == (778, 3)
            assert np.isfinite(geometry["vertices_m"]).all()
            snapshots.append(snap)
    written = [json.loads(line) for line in session.streams["poses"].getvalue().splitlines()]
    assert len(written) == 32 and all(p["upstream_fsglove_solver_executed"] for p in written)
    assert np.linalg.norm(np.asarray(written[0]["mano_pose_rotvec"]) - written[15]["mano_pose_rotvec"]) > .1
    assert all(p["quality"] == "not_accuracy_validated" and not p["yaw_drift_corrected"] for p in written)
    saved = json.loads((tmp_path / "profile.v001.json").read_text())
    assert saved["method"] == ALGORITHM and len(saved["upstream_state"]) == 6
    record_property("official_pipeline", json.dumps(dict(
        fixture="ideal_raw_int16_no_hardware_or_accuracy_validation", frames=32,
        samples_per_node=rig.samples_per_node, actual_modules="VQF / upstream Raw2MANO / manotorch",
        legacy_solver_called=False, all_vertices_finite=True,
        total_ms_median=float(np.median([s["runtime"]["timing_ms"]["total"] for s in snapshots])))))


@pytest.mark.skipif(not os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT"),
                    reason="explicit official source checkout required")
def test_official_known_anatomical_axes_and_common_root_do_not_swap(record_property):
    """Synthetic known-axis regression, NOT a label for any recorded gesture.

    Continuous int16 gyro/accelerometer samples pass through real VQF and the
    original official calibrator. No legacy sparse solver or functional-axis
    retargeter is called. Four-finger DIP completion is deliberately excluded
    from measured-axis claims; assertions target observed index/thumb joints.
    """
    from vt_fsglove12.upstream_backend import OfficialFSGloveBackend, STEP_KEYS

    mesh = ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"])
    rest = mesh.export_rest()
    axes = model_motion_axes(mesh.render(np.zeros((16, 3)))["joints_m"])
    config = load_config()
    rig = ContinuousVirtualRig(config, rest, axes)
    neutral, identity = np.zeros((16, 3)), Rotation.identity()
    captures = {key: [] for key in STEP_KEYS}
    rig.advance(identity, neutral, 1.)
    rig.advance(identity, neutral, 2., capture=captures["base_pose"])
    up = Rotation.from_rotvec(np.asarray(axes["pitch_axis"]) * np.deg2rad(-50.))
    down = Rotation.from_rotvec(np.asarray(axes["pitch_axis"]) * np.deg2rad(50.))
    rig.advance(up, neutral, 1.)
    rig.advance(up, neutral, 2., capture=captures["z_rot_pose_1"])
    rig.advance(down, neutral, 1.)
    rig.advance(down, neutral, 2., capture=captures["z_rot_pose_2"])

    official = OfficialFSGloveBackend(config, rest,
        checkout=os.environ["FSGLOVE12_UPSTREAM_CHECKOUT"], mano_assets_root=mesh.assets_root)
    official.core.torch.set_num_threads(1)
    profile = official.fit_calibration(captures, session_id="synthetic-known-axes")
    reversed_captures = dict(captures)
    reversed_captures["z_rot_pose_1"], reversed_captures["z_rot_pose_2"] = (
        captures["z_rot_pose_2"], captures["z_rot_pose_1"])
    reversed_profile = official.fit_calibration(reversed_captures, session_id="synthetic-known-axes")

    def sample(root, angles):
        rig.advance(root, angles, 1.)
        rig.advance(root, angles, .3)
        frame = rig.fusion.snapshot(100. + rig.samples_per_node * rig.period)
        frame["session_id"] = "synthetic-known-axes"
        result = official.solve(frame, profile)
        assert result["pose_available"] and result["upstream_fsglove_solver_executed"]
        return frame, np.rad2deg(result["anatomical_euler_rad"])

    _, baseline = sample(identity, neutral)
    measured_slots = [1, 2, 4, 5, 7, 8, 10, 11, 14, 15]
    assert np.max(np.abs(baseline[measured_slots])) < .15
    rows = []
    bend_frame = None
    bend_angles = None
    bend_output = None
    for axis in range(3):
        target = neutral.copy()
        target[1, axis] = target[14, axis] = np.deg2rad(20.)
        frame, output = sample(identity, target)
        rows.append(output[[1, 14]].tolist())
        # AxisLayerFK labels XYZ as twist/spread/bend. A small nonzero model
        # plane-vs-canonical-frame mismatch is preserved from official code:
        # the fixture yields about 1.5 degrees of cross-axis leakage at 20 deg.
        for slot in (1, 14):
            assert abs(output[slot, axis] - 20.) < .3
            assert np.max(np.abs(np.delete(output[slot], axis))) < 2.2
        if axis == 2:
            bend_frame, bend_angles, bend_output = frame, target, output

    arbitrary_root = Rotation.from_euler("XYZ", [.35, -.25, .4])
    _, rotated = sample(arbitrary_root, bend_angles)
    common_root_error = float(np.max(np.abs(rotated[measured_slots] - bend_output[measured_slots])))
    assert common_root_error < .3

    # Negative control: the labels' directed axis matters. Swapping UP/DOWN
    # flips the index bend and mixes thumb bend into spread; it must not be
    # silently accepted as evidence for a fixed XYZ permutation in production.
    wrong = official.solve(bend_frame, reversed_profile)
    wrong_angles = np.rad2deg(wrong["anatomical_euler_rad"])
    assert wrong_angles[1, 2] < -15.
    assert abs(wrong_angles[14, 1]) > 10.
    assert abs(wrong_angles[14, 2] - 20.) > 8.
    record_property("official_known_axes", json.dumps({
        "fixture": "synthetic_known_20_degree_axes_not_recorded_human_motion",
        "input_rows": ["twist_x", "spread_y", "bend_z"],
        "output_slots": [1, 14], "output_columns": ["twist_x", "spread_y", "bend_z"],
        "output_deg": rows, "common_root_max_measured_joint_change_deg": common_root_error,
        "reversed_calibration_bend_output_deg": wrong_angles[[1, 14]].tolist(),
        "hardware_or_gui_tested": False,
    }))
