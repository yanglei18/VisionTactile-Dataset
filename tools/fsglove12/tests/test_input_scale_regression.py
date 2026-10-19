"""Input-scale regressions using the real incremental VQF implementation.

Synthetic rigid sensor rotations provide known gyro increments and gravity.
The raw encoding uses an independent fixed physical scale, never the config
under test: otherwise an erroneous override could cancel itself in the fixture.
These are numerical regressions, not firmware readback or hardware accuracy
certification. No serial port, real MANO asset, GUI or upstream solver is needed.
"""
from copy import deepcopy
from importlib.resources import files
import json
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import PARENTS, digest, load_config
from vt_fsglove12.live_stream import OnlineFusion, RawSample
from vt_fsglove12.replay import _configuration
from vt_fsglove12.upstream_backend import (
    ALGORITHM, LEGACY_ALGORITHM, OFFICIAL_STEPS, ZERO_POLICY, OfficialFSGloveBackend,
)


PHYSICAL_GYRO_DPS_PER_LSB = .00875
PHYSICAL_ACCEL_G_PER_LSB = .000122


@pytest.mark.parametrize("name", ["right_current.json", "right_thumb3_proposed.json"])
def test_packaged_layouts_use_uniform_scale_without_claiming_verified_units(name):
    config = load_config(files("vt_fsglove12").joinpath("configs", name))
    _, scales = _configuration(config)
    assert config["gyro_dps_per_lsb"] == PHYSICAL_GYRO_DPS_PER_LSB
    assert scales == {str(node): PHYSICAL_GYRO_DPS_PER_LSB for node in range(1, 13)}
    assert config["units_verified"] is False
    assert config["sample_rate_verified"] is False


def _run_fixed_axis_motion(config, axis, segments, *, nodes=tuple(range(1, 13))):
    """Feed quantized gyro/gravity through production OnlineFusion and VQF.

    Every selected board receives the same virtual sensor pose and raw counts.
    Translation, saturation, noise, bias and transport loss are intentionally
    absent, so this fixture isolates an input conversion regression.
    """
    vqf = pytest.importorskip("vqf")
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    fusion = OnlineFusion(config)
    period = 1. / config["sample_rate_hz"]
    previous, angle, index = Rotation.identity(), 0., 0
    endpoints = []
    max_error, max_gravity_error, max_raw_abs = 0., 0., 0
    for target, duration in segments:
        start = angle
        count = round(duration / period)
        for step in range(1, count + 1):
            fraction = .5 - .5 * np.cos(np.pi * step / count)
            angle = start + (target - start) * fraction
            expected = Rotation.from_rotvec(axis * np.deg2rad(angle))
            gyro_dps = np.rad2deg((previous.inv() * expected).as_rotvec()) / period
            gravity_sensor = expected.inv().apply([0., 0., 1.])
            raw = np.rint(np.r_[gyro_dps / PHYSICAL_GYRO_DPS_PER_LSB,
                                gravity_sensor / PHYSICAL_ACCEL_G_PER_LSB]).astype(np.int64)
            assert np.all((-32768 <= raw) & (raw <= 32767)), "Fixture must never saturate"
            max_raw_abs = max(max_raw_abs, int(np.abs(raw).max()))
            index += 1
            stamp = 100. + index * period
            batch = [RawSample(node, tuple(int(value) for value in raw), stamp,
                               1700000000. + stamp, index % 256) for node in nodes]
            fusion.update(batch)
            frame = fusion.snapshot(stamp)
            quaternions = np.array([frame["nodes"][str(node)]["quat_xyzw"] for node in nodes])
            actual = Rotation.from_quat(quaternions)
            errors = np.rad2deg((expected.inv() * actual).magnitude())
            actual_gravity = actual.inv().apply(np.tile([0., 0., 1.], (len(nodes), 1)))
            gravity_errors = np.rad2deg(np.arccos(np.clip(actual_gravity @ gravity_sensor, -1., 1.)))
            max_error = max(max_error, float(np.max(errors)))
            max_gravity_error = max(max_gravity_error, float(np.max(gravity_errors)))
            previous = expected
            if step == count:
                endpoints.append(dict(angle_deg=float(angle), quaternions=quaternions.copy(),
                                      error_deg=errors.copy(), gravity_error_deg=gravity_errors.copy(),
                                      gravity_sensor=gravity_sensor.copy(), raw=raw.copy()))
    assert all(isinstance(state["filter"], vqf.VQF) for state in fusion.states.values())
    assert all(node["epoch"] == 0 for node in frame["nodes"].values())
    return dict(endpoints=endpoints, max_error_deg=max_error,
                max_gravity_error_deg=max_gravity_error, max_raw_abs=max_raw_abs)


@pytest.mark.parametrize("axis", [[1., 0., 0.], [0., 1., 0.], [.6, .8, 0.]])
def test_default_nodes_5_6_match_every_other_real_vqf_during_repeated_known_rotation(axis, record_property):
    config = load_config()
    segments = [(0., 1.)] + [(90., 1.), (90., .25), (0., 1.), (0., .25)] * 3
    result = _run_fixed_axis_motion(config, axis, segments)
    assert result["max_error_deg"] < .3
    assert result["max_gravity_error_deg"] < .3
    for endpoint in result["endpoints"]:
        # Identical counts + identical conversion must yield identical filters,
        # including former special-case boards 5/6 and auxiliary board 11.
        np.testing.assert_array_equal(endpoint["quaternions"],
                                      np.tile(endpoint["quaternions"][0], (12, 1)))
        if endpoint["angle_deg"] == 0.:
            assert float(np.max(endpoint["error_deg"])) < .3
    assert result["max_raw_abs"] < 32767
    record_property("max_synthetic_rotation_error_deg", result["max_error_deg"])
    record_property("max_synthetic_gravity_error_deg", result["max_gravity_error_deg"])
    record_property("hardware_accuracy_verified", False)


def test_old_double_scale_reproduces_post_motion_tilt_recovery_not_only_yaw():
    current = load_config()
    wrong = deepcopy(current)
    wrong["gyro_dps_per_lsb_by_node"] = {"5": .0175, "6": .0175}
    # Fixed X-axis rotation with identical physical scale on all boards.
    # After returning to 0 degrees, gravity and true pose remain unchanged.
    segments = [(0., 1.), (90., 1.), (90., 2.), (0., 1.), (0., .1), (0., 1.), (0., 4.)]
    good = _run_fixed_axis_motion(current, [1., 0., 0.], segments, nodes=(1, 5, 6))
    bad = _run_fixed_axis_motion(wrong, [1., 0., 0.], segments, nodes=(1, 5, 6))
    assert good["max_error_deg"] < .3 and good["max_gravity_error_deg"] < .3
    immediate, settled = bad["endpoints"][4], bad["endpoints"][-1]
    np.testing.assert_array_equal(immediate["raw"], settled["raw"])
    np.testing.assert_array_equal(immediate["gravity_sensor"], settled["gravity_sensor"])
    assert immediate["gravity_error_deg"][0] < .3  # Board 1 is not rescaled.
    for index in (1, 2):
        assert immediate["gravity_error_deg"][index] > 40.
        assert settled["gravity_error_deg"][index] < 15.
        before = Rotation.from_quat(immediate["quaternions"][index])
        after = Rotation.from_quat(settled["quaternions"][index])
        correction = (after * before.inv()).as_rotvec()
        assert np.rad2deg(np.linalg.norm(correction)) > 30.
        assert abs(correction[2]) < 1e-10  # This recovery is tilt, not a yaw-only change.


def _profile_test_data():
    """A non-numerical core stub isolates configuration rejection before MANO.

    It only supplies serializable calibration state. Any inference call is an
    error: these tests must reject the incompatible profile before the solver.
    """
    calls = []

    def create_calibrator(base_pose, **_):
        calls.append("create")
        return SimpleNamespace(
            _base_quat=np.asarray(base_pose), _delta_rot_fa_rotvec=np.zeros((16, 3)),
            _global_installation_error=np.zeros((16, 3)),
            _local_installation_error=np.zeros((16, 3)),
            _joint_fa=np.array([0, *PARENTS[1:]]),
            _fk_seq=np.arange(1, 16),
            get_mano_param=lambda *_args, **_kwargs: pytest.fail("mismatched profile reached inference"),
        )

    core = SimpleNamespace(create_calibrator=create_calibrator,
                           provenance={"kind": "explicit_binding_test_stub_not_official_numerics"})
    config = load_config()
    rest = {"parents": list(PARENTS), "rest_global_xyzw": [[0., 0., 0., 1.]] * 16}
    captures = {}
    for pose_index, (step, angle) in enumerate(zip(OFFICIAL_STEPS, [0., 50., -50.])):
        quaternion = Rotation.from_euler("x", angle, degrees=True).as_quat().tolist()
        captures[step["key"]] = [
            {"timestamp_s": 100. + pose_index + index * .01, "session_id": "scale-test-session",
             "nodes": {node: {"quat_xyzw": quaternion, "age_s": .001, "epoch": 0,
                              "sample_index": pose_index * 10 + index + 1}
                       for node in config["node_to_slot"]}}
            for index in range(3)]
    return config, rest, core, calls, captures


@pytest.mark.parametrize("warm_cache", [False, True])
@pytest.mark.parametrize("method", [ALGORITHM, LEGACY_ALGORITHM])
def test_old_double_scale_profile_is_rejected_before_core_or_cache_use(warm_cache, method):
    config, rest, core, calls, captures = _profile_test_data()
    old_config = deepcopy(config)
    old_config["gyro_dps_per_lsb_by_node"] = {"5": .0175, "6": .0175}
    old_backend = OfficialFSGloveBackend(old_config, rest, checkout="unused", mano_assets_root="unused", core=core)
    saved = old_backend.fit_calibration(captures, session_id="scale-test-session")
    if method == LEGACY_ALGORITHM:
        saved.update(method=method, missing_slot_policy=ZERO_POLICY)
        saved.pop("missing_slot_options")
    saved = json.loads(json.dumps(saved))
    before_profile = deepcopy(saved)
    current_backend = OfficialFSGloveBackend(config, rest, checkout="unused", mano_assets_root="unused", core=core)
    if warm_cache:
        current = current_backend.fit_calibration(captures, session_id="scale-test-session")
        # Equal matrices deliberately force the tempting cache-hit path: the
        # config digest must still be checked first.
        assert current["upstream_state_sha256"] == saved["upstream_state_sha256"]
    assert saved["config_sha256"] == digest(old_config)
    assert saved["config_sha256"] != digest(config)
    assert saved["rest_sha256"] == digest(rest)
    call_count = len(calls)
    with pytest.raises(ValueError, match="configuration or MANO model digest does not match"):
        current_backend.solve(deepcopy(captures["base_pose"][-1]), saved)
    assert len(calls) == call_count
    assert saved == before_profile


def test_upstream_live_default_launch_passes_updated_packaged_config_without_starting_gui(monkeypatch):
    from vt_fsglove12 import cli, upstream_core

    calls = []
    provenance = {"kind": "explicit_cli_test_stub_no_source_or_model_loaded"}
    monkeypatch.setattr(upstream_core, "load_upstream_core",
                        lambda *_: SimpleNamespace(provenance=provenance))
    ui = ModuleType("vt_fsglove12.live_ui")

    def run_live(config, assets, **options):
        calls.append((deepcopy(config), assets, options))
        return 0

    ui.run_live = run_live
    monkeypatch.setitem(sys.modules, "vt_fsglove12.live_ui", ui)
    assert cli.main(["upstream-live", "--fsglove-source", "unused-source",
                     "--mano-assets", "unused-assets"]) == 0
    assert len(calls) == 1
    launched, _, options = calls[0]
    assert launched["layout"] == load_config()["layout"]
    assert _configuration(launched)[1] == {str(node): PHYSICAL_GYRO_DPS_PER_LSB for node in range(1, 13)}
    assert launched["units_verified"] is False
    assert options["upstream_checkout"] == "unused-source"
    assert options["replay_raw"] is None
