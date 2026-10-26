"""Explicit bias bounds use the original VQF, including every restart path."""
from itertools import groupby
from importlib.resources import files

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from vqf import VQF

from vt_fsglove12.config import digest, load_config, validate_config
from vt_fsglove12.input_contract import raw_to_si
from vt_fsglove12.live_stream import OnlineFusion, RawRecorder, RawSample
from vt_fsglove12.replay import iter_orientation_frames, replay_raw
from vt_fsglove12.vqf_settings import vqf_parameters


def input_config(**changes):
    return dict(sample_rate_hz=104., output_rate_hz=104.,
                gyro_dps_per_lsb=.035, accel_g_per_lsb=.000244, **changes)


def sample(stamp, *, node=1, gyro_z=100):
    return RawSample(node, (0, 0, gyro_z, 0, 0, 4096), stamp, stamp + 1000.)


@pytest.mark.parametrize("value", [None, [], 8., {"restThGyr": 8.}, {"biasClip": True},
    {"biasClip": "8"}, {"biasClip": 0}, {"biasClip": -1}, {"biasClip": float("nan")},
    {"biasClip": float("inf")}, {"biasClip": 10 ** 400}])
def test_invalid_parameters_are_rejected_before_live_or_replay_output(tmp_path, value):
    config = {**load_config(), "vqf": value}
    for entrypoint in (vqf_parameters, validate_config, OnlineFusion):
        with pytest.raises(ValueError, match="vqf"):
            entrypoint(config)
    target = tmp_path / "orientations.jsonl"
    with pytest.raises(ValueError, match="vqf"):
        replay_raw(tmp_path / "raw.csv", config, target)
    assert not target.exists()


def test_missing_parameters_preserve_configuration_digest_and_default_numerics():
    config = load_config()
    before = digest(config)
    assert vqf_parameters(config) == {}
    assert "vqf" not in validate_config(config)
    assert digest(validate_config(config)) == before
    online, official = OnlineFusion({}), VQF(1. / 120.)
    raw = (200, -300, 400, 1200, -100, 8197)
    gyro, accel = raw_to_si(raw, gyro_dps_per_lsb=.00875, accel_g_per_lsb=.000122)
    for index in range(600):
        online.update([RawSample(1, raw, index / 120., 1000. + index / 120.)])
        official.update(gyro, accel)
    np.testing.assert_array_equal(online.states["1"]["filter"].getQuat6D(), official.getQuat6D())
    np.testing.assert_array_equal(online.states["1"]["filter"].getBiasEstimate()[0],
                                  official.getBiasEstimate()[0])


def test_explicit_bound_recovers_known_static_bias_without_changing_raw_input():
    # Synthetic known rest: 3.5 deg/s is an injected zero-rate offset, not motion.
    # Exercise the shipped preset so forgetting its override reproduces the
    # original failure, even when the parameter plumbing itself still works.
    preset = load_config(files("vt_fsglove12").joinpath("configs/right_new_imu.json"))
    legacy = OnlineFusion({key: value for key, value in preset.items() if key != "vqf"})
    configured = OnlineFusion(preset)
    start = {}
    for index in range(12 * 104):
        row = sample(1. + index / 104.)
        legacy.update([row])
        configured.update([row])
        if index == 7 * 104:
            start = {name: Rotation.from_quat(fusion.states["1"]["quat_xyzw"])
                     for name, fusion in (("legacy", legacy), ("configured", configured))}
    assert not legacy.states["1"]["filter"].getRestDetected()
    assert configured.states["1"]["filter"].getRestDetected()
    np.testing.assert_allclose(np.rad2deg(configured.states["1"]["filter"].getBiasEstimate()[0]),
                               [0., 0., 3.5], atol=.02)
    angles = {name: np.rad2deg((start[name].inv() * Rotation.from_quat(
        fusion.states["1"]["quat_xyzw"])).magnitude())
        for name, fusion in (("legacy", legacy), ("configured", configured))}
    assert angles["legacy"] > 17.
    assert angles["configured"] < .1
    assert configured.states["1"]["raw_values"] == list(row.values)
    assert configured.states["1"]["gyro_rad_s"] == legacy.states["1"]["gyro_rad_s"]


def test_rotation_above_explicit_bias_bound_is_not_absorbed_as_rest():
    fusion = OnlineFusion(input_config(vqf={"biasClip": 8.}))
    # 14 deg/s about gravity remains measurable integration, not a zero rate.
    for index in range(8 * 104):
        fusion.update([sample(1. + index / 104., gyro_z=400)])
    state = fusion.states["1"]
    assert not state["filter"].getRestDetected()
    np.testing.assert_allclose(Rotation.from_quat(state["quat_xyzw"]).as_rotvec(),
                               [0., 0., np.deg2rad(112.)], atol=1e-12)


@pytest.mark.parametrize("restart", ["calibration", "long_gap", "clock_rollback"])
def test_explicit_parameters_survive_filter_restarts(restart):
    fusion = OnlineFusion(input_config(vqf={"biasClip": 8.}))
    for index in range(5 * 104):
        row = sample(10. + index / 104.)
        fusion.update([row])
    original = fusion.states["1"]["filter"]
    stamp = row.host_monotonic_s
    if restart == "calibration":
        fusion.reset_for_calibration()
        stamp += 1. / 104.
    elif restart == "long_gap":
        stamp += 1.01
    else:
        stamp -= 5.
    fusion.update([sample(stamp)])
    replacement = fusion.states["1"]["filter"]
    reference = VQF(1. / 104., biasClip=8.)
    reference.update(np.deg2rad([0., 0., 3.5]), np.array([0., 0., 4096 * .000244 * 9.80665]))
    assert replacement is not original
    assert replacement.params["biasClip"] == 8.
    np.testing.assert_array_equal(replacement.getQuat6D(), reference.getQuat6D())
    for index in range(1, 5 * 104):
        fusion.update([sample(stamp + index / 104.)])
        reference.update(np.deg2rad([0., 0., 3.5]), np.array([0., 0., 4096 * .000244 * 9.80665]))
    assert replacement.getRestDetected()
    np.testing.assert_array_equal(replacement.getQuat6D(), reference.getQuat6D())
    np.testing.assert_array_equal(replacement.getBiasEstimate()[0], reference.getBiasEstimate()[0])


def test_raw_replay_and_live_rebuild_the_same_configured_filter(tmp_path):
    config = input_config(vqf={"biasClip": 8.})
    rows = [sample(1. + index / 104.) for index in range(3 * 104)]
    rows += [sample(8. + index / 104.) for index in range(3 * 104)]
    rows += [sample(.5 + index / 104.) for index in range(3 * 104)]
    raw, out = tmp_path / "raw.csv", tmp_path / "orientations.jsonl"
    recorder = RawRecorder(raw).open()
    recorder.write(rows)
    recorder.close()
    replay_raw(raw, config, out)
    frames = iter(iter_orientation_frames(out))
    live = OnlineFusion(config)
    for stamp, batch in groupby(rows, key=lambda row: row.host_monotonic_s):
        live.update(list(batch))
        replayed = next(frames)["nodes"]["1"]
        current = live.snapshot(stamp)["nodes"]["1"]
        for field in ("quat_xyzw", "sample_index", "epoch", "fusion_state", "gyro_rad_s", "raw_values"):
            assert current[field] == replayed[field], field
    assert next(frames, None) is None
    assert live.states["1"]["filter"].params["biasClip"] == 8.


def test_health_recovery_preserves_configured_bias_bound():
    fusion = OnlineFusion({"vqf": {"biasClip": 8.}, "input_health": {"enabled": True}})

    def batch(index, frozen):
        stamp = 10. + index / 120.
        return [RawSample(node, (1000, change, -1000, 100 + change, 20, 8197),
                          stamp, stamp + 1000., 0)
                for node in range(1, 13)
                for change in [0 if frozen and node >= 7 else index % 100]]

    for index in range(480):
        fusion.update(batch(index, True))
    assert fusion.input_health.snapshot(10. + 479 / 120.)["faulted_nodes"] == list(map(str, range(7, 13)))
    previous = {node: state["filter"] for node, state in fusion.states.items()}
    for index in range(480, 960):
        fusion.update(batch(index, False))
    assert not fusion.input_health.snapshot(10. + 959 / 120.)["faulted_nodes"]
    for node in map(str, range(7, 13)):
        assert fusion.states[node]["filter"] is not previous[node]
        assert fusion.states[node]["filter"].params["biasClip"] == 8.
