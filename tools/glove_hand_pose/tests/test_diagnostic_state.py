"""Real-VQF offline runtime round trips, without hardware or live zero reuse."""
from copy import deepcopy
import json
import math

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.diagnostic_state import LEGACY_ALGORITHM, THUMB_ALGORITHM, get_vqf_version
from vt_glove_pose.engine import PoseEngine
from vt_glove_pose.math3d import from_axis_angle
from vt_glove_pose.records import Sample


def sample(node=1, tick=0, *, gyro=(50, -40, 30), acc=(0, 0, 8197), stamp=None):
    stamp = 10 + tick / 120 if stamp is None else stamp
    return Sample(node, tuple(gyro) + tuple(acc), stamp, 1_700_000_000 + stamp, 0)


def feed(engine, tick, moving=False):
    for node in range(1, 13):
        gyro = (int(900*math.sin(tick/17+node)), int(500*math.cos(tick/21)), node*7) if moving else (50, -40, 30)
        engine.process(sample(node, tick, gyro=gyro))


@pytest.fixture(scope="module")
def ready_checkpoint():
    engine = PoseEngine(load_config())
    # Rest bias estimation develops its own bias, intentionally different from
    # engine.bias (the original static calibration estimate).
    for tick in range(550):
        feed(engine, tick)
    assert np.linalg.norm(engine.states[1]["filter"].getBiasEstimate()[0]) > 0
    engine.zero = {n: s["quat"].copy() for n, s in engine.states.items()}
    engine.neutral_acc = {n: np.array([0., 0., 1.]) for n in range(1, 13)}
    engine.mounts = {n: from_axis_angle([1, 2, 3], n*.04) for n in (1, 2, 7, 8, 12)}
    engine.axis_sign_versions = {n: 2 for n in engine.mounts}
    engine.axes_quality = {n: {"linearity": .98, "moving_samples": 90,
                               "sign_window": "first coherent untrimmed motion",
                               "axis_sign_algorithm_version": 2} for n in engine.mounts}
    engine.neutral_ready = engine.mount_confirmed = True
    engine.message = "fixture historical neutral reference"
    return engine.export_diagnostic_checkpoint()


def test_empty_and_partly_initialized_engines_are_supported():
    engine = PoseEngine(load_config())
    empty = engine.export_diagnostic_checkpoint()
    assert empty["offline_only"] is True
    assert empty["vqf_version"] == get_vqf_version()
    assert empty["schema_version"] == 2
    assert empty["pose_algorithm"] == THUMB_ALGORITHM
    assert empty["engine"]["thumb_estimator"] is None
    assert empty["engine"]["thumb_pair_counts"] == [0, 0]
    assert empty["engine"]["thumb_common_capture"] is None
    restored = PoseEngine.from_offline_checkpoint(empty)
    assert restored is not engine and restored.states == {}
    assert not restored.neutral_ready
    # Rejected zero acceleration creates a filter without a valid observation.
    engine.process(sample(acc=(0, 0, 0)))
    state = engine.export_diagnostic_checkpoint()
    assert set(state["engine"]["states"]["1"]) == {"count", "times", "filter"}
    assert PoseEngine.from_offline_checkpoint(state).export_diagnostic_checkpoint() == state


def test_nan_sentinels_are_json_safe_and_restore_exactly():
    engine = PoseEngine(load_config())
    engine.process(sample())
    checkpoint = engine.export_diagnostic_checkpoint()
    text = json.dumps(checkpoint, allow_nan=False)
    assert '__vqf_nan__' in text
    clone = PoseEngine.from_offline_checkpoint(json.loads(text))
    assert clone.export_diagnostic_checkpoint() == checkpoint
    assert math.isnan(clone.states[1]["filter"].state["motionBiasEstRLpState"][0])
    for tick in range(1, 500):
        row = sample(tick=tick, gyro=(tick % 200, 42, -13))
        engine.process(row)
        clone.process(row)
        np.testing.assert_array_equal(engine.states[1]["quat"], clone.states[1]["quat"])


@pytest.mark.parametrize("hand", ["right", "left"])
def test_full_real_vqf_checkpoint_continues_identically(ready_checkpoint, hand):
    from vt_glove_pose.config import fingerprint
    saved = deepcopy(ready_checkpoint)
    saved["config"]["hand"] = hand
    saved["config_fingerprint"] = fingerprint(saved["config"])
    original = PoseEngine.from_offline_checkpoint(saved)
    checkpoint = original.export_diagnostic_checkpoint()
    clone = PoseEngine.from_offline_checkpoint(json.loads(json.dumps(checkpoint, allow_nan=False)))
    assert original.get_snapshot(now=original.last_time) == clone.get_snapshot(now=clone.last_time)
    assert len(clone.states[1]["times"]) == 512
    assert np.linalg.norm(clone.states[1]["filter"].getBiasEstimate()[0]) > 0
    np.testing.assert_array_equal(clone.bias[1], np.zeros(3))
    for tick in range(550, 850):
        feed(original, tick, moving=True)
        feed(clone, tick, moving=True)
        if tick % 11 == 0:
            now = 10 + tick/120 + .03
            assert original.get_snapshot(now=now) == clone.get_snapshot(now=now)
    assert original.export_diagnostic_checkpoint() == clone.export_diagnostic_checkpoint()


def test_export_and_restore_own_independent_copies(ready_checkpoint):
    engine = PoseEngine.from_offline_checkpoint(ready_checkpoint)
    exported = engine.export_diagnostic_checkpoint()
    clone = PoseEngine.from_offline_checkpoint(exported)
    before = clone.export_diagnostic_checkpoint()
    exported["engine"]["states"]["1"]["filter"]["state"]["bias"][0] = 1
    exported["engine"]["axes_quality"]["1"]["linearity"] = 0
    exported["config"]["fingers"]["index"]["lengths_mm"][0] += 1
    engine.bias[1][0] = 2
    assert clone.export_diagnostic_checkpoint() == before
    assert ready_checkpoint["engine"]["axes_quality"]["1"]["linearity"] == .98


def test_active_capture_is_not_exported_or_silently_cancelled(ready_checkpoint):
    engine = PoseEngine.from_offline_checkpoint(ready_checkpoint)
    engine.begin_axis("index")
    active = engine.active
    with pytest.raises(ValueError, match="标定"):
        engine.export_diagnostic_checkpoint()
    assert engine.active is active and engine.neutral_ready


def test_normal_calibration_load_still_clears_live_zero(ready_checkpoint, tmp_path):
    engine = PoseEngine.from_offline_checkpoint(ready_checkpoint)
    calibration = tmp_path / "calibration.json"
    engine.save_calibration(calibration)
    live = PoseEngine(load_config())
    live.process(sample())
    live.load_calibration(calibration)
    assert not live.neutral_ready and live.zero == {} and not live.mount_confirmed
    assert set(live.mounts) == set(engine.mounts)
    # A new filter can legitimately coexist with the previously received quat.
    checkpoint = live.export_diagnostic_checkpoint()
    assert PoseEngine.from_offline_checkpoint(checkpoint).export_diagnostic_checkpoint() == checkpoint
    diagnostic = tmp_path / "checkpoint.json"
    diagnostic.write_text(json.dumps(ready_checkpoint), encoding="utf-8")
    before = live.export_diagnostic_checkpoint()
    with pytest.raises(ValueError):
        live.load_calibration(diagnostic)
    assert live.export_diagnostic_checkpoint() == before


def test_gap_invalid_acc_and_backward_time_restore_without_repairing_history(ready_checkpoint):
    engine = PoseEngine.from_offline_checkpoint(ready_checkpoint)
    engine.process(sample(stamp=engine.last_time + 1, acc=(0, 0, 0)))
    assert not engine.neutral_ready
    assert len(engine.states[1]["times"]) == 0
    assert engine.states[1]["count"] > 0
    restored = PoseEngine.from_offline_checkpoint(engine.export_diagnostic_checkpoint())
    assert restored.export_diagnostic_checkpoint() == engine.export_diagnostic_checkpoint()
    row = sample(stamp=engine.last_time - 1)
    engine.process(row)
    restored.process(row)
    assert engine.get_snapshot(now=engine.last_time) == restored.get_snapshot(now=restored.last_time)


@pytest.mark.parametrize("path,value", [
    (("kind",), "calibration"), (("schema_version",), True), (("schema_version",), 3),
    (("pose_algorithm",), LEGACY_ALGORITHM), (("pose_algorithm",), "unknown"),
    (("offline_only",), False), (("offline_only",), 1), (("vqf_version",), "0.0"),
    (("config_fingerprint",), "invalid"), (("engine", "active"), {}),
    (("engine", "neutral_ready"), 1), (("engine", "mount_confirmed"), "true"),
    (("engine", "gap_count"), -1), (("engine", "gap_count"), 2**53),
    (("engine", "last_time"), float("inf")), (("engine", "last_time"), 1),
    (("engine", "last_time"), 10**400), (("engine", "message"), "\ud800"),
    (("engine", "bias", "1"), [11., 0, 0]),
    (("engine", "zero", "1"), [2., 0, 0, 0]),
    (("engine", "neutral_acc", "1"), [0, 0, 0]),
    (("engine", "axis_sign_versions", "1"), True),
    (("engine", "axis_sign_versions", "1"), 3),
    (("engine", "thumb_pair_counts"), [0]), (("engine", "thumb_pair_counts"), [True, 0]),
    (("engine", "thumb_pair_counts"), [-1, 0]), (("engine", "thumb_pair_counts"), [2**53, 0]),
    (("engine", "thumb_pair_counts"), [551, 0]), (("engine", "thumb_estimator"), {}),
    (("engine", "thumb_common_capture"), {}),
    (("engine", "states", "1", "count"), True),
    (("engine", "states", "1", "times"), [12., 11.]),
    (("engine", "states", "1", "times"), [1.]*513),
    (("engine", "states", "1", "quat"), [0, 0, 0, 0]),
    (("engine", "states", "1", "gyro"), [1e10, 0, 0]),
    (("engine", "states", "1", "filter", "params", "tauAcc"), 0),
    (("engine", "states", "1", "filter", "params", "restBiasEstEnabled"), 1),
    (("engine", "states", "1", "filter", "sample_periods_s", "gyr"), .01),
    (("engine", "states", "1", "filter", "state", "biasP"), [0]*8),
    (("engine", "states", "1", "filter", "state", "gyrQuat"), [2, 0, 0, 0]),
    (("engine", "states", "1", "filter", "state", "bias"), [{"__vqf_nan__": True}, 0, 0]),
    (("engine", "states", "1", "filter", "state", "accLpState"), [{"__vqf_nan__": 1}]*6),
    (("engine", "states", "1", "filter", "state", "accLpState"), [float("nan")]*6),
    (("engine", "states", "1", "filter", "state", "accLpState"), [float("inf")]*6),
])
def test_malformed_checkpoint_is_rejected_without_mutating_source(ready_checkpoint, path, value):
    data = deepcopy(ready_checkpoint)
    cursor = data
    for key in path[:-1]:
        cursor = cursor[key]
    cursor[path[-1]] = value
    with pytest.raises(ValueError):
        PoseEngine.from_offline_checkpoint(data)
    assert ready_checkpoint["kind"] == "vt_glove_pose.engine_checkpoint"
    assert ready_checkpoint["engine"]["gap_count"] == 0


@pytest.mark.parametrize("node", ["0", "13", "01", 1, True])
def test_node_keys_are_canonical_strings_only(ready_checkpoint, node):
    data = deepcopy(ready_checkpoint)
    data["engine"]["states"][node] = data["engine"]["states"].pop("1")
    with pytest.raises(ValueError):
        PoseEngine.from_offline_checkpoint(data)


def test_unknown_missing_or_excessive_structure_is_rejected(ready_checkpoint):
    for path in [(), ("engine",), ("engine", "states", "1", "filter", "state")]:
        data = deepcopy(ready_checkpoint)
        cursor = data
        for key in path:
            cursor = cursor[key]
        cursor["surprise"] = 0
        with pytest.raises(ValueError):
            PoseEngine.from_offline_checkpoint(data)
    data = deepcopy(ready_checkpoint)
    del data["engine"]["bias"]["12"]
    with pytest.raises(ValueError):
        PoseEngine.from_offline_checkpoint(data)
    data = deepcopy(ready_checkpoint)
    data["engine"]["message"] = "x"*2_000_001
    with pytest.raises(ValueError):
        PoseEngine.from_offline_checkpoint(data)


def test_nondefault_filter_parameters_are_preserved(ready_checkpoint):
    engine = PoseEngine.from_offline_checkpoint(ready_checkpoint)
    engine.states[1]["filter"].setTauAcc(1.5)
    engine.states[1]["filter"].setRestBiasEstEnabled(False)
    data = engine.export_diagnostic_checkpoint()
    restored = PoseEngine.from_offline_checkpoint(data)
    assert restored.states[1]["filter"].params == engine.states[1]["filter"].params
    for tick in range(550, 570):
        feed(engine, tick, moving=True)
        feed(restored, tick, moving=True)
    assert restored.export_diagnostic_checkpoint() == engine.export_diagnostic_checkpoint()


def test_legacy_checkpoint_restores_legacy_mode_and_reexports_unchanged_schema():
    config = load_config()
    engine = PoseEngine(config, algorithm_mode=LEGACY_ALGORITHM)
    for tick in range(20):
        feed(engine, tick)
    engine.zero = {node: state["quat"].copy() for node, state in engine.states.items()}
    engine.neutral_acc = {node: np.array([0., 0., 1.]) for node in range(1, 13)}
    engine.mounts = {node: np.array([1., 0., 0., 0.]) for node in range(1, 13)}
    engine.axis_sign_versions = {node: 2 for node in engine.mounts}
    engine.neutral_ready = engine.mount_confirmed = True
    saved = engine.export_diagnostic_checkpoint()
    assert saved["schema_version"] == 1 and "pose_algorithm" not in saved
    assert "thumb_estimator" not in saved["engine"] and "thumb_pair_counts" not in saved["engine"]
    assert "thumb_common_capture" not in saved["engine"]
    restored = PoseEngine.from_offline_checkpoint(deepcopy(saved))
    assert restored.algorithm_mode == LEGACY_ALGORITHM
    assert restored.thumb_estimator is None
    assert restored.export_diagnostic_checkpoint() == saved
    for tick in range(20, 25):
        feed(engine, tick, moving=True)
        feed(restored, tick, moving=True)
        assert restored.get_snapshot(now=restored.last_time) == engine.get_snapshot(now=engine.last_time)
    assert restored.thumb_estimator is None
    assert restored.export_diagnostic_checkpoint() == engine.export_diagnostic_checkpoint()


@pytest.mark.parametrize("version", [1, 2])
def test_schema_does_not_silently_accept_other_algorithm_state(version):
    algorithm = LEGACY_ALGORITHM if version == 1 else THUMB_ALGORITHM
    saved = PoseEngine(load_config(), algorithm_mode=algorithm).export_diagnostic_checkpoint()
    saved["schema_version"] = 2 if version == 1 else 1
    with pytest.raises(ValueError):
        PoseEngine.from_offline_checkpoint(saved)


def test_consumed_counts_are_checked_against_corresponding_custom_thumb_nodes():
    config = load_config()
    config["fingers"]["thumb"]["proximal"], config["fingers"]["index"]["proximal"] = 3, 5
    engine = PoseEngine(config)
    engine.process(sample(node=3))
    saved = engine.export_diagnostic_checkpoint()
    saved["engine"]["thumb_pair_counts"] = [1, 0]
    restored = PoseEngine.from_offline_checkpoint(saved)
    assert restored.thumb_pair_counts == [1, 0]
    assert restored.export_diagnostic_checkpoint() == saved
    saved["engine"]["thumb_pair_counts"] = [0, 1]
    with pytest.raises(ValueError, match="received node"):
        PoseEngine.from_offline_checkpoint(saved)


def thumb_engine():
    engine = PoseEngine(load_config())
    for tick in range(20):
        for node in range(1, 13):
            engine.process(sample(node, tick, gyro=(0, 0, 0)))
    engine.zero = {node: state["quat"].copy() for node, state in engine.states.items()}
    engine.neutral_acc = {node: np.array([0., 0., 1.]) for node in range(1, 13)}
    nodes = [engine.config["fingers"]["thumb"][role] for role in ("proximal", "distal")]
    engine.mounts = {node: np.array([1., 0., 0., 0.]) for node in (*nodes, engine.config["palm_node"])}
    engine.axis_sign_versions = {node: 2 for node in engine.mounts}
    engine.neutral_ready = engine.mount_confirmed = True
    engine._reset_thumb_flexion()
    return engine


def thumb_row(engine, node, tick, *, repeat_host_times=False):
    config = engine.config
    distal = config["fingers"]["thumb"]["distal"]
    angle = math.radians((tick - 19) * .15) if node == distal else 0.
    gyro = (-18., 0, 0) if node == distal else (0., 0, 0)
    acc = (0, -math.sin(angle), math.cos(angle))
    values = np.rint(np.concatenate((np.asarray(gyro)/config["gyro_dps_per_lsb"],
                                     np.asarray(acc)/config["accel_g_per_lsb"]))).astype(int)
    stamp = 10 + (tick // 2 * 2 if repeat_host_times else tick) / 120
    return Sample(node, tuple(int(v) for v in values), stamp, 1700000000 + stamp)


@pytest.mark.parametrize("repeat_host_times", [False, True])
def test_active_thumb_and_unconsumed_half_pair_restore_exactly(repeat_host_times):
    engine = thumb_engine()
    for tick in range(20, 50):
        for node in range(1, 13):
            engine.process(thumb_row(engine, node, tick, repeat_host_times=repeat_host_times))
    pair = engine.config["fingers"]["thumb"]
    assert engine.thumb_estimator is not None
    assert engine.thumb_estimator.snapshot()["samples"] > 0
    previous_counts = list(engine.thumb_pair_counts)
    # Capture between the two IMU arrivals. Restoration must not double-count
    # the cached proximal sample or discard the next matching distal sample.
    engine.process(thumb_row(engine, pair["proximal"], 50, repeat_host_times=repeat_host_times))
    assert engine.thumb_pair_counts == previous_counts
    saved = engine.export_diagnostic_checkpoint()
    assert saved["engine"]["states"][str(pair["proximal"])]["count"] == previous_counts[0] + 1
    clone = PoseEngine.from_offline_checkpoint(json.loads(json.dumps(saved, allow_nan=False)))
    assert clone.export_diagnostic_checkpoint() == saved
    before = clone.export_diagnostic_checkpoint()
    for _ in range(20):
        clone.get_snapshot(now=clone.last_time)
    assert clone.export_diagnostic_checkpoint() == before
    pending = thumb_row(engine, pair["distal"], 50, repeat_host_times=repeat_host_times)
    engine.process(pending)
    clone.process(pending)
    for tick in range(51, 80):
        for node in range(1, 13):
            row = thumb_row(engine, node, tick, repeat_host_times=repeat_host_times)
            engine.process(row)
            clone.process(row)
        assert engine.get_snapshot(now=engine.last_time) == clone.get_snapshot(now=clone.last_time)
    assert engine.export_diagnostic_checkpoint() == clone.export_diagnostic_checkpoint()


def test_thumb_state_validation_happens_before_engine_or_native_filter_construction():
    from vt_glove_pose.diagnostic_state import from_checkpoint
    from vt_glove_pose.thumb_flexion import ThumbFlexionEstimator
    engine = PoseEngine(load_config())
    saved = engine.export_diagnostic_checkpoint()
    saved["engine"]["thumb_estimator"] = ThumbFlexionEstimator(120).export_state()

    class MustNotConstruct:
        def __init__(self, *_args, **_kwargs):
            pytest.fail("Checkpoint validation must precede engine construction")

    bad_rate = deepcopy(saved)
    bad_rate["engine"]["thumb_estimator"]["sample_rate_hz"] = 60
    with pytest.raises(ValueError, match="nominal rate"):
        from_checkpoint(MustNotConstruct, bad_rate)
    bad_state = deepcopy(saved)
    bad_state["engine"]["thumb_estimator"]["state"]["samples"] = True
    with pytest.raises(ValueError):
        from_checkpoint(MustNotConstruct, bad_state)


def common_capture(rows=30):
    return {key: [[1., -2., 3.] if key.endswith("gyro") else [0., 0., 1.] for _ in range(rows)]
            for key in ("prox_gyro", "dist_gyro", "prox_acc", "dist_acc")}


def test_pending_common_motion_capture_is_roundtripped_without_claiming_completed_axes():
    engine = PoseEngine(load_config())
    engine.thumb_common_capture = common_capture()
    saved = engine.export_diagnostic_checkpoint()
    clone = PoseEngine.from_offline_checkpoint(saved)
    assert clone.thumb_common_capture == engine.thumb_common_capture
    assert clone.mounts == {} and not clone.neutral_ready
    assert clone.export_diagnostic_checkpoint() == saved
    saved["engine"]["thumb_common_capture"]["prox_gyro"][0][0] = 99.
    engine.thumb_common_capture["prox_acc"][0][0] = .1
    assert clone.thumb_common_capture == common_capture()


@pytest.mark.parametrize("bad", [
    {}, common_capture(29), common_capture(2001),
    dict(common_capture(), prox_gyro=[[0., 0., 0.]] * 31),
    dict(common_capture(), prox_acc=[[0., 0.]] * 30),
    dict(common_capture(), prox_acc=[[0., 0., True]] * 30),
    dict(common_capture(), prox_acc=[[0., 0., 10.]] * 30),
    dict(common_capture(), prox_gyro=[[400., 0., 0.]] * 30),
    dict(common_capture(), unexpected=[]),
])
def test_bad_pending_common_capture_is_rejected(bad):
    saved = PoseEngine(load_config()).export_diagnostic_checkpoint()
    saved["engine"]["thumb_common_capture"] = bad
    with pytest.raises(ValueError):
        PoseEngine.from_offline_checkpoint(saved)
