"""Calibration/state regressions plus real-VQF synthetic inertial checks.

Fake filters isolate operator gating, transactional calibration and artifacts.
The final tests use the installed VQF implementation; nothing opens hardware.
"""

from copy import deepcopy
import json
import math
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from vt_glove_pose.config import fingerprint, load_config, validate_config, write_json_exclusive
from vt_glove_pose.engine import PoseEngine
from vt_glove_pose.math3d import from_axis_angle, rotate, to_matrix
from vt_glove_pose.records import Sample


IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])
GRAVITY_RAW = (0, 0, 8197)


class FakeVQF:
    def __init__(self, sample_period, **kwargs):
        self.sample_period = sample_period
        self.kwargs = kwargs
        self.bias = np.zeros(3)
        self.updates = []
        self.quat = IDENTITY.copy()

    def setBiasEstimate(self, bias):
        self.bias = np.array(bias, dtype=float)

    def update(self, gyro, acceleration):
        self.updates.append((gyro.copy(), acceleration.copy()))

    def getQuat6D(self):
        return self.quat.copy()


def make_sample(node=1, stamp=10.0, gyro=(0, 0, 0), acceleration=GRAVITY_RAW):
    return Sample(node, tuple(gyro) + tuple(acceleration), stamp, 1_700_000_000.0 + stamp, 0)


def prime(engine, stamp=10.0, nodes=range(1, 13)):
    for node in nodes:
        engine.process(make_sample(node, stamp))


def index_nodes(engine):
    finger = engine.config["fingers"]["index"]
    return finger["proximal"], finger["distal"]


def fill_capture(engine, gyros=None, accelerations=None, omit=()):
    """Populate the active capture at 120 Hz without confounding filter math.

    Gyro/acceleration callback arguments are (node, index, total); their return
    values are in physical dps/g, just like PoseEngine's captured rows.
    """
    capture = engine.active
    count = int((capture["end"] - capture["start"]) * engine.config["sample_rate_hz"])
    for node in range(1, 13):
        if node in omit:
            continue
        rows = []
        for index in range(count):
            stamp = capture["start"] + index / engine.config["sample_rate_hz"]
            gyro = gyros(node, index, count) if gyros else engine.bias[node]
            acc = accelerations(node, index, count) if accelerations else [0, 0, 1]
            rows.append((stamp, np.asarray(gyro, dtype=float).copy(),
                         np.asarray(acc, dtype=float).copy(), IDENTITY.copy()))
        capture["rows"][node] = rows


def calibration_state(engine):
    return {
        "bias": {n: v.copy() for n, v in engine.bias.items()},
        "zero": {n: v.copy() for n, v in engine.zero.items()},
        "neutral_acc": {n: v.copy() for n, v in engine.neutral_acc.items()},
        "mounts": {n: v.copy() for n, v in engine.mounts.items()},
        "axes_quality": deepcopy(engine.axes_quality),
    }


def assert_calibration_equal(engine, expected):
    for field in ("bias", "zero", "neutral_acc", "mounts"):
        actual = getattr(engine, field)
        assert set(actual) == set(expected[field])
        for node in actual:
            np.testing.assert_array_equal(actual[node], expected[field][node])
    assert engine.axes_quality == expected["axes_quality"]


@pytest.fixture
def engine(monkeypatch):
    monkeypatch.setitem(sys.modules, "vqf", SimpleNamespace(VQF=FakeVQF))
    return PoseEngine(load_config())


@pytest.fixture
def ready(engine):
    prime(engine)
    engine.neutral_ready = True
    engine.mount_confirmed = True
    engine.zero = {node: IDENTITY.copy() for node in range(1, 13)}
    engine.neutral_acc = {node: np.array([0.0, 0.0, 1.0]) for node in range(1, 13)}
    return engine


class TestConfig:
    def test_default_mapping_and_copy_are_explicit(self):
        config = load_config()
        assert config["sample_rate_hz"] == 120
        assert config["palm_node"] == 12
        assert config["auxiliary_node"] == 11
        assert not config["units_verified"]
        validated = validate_config(config)
        config["fingers"]["index"]["lengths_mm"][0] += 1
        assert config != validated

    @pytest.mark.parametrize("field,value", [
        ("hand", "both"), ("hand", None), ("schema_version", True),
        ("schema_version", 2), ("sample_rate_hz", float("nan")),
        ("sample_rate_hz", float("inf")), ("sample_rate_hz", 0),
        ("sample_rate_hz", True), ("gyro_dps_per_lsb", -1),
        ("accel_g_per_lsb", 0), ("stale_after_s", 0), ("dip_ratio", 1.1),
        ("units_verified", "yes"), ("sample_rate_verified", 1),
        ("palm_mount_verified", None), ("estimate_dip", "false"),
        ("hand_side_confirmed", 0), ("palm_node", True),
        ("auxiliary_node", 12), ("fingers", None),
    ])
    def test_invalid_top_level_values(self, field, value):
        config = load_config()
        config[field] = value
        with pytest.raises(ValueError):
            validate_config(config)

    @pytest.mark.parametrize("field,value", [
        ("lengths_mm", None), ("lengths_mm", [30, 20]),
        ("lengths_mm", [30, float("nan"), 20]),
        ("base_mm", [0, 0]), ("base_mm", None),
        ("neutral_splay_deg", 100), ("anatomy_verified", "yes"),
        ("proximal", 1), ("distal", 12),
    ])
    def test_invalid_finger_values_and_duplicate_nodes(self, field, value):
        config = load_config()
        config["fingers"]["index"][field] = value
        with pytest.raises(ValueError):
            validate_config(config)

    def test_missing_finger_and_nonfinite_extra_fields(self):
        config = load_config()
        del config["fingers"]["thumb"]
        with pytest.raises(ValueError):
            validate_config(config)
        config = load_config()
        config["extra"] = {"nonfinite": float("nan")}
        with pytest.raises(ValueError):
            validate_config(config)

    def test_fingerprint_is_order_independent_and_changes_with_geometry(self):
        config = load_config()
        rearranged = dict(reversed(list(config.items())))
        assert fingerprint(config) == fingerprint(rearranged)
        rearranged = deepcopy(rearranged)
        rearranged["fingers"]["index"]["lengths_mm"][0] += 1
        assert fingerprint(config) != fingerprint(rearranged)

    def test_json_writer_is_exclusive_and_rejects_nonfinite(self, tmp_path):
        path = tmp_path / "calibration.json"
        write_json_exclusive(path, {"ok": True})
        original = path.read_bytes()
        with pytest.raises(FileExistsError):
            write_json_exclusive(path, {"ok": False})
        assert path.read_bytes() == original
        with pytest.raises(ValueError):
            write_json_exclusive(tmp_path / "bad.json", {"bad": float("nan")})
        assert not (tmp_path / "bad.json").exists()


class TestNeutral:
    def test_no_automatic_zero_and_mount_confirmation_is_required(self, engine):
        prime(engine)
        assert not engine.neutral_ready
        assert engine.active is None
        with pytest.raises(ValueError, match="确认"):
            engine.begin_neutral()
        assert not engine.mount_confirmed
        assert engine.active is None

    def test_all_twelve_fresh_nodes_required(self, engine):
        prime(engine, nodes=range(1, 12))
        with pytest.raises(ValueError, match="12"):
            engine.begin_neutral(confirm_mount=True)
        prime(engine, stamp=11, nodes=[12])
        with pytest.raises(ValueError, match="12"):
            engine.begin_neutral(confirm_mount=True)

    @pytest.mark.parametrize("duration", [None, "5", 2, 16, float("nan"), float("inf"), True])
    def test_invalid_duration(self, engine, duration):
        prime(engine)
        with pytest.raises(ValueError):
            engine.begin_neutral(duration=duration, confirm_mount=True)

    def test_five_seconds_static_all_nodes_produces_bias_zero_and_gravity(self, engine):
        prime(engine)
        engine.begin_neutral(duration=5, confirm_mount=True)
        assert engine.active["start"] == 10
        assert engine.active["end"] == 15
        assert not engine.neutral_ready
        for tick in range(1, 601):
            stamp = 10 + tick / 120
            for node in range(1, 13):
                engine.process(make_sample(node, stamp, gyro=(100, -200, 50)))
            if tick < 600:
                assert not engine.neutral_ready
        assert engine.active is None
        assert engine.neutral_ready
        assert engine.mount_confirmed
        assert set(engine.zero) == set(range(1, 13))
        assert engine.mounts == {}  # A zero is not an installation-axis calibration.
        for node in range(1, 13):
            np.testing.assert_allclose(engine.bias[node], np.array([100, -200, 50]) * 0.00875)
            np.testing.assert_allclose(engine.zero[node], IDENTITY)
            np.testing.assert_allclose(engine.neutral_acc[node], [0, 0, 8197 * 0.000122])
            np.testing.assert_allclose(engine.states[node]["filter"].bias, np.deg2rad(engine.bias[node]))

    @pytest.mark.parametrize("failure", ["motion", "large_bias", "accel_scale", "accel_motion", "missing_node"])
    def test_rejection_does_not_partially_replace_previous_calibration(self, ready, failure):
        ready.bias = {n: np.array([0.25, 0.5, 0.75]) for n in range(1, 13)}
        ready.mounts[1] = from_axis_angle([0, 0, 1], 0.3)
        ready.axes_quality[1] = {"saved": True}
        previous = calibration_state(ready)
        ready.begin_neutral(duration=5, confirm_mount=True)

        def gyro(node, index, count):
            if node == 12 and failure == "motion":
                return [5 * (-1) ** index, 0, 0]
            if node == 12 and failure == "large_bias":
                return [11, 0, 0]
            return [1, 2, 3]

        def acc(node, index, count):
            if node == 12 and failure == "accel_scale":
                return [0, 0, 2]
            if node == 12 and failure == "accel_motion":
                return [0.3 * (-1) ** index, 0, 1]
            return [0, 0, 1]

        fill_capture(ready, gyro, acc, omit=[12] if failure == "missing_node" else [])
        ready._finish_calibration()
        assert ready.active is None
        assert ready.neutral_ready
        assert "未通过" in ready.message
        assert_calibration_equal(ready, previous)

    def test_axes_need_neutral_and_overlapping_capture_is_rejected(self, engine):
        prime(engine)
        with pytest.raises(ValueError, match="归零"):
            engine.begin_axis("index")
        with pytest.raises(ValueError, match="未知"):
            engine.begin_axis("not-a-finger")
        engine.begin_neutral(confirm_mount=True)
        with pytest.raises(ValueError, match="正在进行"):
            engine.begin_neutral(confirm_mount=True)
        engine.cancel_calibration()
        assert engine.active is None
        assert not engine.neutral_ready


class TestFunctionalAxes:
    @pytest.mark.parametrize("first_sign", [1, -1])
    def test_pca_axes_use_first_instructed_flexion_for_sign(self, ready, first_sign):
        proximal, distal = index_nodes(ready)
        axes = {proximal: np.array([1.0, 2.0, 0.0]) / np.sqrt(5),
                distal: np.array([2.0, -1.0, 0.0]) / np.sqrt(5)}
        ready.bias = {node: np.array([0.4, -0.2, 0.1]) for node in range(1, 13)}
        ready.begin_axis("index", duration=5)

        def gyro(node, index, count):
            motion = axes[node] * first_sign * (30 if index < count // 2 else -30) if node in axes else np.zeros(3)
            return ready.bias[node] + motion

        fill_capture(ready, gyro)
        ready._finish_calibration()
        assert set(ready.mounts) == {proximal, distal}
        for node, axis in axes.items():
            np.testing.assert_allclose(rotate(ready.mounts[node], axis * first_sign), [-1, 0, 0], atol=1e-12)
            np.testing.assert_allclose(rotate(ready.mounts[node], [0, 0, 1]), [0, 0, 1], atol=1e-12)
            assert ready.axes_quality[node]["linearity"] > 0.999999
            assert ready.axes_quality[node]["moving_samples"] > 25
            assert ready.axes_quality[node]["sign_basis"] == "first instructed flexion"

    @pytest.mark.parametrize("failure", ["palm_motion", "nonlinear", "parallel_gravity", "too_slow", "too_fast", "few_moving"])
    def test_rejection_is_transactional_for_both_finger_nodes(self, ready, failure):
        proximal, distal = index_nodes(ready)
        ready.mounts = {proximal: from_axis_angle([0, 0, 1], 0.2), distal: IDENTITY.copy()}
        ready.axes_quality = {proximal: {"previous": 1}, distal: {"previous": 2}}
        previous = calibration_state(ready)
        ready.begin_axis("index", duration=5)

        def gyro(node, index, count):
            if node == 12 and failure == "palm_motion":
                return [6, 0, 0]
            if node not in (proximal, distal):
                return [0, 0, 0]
            if node == proximal:  # First node succeeds; the failing second must not half-save.
                return [30, 0, 0]
            if failure == "nonlinear":
                return [30, 0, 0] if index % 2 else [0, 30, 0]
            if failure == "parallel_gravity":
                return [0, 0, 30]
            if failure == "too_slow":
                return [4, 0, 0]
            if failure == "too_fast":
                return [250, 0, 0]
            if failure == "few_moving":
                return [30, 0, 0] if 60 <= index < 80 else [0, 0, 0]
            return [30, 0, 0]

        fill_capture(ready, gyro)
        ready._finish_calibration()
        assert "未通过" in ready.message
        assert_calibration_equal(ready, previous)

    def test_palm_axis_is_independent_and_auxiliary_is_not_a_reference(self, ready):
        ready.begin_axis("palm", duration=5)
        fill_capture(ready, lambda node, index, count: [20, 0, 0] if node in (11, 12) else [0, 0, 0])
        ready._finish_calibration()
        assert set(ready.mounts) == {12}
        assert 11 not in ready.axes_quality


class TestTimingAndInvalidData:
    def test_nominal_sample_period_and_units_not_host_batch_dt(self, engine):
        for _ in range(3):
            engine.process(make_sample(1, 10, gyro=(100, 0, 0)))
        state = engine.states[1]
        filt = state["filter"]
        assert filt.sample_period == 1 / 120
        assert filt.kwargs["motionBiasEstEnabled"] is False
        assert len(filt.updates) == 3
        np.testing.assert_allclose(filt.updates[-1][0], np.deg2rad([0.875, 0, 0]))
        np.testing.assert_allclose(filt.updates[-1][1], [0, 0, 8197 * 0.000122 * 9.80665])
        assert state["count"] == 3
        assert engine.invalid_count == 0
        assert engine.gap_count == 0

    def test_zero_acceleration_first_sample_then_valid_sample_recovers(self, engine):
        engine.process(make_sample(acceleration=(0, 0, 0)))
        assert engine.invalid_count == 1
        assert engine.last_time is None
        engine.process(make_sample(stamp=10.01))
        assert engine.states[1]["count"] == 1
        assert engine.last_time == 10.01

    def test_zero_acceleration_does_not_refresh_freshness(self, engine):
        engine.process(make_sample(stamp=10))
        engine.process(make_sample(stamp=10.1, acceleration=(0, 0, 0)))
        assert engine.states[1]["time"] == 10
        assert engine.states[1]["count"] == 1
        assert engine.last_time == 10
        assert engine.invalid_count == 1

    @pytest.mark.parametrize("change", [
        {"values": [float("nan"), 0, 0, 0, 0, 8197]},
        {"values": [0, 0, 0, 0, 0, float("inf")]},
        {"values": [0] * 5}, {"node_id": 13},
        {"host_monotonic_s": float("nan")}, {"host_monotonic_s": float("inf")},
    ])
    def test_nonfinite_or_malformed_external_records_are_rejected(self, engine, change):
        # Public Sample rejects these already; check the engine's defense too.
        values = dict(node_id=1, values=(0, 0, 0, 0, 0, 8197), host_monotonic_s=10)
        values.update(change)
        engine.process(SimpleNamespace(**values))
        assert engine.invalid_count == 1
        assert engine.last_time is None
        assert not engine.states

    def test_reordering_invalidates_zero_and_cancels_active_capture(self, ready):
        proximal, _ = index_nodes(ready)
        ready.begin_axis("index", duration=5)
        ready.process(make_sample(proximal, 9.9))
        assert ready.invalid_count == 1
        assert not ready.neutral_ready
        assert ready.active is None
        assert ready.states[proximal]["time"] == 10

    def test_gap_restarts_filter_invalidates_zero_but_keeps_mount(self, ready):
        proximal, _ = index_nodes(ready)
        ready.mounts[proximal] = IDENTITY.copy()
        ready.begin_axis("index", duration=5)
        old_filter = ready.states[proximal]["filter"]
        ready.process(make_sample(proximal, 10.6))
        assert ready.gap_count == 1
        assert not ready.neutral_ready
        assert ready.active is None
        assert set(ready.mounts) == {proximal}
        assert ready.states[proximal]["filter"] is not old_filter
        assert list(ready.states[proximal]["times"]) == [10.6]

    def test_snapshot_marks_stale_nodes_and_cancels_paused_capture(self, ready):
        ready.begin_axis("index", duration=5)
        snapshot = ready.get_snapshot(now=10.6)
        assert all(not node["fresh"] for node in snapshot["nodes"].values())
        assert all(node["rate_hz"] == 0 for node in snapshot["nodes"].values())
        assert snapshot["calibration"]["active"] is None
        assert ready.active is None
        assert snapshot["timing_basis"] == "configured_nominal_period_no_device_timestamps"

    def test_near_int16_gyro_limit_is_reported(self, engine):
        engine.process(make_sample(gyro=(32700, 0, 0)))
        assert engine.near_limit_count == 1
        assert any("int16" in warning for warning in engine.get_snapshot(now=10)["warnings"])


class TestPersistence:
    def save_valid(self, engine, tmp_path):
        proximal, _ = index_nodes(engine)
        engine.mounts = {proximal: IDENTITY.copy(), engine.config["palm_node"]: from_axis_angle([0, 0, 1], 0.3)}
        engine.axes_quality = {proximal: {"linearity": 1.0, "moving_samples": 300}}
        path = tmp_path / "calibration.json"
        engine.save_calibration(path)
        return path

    def test_save_requires_zero_and_no_active_capture(self, engine, tmp_path):
        with pytest.raises(ValueError):
            engine.save_calibration(tmp_path / "no-zero.json")
        prime(engine)
        engine.neutral_ready = True
        engine.begin_neutral(confirm_mount=True)
        with pytest.raises(ValueError):
            engine.save_calibration(tmp_path / "active.json")
        assert not list(tmp_path.iterdir())

    def test_save_is_exclusive_and_load_requires_fresh_confirmed_zero(self, ready, tmp_path):
        proximal, _ = index_nodes(ready)
        path = self.save_valid(ready, tmp_path)
        previous_bytes = path.read_bytes()
        payload = json.loads(previous_bytes)
        assert payload["requires_neutral_on_load"] is True
        assert payload["heading_drift_corrected"] is False
        assert "zero" not in payload
        with pytest.raises(FileExistsError):
            ready.save_calibration(path)
        assert path.read_bytes() == previous_bytes
        old_filter = ready.states[proximal]["filter"]
        expected_mounts = {n: q.copy() for n, q in ready.mounts.items()}
        ready.load_calibration(path)
        assert not ready.neutral_ready
        assert not ready.mount_confirmed
        assert ready.zero == {}
        assert ready.axes_quality == {}
        assert ready.states[proximal]["filter"] is not old_filter
        for node, quat in expected_mounts.items():
            np.testing.assert_allclose(ready.mounts[node], quat)
        with pytest.raises(ValueError, match="归零"):
            ready.begin_axis("index")
        with pytest.raises(ValueError, match="确认"):
            ready.begin_neutral()
        assert ready._orientations() == {}

    def test_configuration_fingerprint_mismatch_preserves_state(self, ready, tmp_path):
        path = self.save_valid(ready, tmp_path)
        previous = calibration_state(ready)
        payload = json.loads(path.read_text())
        payload["config_fingerprint"] = "different node layout or sensitivity"
        bad = tmp_path / "mismatch.json"
        bad.write_text(json.dumps(payload))
        with pytest.raises(ValueError, match="不匹配"):
            ready.load_calibration(bad)
        assert ready.neutral_ready
        assert_calibration_equal(ready, previous)

    @pytest.mark.parametrize("damage", [
        "bad_schema", "missing_bias", "extra_node", "bad_bias", "nan_bias",
        "missing_gravity", "zero_gravity", "bad_shape", "zero_quaternion",
        "nonunit_quaternion", "nan_quaternion", "nonobject", "invalid_json",
    ])
    def test_corrupt_calibration_is_rejected_transactionally(self, ready, tmp_path, damage):
        proximal_key = str(index_nodes(ready)[0])
        path = self.save_valid(ready, tmp_path)
        previous = calibration_state(ready)
        data = json.loads(path.read_text())
        if damage == "bad_schema":
            data["schema_version"] = 2
        elif damage == "missing_bias":
            del data["gyro_bias_dps"]["12"]
        elif damage == "extra_node":
            data["sensor_to_segment_wxyz"]["13"] = [1, 0, 0, 0]
        elif damage == "bad_bias":
            data["gyro_bias_dps"]["12"] = [11, 0, 0]
        elif damage == "nan_bias":
            data["gyro_bias_dps"]["12"] = [float("nan"), 0, 0]
        elif damage == "missing_gravity":
            del data["neutral_acc_g"]["12"]
        elif damage == "zero_gravity":
            data["neutral_acc_g"]["12"] = [0, 0, 0]
        elif damage == "bad_shape":
            data["sensor_to_segment_wxyz"][proximal_key] = [1, 0, 0]
        elif damage == "zero_quaternion":
            data["sensor_to_segment_wxyz"][proximal_key] = [0, 0, 0, 0]
        elif damage == "nonunit_quaternion":
            data["sensor_to_segment_wxyz"][proximal_key] = [2, 0, 0, 0]
        elif damage == "nan_quaternion":
            data["sensor_to_segment_wxyz"][proximal_key] = [float("nan"), 0, 0, 0]
        elif damage == "nonobject":
            data = []
        bad = tmp_path / "corrupt.json"
        bad.write_text("{bad json" if damage == "invalid_json" else json.dumps(data))
        with pytest.raises(ValueError):
            ready.load_calibration(bad)
        assert ready.neutral_ready
        assert_calibration_equal(ready, previous)

    def test_load_during_capture_is_forbidden(self, ready, tmp_path):
        path = self.save_valid(ready, tmp_path)
        ready.begin_axis("index", duration=5)
        with pytest.raises(ValueError, match="取消"):
            ready.load_calibration(path)
        assert ready.active is not None


class TestActualVQF:
    def test_static_gravity_real_filter_remains_finite_and_level(self):
        engine = PoseEngine(load_config())
        assert engine._vqf is not FakeVQF
        for tick in range(600):
            engine.process(make_sample(1, 10 + tick / 120))
        quat = engine.states[1]["quat"]
        assert np.isfinite(quat).all()
        np.testing.assert_allclose(np.linalg.norm(quat), 1, atol=1e-12)
        np.testing.assert_allclose(rotate(quat, [0, 0, 1]), [0, 0, 1], atol=1e-8)
        assert not engine.neutral_ready

    def test_synthetic_yaw_uses_120_hz_even_for_duplicate_parse_times(self):
        engine = PoseEngine(load_config())
        # 10286 * 0.00875 = 90.0025 deg/s; rotating around gravity leaves acc fixed.
        for _ in range(120):
            engine.process(make_sample(1, 10, gyro=(0, 0, 10286)))
        rotation = to_matrix(engine.states[1]["quat"])
        yaw = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
        assert yaw == pytest.approx(90.0025, abs=0.2)
        np.testing.assert_allclose(rotation[:, 2], [0, 0, 1], atol=1e-8)
        assert engine.invalid_count == 0
        assert engine.gap_count == 0
