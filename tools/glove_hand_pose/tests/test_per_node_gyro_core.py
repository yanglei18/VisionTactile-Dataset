"""Per-node sensitivity is configuration, not firmware-range verification."""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
from types import SimpleNamespace

import numpy as np
import pytest

from vt_glove_pose.calibration_feedback import thumb_capture_feedback
from vt_glove_pose.config import (fingerprint, gyro_scale, load_config,
                                  pair_gyro_scales, validate_config)
from vt_glove_pose.diagnostics import _RawStatistics, diagnose_session
from vt_glove_pose.engine import PoseEngine
from vt_glove_pose.records import Sample


def settings(overrides=None):
    config = load_config()
    config["gyro_dps_per_lsb_by_node"] = {"5": .0175, "6": .0175} if overrides is None else overrides
    return config


def sample(node, raw=20000, tick=0):
    return Sample(node, (raw, 0, 0, 0, 0, 8197), 10 + tick/120, 1000 + tick/120)


def test_fallback_and_explicit_scales_preserve_pair_order():
    config = validate_config(settings())
    assert gyro_scale(config, 1) == .00875
    assert gyro_scale(config, 5) == .0175
    assert pair_gyro_scales(config, 5, 3) == (.0175, .00875)
    assert pair_gyro_scales(config, 3, 5) == (.00875, .0175)
    assert config["units_verified"] is False


@pytest.mark.parametrize("overrides", [None, [], True, .0175, {5: .0175},
    {"05": .0175}, {"0": .0175}, {"13": .0175}, {"5.0": .0175},
    {" 5": .0175}, {"5": None}, {"5": True}, {"5": "0.0175"},
    {"5": 0}, {"5": -1}, {"5": 1e-7}, {"5": 11},
    {"5": float("nan")}, {"5": float("inf")}])
def test_invalid_map_is_rejected_before_json_coercion(overrides):
    config = load_config()
    config["gyro_dps_per_lsb_by_node"] = overrides
    with pytest.raises(ValueError, match="gyro_dps_per_lsb_by_node"):
        validate_config(config)


@pytest.mark.parametrize("node", [True, "5", 0, 13, 5.0, None])
def test_helpers_reject_ambiguous_node_ids(node):
    with pytest.raises(ValueError, match="node_id"):
        gyro_scale(settings(), node)


def test_config_copy_and_file_roundtrip(tmp_path):
    original = settings()
    validated = validate_config(original)
    original["gyro_dps_per_lsb_by_node"]["5"] = .035
    assert gyro_scale(validated, 5) == .0175
    path = tmp_path / "settings.json"
    path.write_text(json.dumps(validated), encoding="utf-8")
    assert load_config(path) == validated


def test_fingerprint_keeps_legacy_hash_for_empty_and_redundant_maps():
    config = load_config()
    relevant = deepcopy(config)
    for key in ("notes", "hand_side_confirmed", "palm_mount_verified", "units_verified", "sample_rate_verified"):
        relevant.pop(key, None)
    legacy = hashlib.sha256(json.dumps(relevant, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    assert fingerprint(config) == legacy
    assert fingerprint(settings({})) == legacy
    assert fingerprint(settings({str(n): .00875 for n in range(1, 13)})) == legacy
    assert fingerprint(settings()) != legacy
    assert fingerprint(settings({"6": .0175, "5": .0175, "3": .00875})) == fingerprint(settings())
    assert fingerprint(settings({"5": .0175})) != fingerprint(settings())


def test_engine_converts_only_overridden_nodes_and_keeps_raw_saturation():
    engine = PoseEngine(settings())
    for node in range(1, 13):
        engine.process(sample(node))
        expected = 350. if node in (5, 6) else 175.
        np.testing.assert_allclose(engine.states[node]["gyro"], [expected, 0., 0.], rtol=1e-14)
    assert engine.near_limit_count == 0
    for node in (1, 5, 6):
        engine.process(sample(node, 32767, tick=1))
    assert engine.near_limit_count == 3


def test_thumb_initialization_and_updates_use_each_raw_limit():
    engine = PoseEngine(settings({"5": .0175}))
    engine.neutral_ready = True
    engine.mounts = {n: np.array([1., 0., 0., 0.]) for n in (5, 6)}
    engine.neutral_acc = {n: np.array([0., 0., 1.]) for n in (5, 6)}
    engine.axis_sign_versions = {5: 2, 6: 2}
    engine.process(sample(5, 20000))  # 350 dps, valid for node 5.
    engine.process(sample(6, 10000))
    assert engine.thumb_estimator is not None
    calls = []
    engine.thumb_estimator = SimpleNamespace(update=lambda *a, **kw: calls.append(kw))
    engine.process(sample(5, 20000, 1))
    engine.process(sample(6, 10000, 1))
    assert calls[-1]["proximal_saturated"] is False
    assert calls[-1]["distal_saturated"] is False
    engine.process(sample(5, 20000, 2))
    engine.process(sample(6, 32767, 2))
    assert calls[-1]["proximal_saturated"] is False
    assert calls[-1]["distal_saturated"] is True


def capture_for_feedback(config, raw_prox, raw_dist):
    return {"target": "thumb_common", "rows": {
        node: [(10+i/120, np.array([raw*gyro_scale(config, node), 0., 0.]),
                np.array([0., 0., 1.]), np.array([1., 0., 0., 0.])) for i in range(60)]
        for node, raw in ((5, raw_prox), (6, raw_dist))}}


def test_feedback_saturation_uses_separate_node_sensitivity():
    config = settings({"5": .0175})
    bias = {5: np.zeros(3), 6: np.zeros(3)}
    feedback = thumb_capture_feedback(capture_for_feedback(config, 20000, 10000), config, bias)
    assert "量程" not in feedback["message"]
    feedback = thumb_capture_feedback(capture_for_feedback(config, 20000, 32767), config, bias)
    assert "量程" in feedback["message"]


def test_joint_fitter_keeps_raw_node_limits_before_shared_physical_cap(monkeypatch):
    config = settings({"5": .0175})
    engine = PoseEngine(config)
    capture = capture_for_feedback(config, 20000, 10000)
    # Replicate a complete second of test input; the fitter trims the edges.
    capture.update(start=10., end=11.)
    for node, raw in ((5, 20000), (6, 10000)):
        capture["rows"][node] = [(10+i/120, np.array([raw*gyro_scale(config, node), 0., 0.]),
                                  np.array([0., 0., 1.]), np.array([1., 0., 0., 0.]))
                                 for i in range(120)]
    calls = []
    monkeypatch.setattr("vt_glove_pose.thumb_calibration.fit_thumb_common_rotation",
                        lambda *args, **kwargs: calls.append((args, kwargs)))
    engine._finish_thumb_joint(capture, 30)
    assert len(calls) == 1
    assert calls[0][1]["gyro_saturation_dps"] == 32700*.0175
    assert engine.thumb_common_capture is not None
    # A higher-range partner cannot mask a clipped lower-range node.
    for row in capture["rows"][6]:
        row[1][0] = 32767*gyro_scale(config, 6)
    with pytest.raises(ValueError, match="近量程"):
        engine._finish_thumb_joint(capture, 30)
    assert len(calls) == 1


def test_checkpoint_roundtrip_and_continuation_allow_high_range_node():
    engine = PoseEngine(settings({"5": .0175}))
    for node in range(1, 13):
        engine.process(sample(node, 32767))
    saved = engine.export_diagnostic_checkpoint()
    clone = PoseEngine.from_offline_checkpoint(json.loads(json.dumps(saved)))
    assert clone.export_diagnostic_checkpoint() == saved
    for tick in range(1, 5):
        for node in range(1, 13):
            row = sample(node, 23000, tick)
            engine.process(row)
            clone.process(row)
    assert clone.export_diagnostic_checkpoint() == engine.export_diagnostic_checkpoint()


def test_checkpoint_bounds_do_not_apply_other_nodes_override():
    engine = PoseEngine(settings({"5": .0175}))
    engine.process(sample(5))
    engine.process(sample(6))
    saved = engine.export_diagnostic_checkpoint()
    saved["engine"]["states"]["6"]["gyro"][0] = 350.
    with pytest.raises(ValueError, match="gyro_dps"):
        PoseEngine.from_offline_checkpoint(saved)


def test_pending_common_capture_bounds_are_per_node():
    engine = PoseEngine(settings({"5": .0175}))
    engine.thumb_common_capture = {
        "prox_gyro": [[350., 0., 0.]] * 30, "dist_gyro": [[100., 0., 0.]] * 30,
        "prox_acc": [[0., 0., 1.]] * 30, "dist_acc": [[0., 0., 1.]] * 30}
    saved = engine.export_diagnostic_checkpoint()
    assert PoseEngine.from_offline_checkpoint(saved).export_diagnostic_checkpoint() == saved
    saved["engine"]["thumb_common_capture"]["dist_gyro"][0][0] = 350.
    with pytest.raises(ValueError, match="dist_gyro"):
        PoseEngine.from_offline_checkpoint(saved)


def test_changed_scale_rejects_stale_saved_mounts_without_mutating_engine(tmp_path):
    legacy = PoseEngine(load_config())
    legacy.neutral_ready = True
    legacy.mount_confirmed = True
    legacy.neutral_acc = {n: np.array([0., 0., 1.]) for n in range(1, 13)}
    path = tmp_path / "mounts.json"
    legacy.save_calibration(path)
    unchanged = PoseEngine(settings({"5": .00875}))
    unchanged.load_calibration(path)  # A redundant override is compatible.
    corrected = PoseEngine(settings())
    before = corrected.export_diagnostic_checkpoint()
    with pytest.raises(ValueError, match="配置不匹配"):
        corrected.load_calibration(path)
    assert corrected.export_diagnostic_checkpoint() == before


def test_changed_map_with_old_checkpoint_hash_is_rejected():
    saved = PoseEngine(load_config()).export_diagnostic_checkpoint()
    saved["config"]["gyro_dps_per_lsb_by_node"] = {"5": .0175}
    with pytest.raises(ValueError, match="fingerprint"):
        PoseEngine.from_offline_checkpoint(saved)


def test_legacy_raw_report_shape_is_preserved():
    legacy = _RawStatistics(load_config())
    mixed = _RawStatistics(settings())
    for node in (1, 5, 6):
        legacy.add(sample(node))
        mixed.add(sample(node))
    for node in ("1", "5", "6"):
        assert "gyro_dps_per_lsb_assumed" not in legacy.report()[node]
        assert mixed.report()[node]["gyro_dps_per_lsb_assumed"] == (.00875 if node == "1" else .0175)


def test_diagnostic_replay_reports_and_uses_mixed_sensitivities(tmp_path):
    engine = PoseEngine(settings())
    events = [{"type": "header", "kind": "vt_glove_pose.diagnostic_session", "schema_version": 1,
               "time_basis": "host_observed_not_sensor_latency", "checkpoint": engine.export_diagnostic_checkpoint()}]
    rows = [sample(n) for n in range(1, 13)]
    for row in rows:
        engine.process(row)
    events.extend([{"type": "batch", "samples": [asdict(s) for s in rows], "performance": {}},
                   {"type": "snapshot", "now_s": 10., "snapshot": engine.get_snapshot(now=10.)},
                   {"type": "end", "complete": True, "reason": "manual"}])
    path = tmp_path / "mixed.jsonl"
    path.write_text("".join(json.dumps(dict(event, seq=i)) + "\n" for i, event in enumerate(events)), encoding="utf-8")
    report = diagnose_session(path)
    assert report["reconstruction"]["reproducible"]
    assert report["configuration"]["gyro_dps_per_lsb_by_node"] == {"5": .0175, "6": .0175}
    assert report["configuration"]["units_verified"] is False
    assert report["nodes"]["5"]["gyro_dps_per_lsb_assumed"] == .0175
