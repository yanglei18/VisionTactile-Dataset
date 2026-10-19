"""Advisory UI feedback is not an acceptance gate or a calibration mutation."""
from copy import deepcopy
import json

import numpy as np
import pytest

from vt_glove_pose.calibration_feedback import thumb_capture_feedback
from vt_glove_pose.engine import PoseEngine
from test_thumb_calibration import make_capture
from test_thumb_joint_workflow import ready_engine, prepare_capture, finish_capture, saved_state, assert_same_saved


def setup_capture(target="thumb_common", method="robust_v2"):
    data, _ = make_capture()
    engine = ready_engine(data)
    capture = prepare_capture(engine, target, data)
    if method != "strict_v1":
        capture["calibration_method"] = method
    return engine, capture


def finish_injected_capture(engine):
    finish_capture(engine)
    # Injected capture rows did not pass through process(); do not invent
    # received samples or export a timestamp history longer than its count.
    for state in engine.states.values():
        state["times"].clear()
        state["times"].append(state["time"])


def test_legacy_command_snapshot_has_no_new_feedback_fields():
    engine, _ = setup_capture(method="strict_v1")
    assert "quality_feedback" not in engine.get_snapshot(now=engine.last_time)["calibration"]


def test_feedback_is_finite_advisory_and_preserves_mounts_and_capture():
    engine, capture = setup_capture()
    previous = saved_state(engine)
    rows = deepcopy(capture["rows"])
    feedback = engine.get_snapshot(now=engine.last_time)["calibration"]["quality_feedback"]
    assert feedback["advisory_only"] is True
    assert feedback["stage"] == "A" and feedback["status"] == "collecting"
    assert feedback["amplitude_consistent_fraction"] == 1
    json.dumps(feedback, allow_nan=False)
    assert_same_saved(engine, previous)
    for node in rows:
        np.testing.assert_equal(capture["rows"][node], rows[node])


def test_empty_and_unpaired_window_waits_without_division_or_nan():
    engine, capture = setup_capture()
    capture["rows"] = {}
    result = thumb_capture_feedback(capture, engine.config, engine.bias)
    assert result["status"] == "waiting"
    assert result["paired_samples"] == 0
    json.dumps(result, allow_nan=False)


def test_bad_speed_pair_has_neutral_wording_not_accusation():
    engine, capture = setup_capture()
    distal = engine.config["fingers"]["thumb"]["distal"]
    capture["rows"][distal] = [(t, engine.bias[distal].copy(), a, q)
                               for t, g, a, q in capture["rows"][distal]]
    result = thumb_capture_feedback(capture, engine.config, engine.bias)
    assert result["status"] == "needs_attention"
    assert "并非一定是动作问题" in result["message"]
    assert result["amplitude_consistent_fraction"] == 0


def test_saturated_advisory_does_not_silently_drop_samples():
    engine, capture = setup_capture()
    distal = engine.config["fingers"]["thumb"]["distal"]
    capture["rows"][distal][-1][1][0] = 32764 * engine.config["gyro_dps_per_lsb"]
    result = thumb_capture_feedback(capture, engine.config, engine.bias)
    assert result["status"] == "needs_attention"
    assert "量程" in result["message"]
    assert len(capture["rows"][distal]) == 960


def test_b_near_segment_feedback():
    engine, capture = setup_capture()
    capture["target"] = "thumb_hinge"
    proximal = engine.config["fingers"]["thumb"]["proximal"]
    for _, gyro, _, _ in capture["rows"][proximal]:
        gyro[:] = engine.bias[proximal] + [3., 0, 0]
    result = thumb_capture_feedback(capture, engine.config, engine.bias)
    assert result["stage"] == "B" and result["proximal_stable_window"] is False
    assert result["status"] == "needs_attention"


@pytest.mark.parametrize("method", [None, "anything", "strict_v2"])
def test_unknown_method_rejects_without_touching_engine(method):
    data, _ = make_capture()
    engine = ready_engine(data)
    with pytest.raises(ValueError, match="未知"):
        engine.begin_axis("thumb_common", calibration_method=method)
    assert engine.active is None


def test_robust_method_is_only_for_joint_thumb():
    data, _ = make_capture()
    engine = ready_engine(data)
    with pytest.raises(ValueError, match="仅适用"):
        engine.begin_axis("index", calibration_method="robust_v2")
    assert engine.active is None


def test_pending_robust_a_roundtrips_and_rejects_mixed_b_without_discarding_a():
    engine, _ = setup_capture()
    finish_injected_capture(engine)
    assert engine.thumb_common_capture is not None, engine.message
    assert engine.get_snapshot(now=engine.last_time)["calibration"]["quality_feedback"]["status"] == "ready_for_b"
    pending = deepcopy(engine.thumb_common_capture)
    checkpoint = engine.export_diagnostic_checkpoint()
    restored = PoseEngine.from_offline_checkpoint(checkpoint)
    assert restored.export_diagnostic_checkpoint() == checkpoint
    with pytest.raises(ValueError, match="方法不一致"):
        restored.begin_axis("thumb_hinge")
    assert restored.active is None and restored.thumb_common_capture == pending
    restored.begin_axis("thumb_hinge", calibration_method="robust_v2")
    restored.cancel_calibration()
    assert restored.thumb_common_capture == pending


def test_unknown_checkpoint_capture_method_rejected():
    engine, _ = setup_capture()
    finish_injected_capture(engine)
    assert engine.thumb_common_capture is not None, engine.message
    checkpoint = engine.export_diagnostic_checkpoint()
    checkpoint["engine"]["thumb_common_capture"]["calibration_method"] = "guess"
    with pytest.raises(ValueError, match="calibration method"):
        PoseEngine.from_offline_checkpoint(checkpoint)


def test_new_neutral_does_not_claim_ready_for_b_while_busy():
    engine, _ = setup_capture()
    finish_injected_capture(engine)
    assert engine.thumb_common_capture is not None, engine.message
    engine.begin_neutral(confirm_mount=True)
    calibration = engine.get_snapshot(now=engine.last_time)["calibration"]
    assert calibration["active"]["kind"] == "neutral"
    assert "quality_feedback" not in calibration


def test_b_failure_feedback_preserves_a_and_mentions_failure():
    engine, _ = setup_capture()
    finish_injected_capture(engine)
    engine.begin_axis("thumb_hinge", calibration_method="robust_v2")
    engine._finish_calibration()  # No samples: it must fail without losing A.
    feedback = engine.get_snapshot(now=engine.last_time)["calibration"]["quality_feedback"]
    assert feedback["status"] == "ready_for_b"
    assert "失败原因" in feedback["message"]
    assert engine.thumb_common_capture is not None
