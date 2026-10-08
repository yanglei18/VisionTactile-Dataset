"""Input-integrity faults must not masquerade as a fresh 120 Hz pose."""
from copy import deepcopy
import json

import pytest

from vt_fsglove12.health_contract import blocked_input_nodes, calibration_input_ready, result_input_health
from test_pose_export import inputs, state
from test_live_ui import window, make_measured


def report(node="12", status="faulted"):
    return dict(enabled=True, faulted_nodes=[node] if status == "faulted" else [],
                recovering_nodes=[node] if status == "recovering" else [],
                nodes={node: dict(healthy=status == "healthy", state=status,
                    reasons=["frozen_bank"], observed_receive_rate_hz=6.15,
                    nominal_sample_rate_hz=120., raw_values=[1, 2, 3, 4, 5, 6],
                    signature="private wire fingerprint", status_byte=208)})


@pytest.mark.parametrize("phase", ["faulted", "recovering", "warming_up"])
def test_fault_cannot_be_relabelled_valid_by_complete_arrays_and_new_receive_time(phase):
    frame, pose, geometry = inputs()
    frame["input_health"] = report(status=phase)
    pose["source_fresh"] = True
    before = deepcopy((frame, pose, geometry))
    result = state(frame, pose, geometry)
    assert not result["pose_valid"]
    assert result["reason"] == "input_health_invalid"
    assert json.loads(result["metadata_json"])["source_fresh"] is False
    # Receive timestamps are still real evidence, not newly synthesized times.
    assert result["time_valid"]
    assert result["host_monotonic_ns"] == frame["nodes"]["12"]["host_monotonic_ns"]
    assert (frame, pose, geometry) == before
    exported = json.loads(result["quality_json"])["input_health"]
    assert "private" not in json.dumps(exported)
    assert "raw_values" not in json.dumps(exported) and "status_byte" not in json.dumps(exported)


def test_auxiliary_health_fault_does_not_disable_healthy_mapped_hand():
    frame, pose, geometry = inputs()
    frame["input_health"] = report(node="11")
    assert state(frame, pose, geometry)["pose_valid"]
    assert blocked_input_nodes(frame, {"12": 0}) == []


def test_required_health_report_absence_and_warmup_cannot_start_calibration():
    config = dict(input_health={"enabled": True}, node_to_slot={"12": 0})
    assert not calibration_input_ready({}, config)
    frame = dict(input_health=report(), nodes={"12": {"orientation_valid": True}})
    assert not calibration_input_ready(frame, config)
    frame["input_health"] = report(status="healthy")
    assert calibration_input_ready(frame, config)
    frame["nodes"]["12"]["orientation_valid"] = False
    assert not calibration_input_ready(frame, config)
    assert calibration_input_ready({}, {"input_health": {"enabled": False}})


def test_compact_health_is_an_independent_whitelisted_result():
    frame = {"input_health": report()}
    compact = result_input_health(frame)
    compact["nodes"]["12"]["reasons"].append("caller edit")
    assert frame["input_health"]["nodes"]["12"]["reasons"] == ["frozen_bank"]


def test_input_health_warning_does_not_require_a_fresh_node_or_an_unfiltered_peak():
    from vt_fsglove12.live_ui import solver_input_status
    frame = dict(input_health=report(), timestamp_s=10., nodes={})
    text, nodes = solver_input_status(frame, 10.01, connected=True)
    assert nodes == ["12"] and "冻结" in text and "暂停姿态积分" in text
    assert "疑似截顶" not in text


def test_confirmed_fault_keeps_last_hand_and_disables_recalibration_without_flash(window):
    import time
    w, controller, backend = window
    make_measured(controller)
    w.refresh()
    assert w._has_display_cache
    previous = w._last_pose
    w.config["input_health"] = {"enabled": True}
    for _ in range(5):
        now = time.monotonic()
        controller.state["runtime"].update(published_monotonic_s=now, last_sample_age_s=.001)
        controller.state["pose"]["timestamp_s"] = now  # A racing old good pose must not win.
        controller.state["orientation"] = dict(timestamp_s=now, nodes={}, input_health=report())
        w.refresh()
        assert w._last_pose == previous
        assert w.view.mesh.visible()
        assert "输入异常保护" in w.view_state.text()
        assert not any(button.isEnabled() for button in w.step_buttons.values())
        assert "暂停姿态积分" in w.input_status.text()
