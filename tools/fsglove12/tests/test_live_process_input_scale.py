"""Dual-mailbox completion acknowledgement; no serial device or child required."""
from copy import deepcopy
import queue
import time

import pytest

from test_live_process import InputRevisionRuntime, controller, wait_for
from vt_fsglove12.live_process import _idle, _publish


class RejectScaleRuntime(InputRevisionRuntime):
    """A full/unavailable runtime queue rejects, with no execution generation."""
    def submit(self, name, **args):
        if name == "use_input_scale_candidate":
            return False
        return super().submit(name, **args)


def snapshot(generation=4, *, connected=True):
    now = time.monotonic()
    value = _idle()
    value["runtime"].update(connected=connected, session_id="old-session",
        input_scale_generation=generation, active_input_contract={"generation": generation})
    value["orientation"] = dict(timestamp_s=now, session_id="old-session",
        nodes={"12": dict(age_s=0., sample_index=1, epoch=0)}) if connected else None
    value["pose"].update(timestamp_s=now, pose_available=connected, solver_success=connected)
    value["geometry"] = {"vertices_m": [[1., 2., 3.]]} if connected else None
    value["display_pose"] = {"display_only": True} if connected else None
    value["display_geometry"] = deepcopy(value["geometry"])
    value["calibration"].update(profile_status="calibrated" if connected else "not_calibrated",
        captured_steps=["base_pose"] if connected else [], clear_generation=generation)
    return value


def publish(value, data, seen=0, *, pose=True, inputs=True):
    if pose:
        assert _publish(value._mailbox, deepcopy(data), seen)
    if inputs:
        light = {key: deepcopy(data[key]) for key in ("runtime", "orientation")}
        assert _publish(value._input_mailbox, light, seen, input_only=True)


def assert_hidden(value):
    data = value.latest_snapshot()
    assert not data["runtime"]["connected"]
    assert data["runtime"]["input_scale_pending"]
    assert not data["pose"]["pose_available"]
    assert data["geometry"] is None
    assert data["display_pose"] is None and data["display_geometry"] is None
    assert data["orientation"] is None
    assert data["input_snapshot"]["orientation"] is None
    assert data["calibration"]["captured_steps"] == []
    return data


def test_input_scale_command_is_argumentless_and_never_connects_or_starts_a_child(controller):
    value = controller()
    assert not value.submit("use_input_scale_candidate", enabled=True)
    assert not value.submit("use_input_scale_candidate", path="untrusted.json")
    assert value.submit("use_input_scale_candidate")
    assert value.pid is None
    assert value._commands.get(timeout=1.) == (1, "use_input_scale_candidate", {})
    with pytest.raises(queue.Empty):
        value._commands.get(timeout=.02)


@pytest.mark.parametrize("ack_order", ["pose_first", "input_first"])
def test_queue_receipt_cannot_expose_old_pose_before_both_execution_acks(controller, ack_order):
    value = controller()
    old = snapshot()
    publish(value, old)
    assert value.latest_snapshot()["pose"]["pose_available"]
    original_config = deepcopy(value.config)
    assert value.submit("use_input_scale_candidate")
    assert value._pending_input_scale_generation == 5
    assert_hidden(value)
    assert not value.submit("connect", port="MUST_NOT_OPEN")

    # Child dequeue/submit acknowledgement still refers to old runtime state.
    publish(value, old, value._command_id)
    assert_hidden(value)
    changed = snapshot(5, connected=False)
    first_pose = ack_order == "pose_first"
    publish(value, changed, value._command_id, pose=first_pose, inputs=not first_pose)
    assert_hidden(value)
    publish(value, changed, value._command_id, pose=not first_pose, inputs=first_pose)
    actual = value.latest_snapshot()
    assert not actual["runtime"]["input_scale_pending"]
    assert value._pending_input_scale_generation is None
    assert not actual["runtime"]["connected"]
    assert not actual["pose"]["pose_available"]
    assert actual["display_geometry"] is None
    assert actual["runtime"]["active_input_contract"] == {"generation": 5}
    assert value.config == original_config, "GUI config remains stable; runtime contract owns input scale"
    assert value._commands.get(timeout=1.)[1] == "use_input_scale_candidate"
    with pytest.raises(queue.Empty):
        value._commands.get(timeout=.02)
    assert value.submit("connect", port="EXPLICIT_NEW_USER_ACTION")


def test_failed_completed_change_releases_pending_and_preserves_runtime_error(controller):
    value = controller()
    publish(value, snapshot())
    value.latest_snapshot()
    assert value.submit("use_input_scale_candidate")
    failed = snapshot(5, connected=False)
    failed["runtime"].update(error="candidate no longer matches input configuration",
                              active_input_contract={"generation": 4})
    publish(value, failed, value._command_id)
    result = value.latest_snapshot()
    assert value._pending_input_scale_generation is None
    assert not result["runtime"]["input_scale_pending"]
    assert result["runtime"]["error"] == failed["runtime"]["error"]
    assert not result["runtime"]["connected"] and result["geometry"] is None


def test_old_input_mailbox_cannot_return_after_pose_generation_has_completed(controller):
    value = controller()
    old = snapshot()
    publish(value, old)
    value.latest_snapshot()
    assert value.submit("use_input_scale_candidate")
    publish(value, snapshot(5, connected=False), value._command_id)
    value.latest_snapshot()
    assert value._pending_input_scale_generation is None
    publish(value, old, value._command_id, pose=False)
    inputs = value.latest_input_snapshot()
    assert not inputs["runtime"]["connected"]
    assert inputs["runtime"]["input_scale_pending"]
    assert inputs["orientation"] is None
    assert_hidden(value)


def test_multiple_explicit_requests_wait_for_latest_completed_generation(controller):
    value = controller()
    publish(value, snapshot())
    value.latest_snapshot()
    assert value.submit("use_input_scale_candidate")
    assert value.submit("use_input_scale_candidate")
    assert value._pending_input_scale_generation == 6
    publish(value, snapshot(5, connected=False), value._command_id)
    assert_hidden(value)
    publish(value, snapshot(6, connected=False), value._command_id)
    result = value.latest_snapshot()
    assert not result["runtime"]["input_scale_pending"]
    assert value._pending_input_scale_generation is None


def test_full_command_queue_does_not_create_a_phantom_input_scale_transaction(controller):
    value = controller()
    class FullQueue:
        def put_nowait(self, command):
            raise queue.Full
    real_queue = value._commands
    try:
        value._commands = FullQueue()
        assert not value.submit("use_input_scale_candidate")
        assert value._pending_input_scale_generation is None
        assert value._input_scale_generation_floor == 0
        assert value._command_id == 0
    finally:
        value._commands = real_queue


def test_actual_child_queue_rejection_finishes_request_without_config_change_or_retry(controller):
    value = controller(runtime_factory=RejectScaleRuntime).start()
    assert value.submit("connect", port="FAKE")
    wait_for(lambda: value.latest_snapshot()["runtime"].get("connected"))
    for count in (1, 2):
        assert value.submit("use_input_scale_candidate")
        assert_hidden(value)

        def completed():
            data = value.latest_snapshot()
            return data if (value._pending_input_scale_generation is None
                            and data["runtime"].get("input_scale_rejected_count") == count) else None

        actual = wait_for(completed)
        assert actual["runtime"].get("input_scale_generation", 0) == 0
        assert "未自动重试" in actual["runtime"]["error"]
        assert actual["runtime"]["input_scale_command_error"] == actual["runtime"]["error"]
        assert actual["runtime"]["connected"]  # Rejection did not execute disconnect.
        assert actual["input_snapshot"]["runtime"]["input_scale_rejected_count"] == count
    time.sleep(.12)
    later = value.latest_snapshot()
    assert later["runtime"]["input_scale_rejected_count"] == 2
    assert "未自动重试" in later["runtime"]["error"]


def test_executed_and_rejected_requests_share_completion_count_not_execution_generation(controller):
    value = controller()
    publish(value, snapshot(4))
    value.latest_snapshot()
    assert value.submit("use_input_scale_candidate")
    assert value.submit("use_input_scale_candidate")
    assert value._pending_input_scale_generation == 6
    rejected = snapshot(4)
    rejected["runtime"].update(input_scale_rejected_count=1, error="runtime queue full")
    publish(value, rejected, value._command_id)
    assert_hidden(value)  # One rejected plus one still waiting for execution.
    finished = snapshot(5, connected=False)
    finished["runtime"]["input_scale_rejected_count"] = 1
    publish(value, finished, value._command_id)
    actual = value.latest_snapshot()
    assert value._pending_input_scale_generation is None
    assert not actual["runtime"]["input_scale_pending"]
    assert actual["runtime"]["input_scale_generation"] == 5
