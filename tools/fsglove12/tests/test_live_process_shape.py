"""Small official-only shape IPC commands and safe preview publication."""
from copy import deepcopy
import pickle
import queue
import time

import pytest

from test_live_process import Runtime, controller, wait_for
from vt_fsglove12.live_process import SHAPE_COMMAND_ARGS, _hide, _idle, _publish


COMMANDS = [
    ("capture_shape_contact", dict(category=2, contact_confirmed=True)),
    ("optimize_shape", {}), ("cancel_shape", {}),
    ("preview_shape", dict(mode="candidate")),
    ("apply_shape_candidate", {}),
    ("load_shape_profile", dict(path="shape.json")),
    ("save_shape_profile", dict(path="shape.json")),
    ("revert_shape", {}),
    ("retry_calibration_save", {}),
    ("retry_calibration_save_to", dict(directory="another-directory")),
    ("save_shape_task", dict(path="touch_task.json")),
    ("load_shape_task", dict(path="touch_task.json")),
]


def shape_state(**changes):
    return dict(dict(status="candidate_ready", candidate_available=True,
        contacts={"2": {"accepted_frames": 30}}, active_category=3,
        busy=True, phase="optimize", preview_mode="candidate", preview_revision=7,
        shape_calibrated=True, can_revert=True, applied_path="approved_shape.json"), **changes)


def snapshot(*, connected=True, shape=None):
    value = _idle()
    now = time.monotonic()
    value["runtime"].update(connected=connected, session_id="shape-session")
    value["orientation"] = dict(timestamp_s=now, nodes={"12": dict(age_s=0.)}) if connected else None
    value["pose"].update(timestamp_s=now, pose_available=connected, solver_success=connected)
    value["geometry"] = {"vertices_m": [[1., 2., 3.]]}
    value["display_pose"] = {"display_only": True}
    value["display_geometry"] = {"vertices_m": [[4., 5., 6.]]}
    value["calibration"]["shape_calibration"] = shape if shape is not None else shape_state()
    return value


@pytest.mark.parametrize("name,args", COMMANDS)
def test_shape_commands_are_official_only_small_and_do_not_start_or_connect(controller, name, args):
    local = controller()
    assert not local.submit(name, **args)
    official = controller(upstream_checkout="UNUSED_OFFICIAL_CHECKOUT")
    assert official.submit(name, **args)
    command = official._commands.get(timeout=1.)
    assert command == (1, name, args)
    assert len(pickle.dumps(command)) < 512
    assert official.pid is None and not official.latest_snapshot()["runtime"]["connected"]
    with pytest.raises(queue.Empty):
        official._commands.get(timeout=.02)


@pytest.mark.parametrize("category", [2, 3, 4, 5])
def test_contact_categories_use_exact_supported_integer_ids(controller, category):
    official = controller(upstream_checkout="UNUSED")
    assert official.submit("capture_shape_contact", category=category, contact_confirmed=True)


@pytest.mark.parametrize("name,args", [
    ("capture_shape_contact", {}),
    ("capture_shape_contact", dict(category=2)),
    ("capture_shape_contact", dict(category=True, contact_confirmed=True)),
    ("capture_shape_contact", dict(category=2., contact_confirmed=True)),
    ("capture_shape_contact", dict(category="2", contact_confirmed=True)),
    ("capture_shape_contact", dict(category=[2], contact_confirmed=True)),
    ("capture_shape_contact", dict(category=1, contact_confirmed=True)),
    ("capture_shape_contact", dict(category=6, contact_confirmed=True)),
    ("capture_shape_contact", dict(category=2, contact_confirmed=False)),
    ("capture_shape_contact", dict(category=2, contact_confirmed=1)),
    ("capture_shape_contact", dict(category=2, contact_confirmed="yes")),
    ("preview_shape", dict(mode="unknown")), ("preview_shape", dict(mode=[])),
    ("preview_shape", dict(mode=True)),
    ("load_shape_profile", dict(path="")), ("load_shape_profile", dict(path="  ")),
    ("save_shape_profile", dict(path=None)), ("save_shape_profile", dict(path="x" * 4097)),
    ("load_shape_profile", dict(path="shape.json", shape={"blob": "must_not_cross_IPC"})),
    ("load_shape_task", dict(path="")), ("load_shape_task", dict(path=" " * 3)),
    ("save_shape_task", dict(path=None)), ("save_shape_task", dict(path="x" * 4097)),
    ("load_shape_task", dict(path="task.json", contacts={"blob": "must_not_cross_IPC"})),
    ("retry_calibration_save_to", dict(directory="")),
    ("retry_calibration_save_to", dict(directory="   ")),
    ("retry_calibration_save_to", dict(directory=None)),
    ("retry_calibration_save_to", dict(directory="x" * 4097)),
    ("retry_calibration_save_to", dict(path="wrong-key")),
    ("retry_calibration_save_to", dict(directory="target", profile={"blob": "must_not_cross_IPC"})),
] + [(name, dict(untrusted_blob="x")) for name, args in COMMANDS if not args])
def test_invalid_shape_arguments_never_enter_the_command_queue(controller, name, args):
    official = controller(upstream_checkout="UNUSED")
    assert not official.submit(name, **args)
    assert official._command_id == 0
    with pytest.raises(queue.Empty):
        official._commands.get(timeout=.01)


@pytest.mark.parametrize("mode", ["current", "candidate", "baseline"])
def test_supported_preview_modes(controller, mode):
    assert controller(upstream_checkout="UNUSED").submit("preview_shape", mode=mode)


def test_full_queue_cannot_acknowledge_a_shape_operation(controller):
    official = controller(upstream_checkout="UNUSED")
    real_queue = official._commands
    class FullQueue:
        def put_nowait(self, command):
            raise queue.Full
    try:
        official._commands = FullQueue()
        assert not official.submit("apply_shape_candidate")
        assert official._command_id == 0
    finally:
        official._commands = real_queue


@pytest.mark.parametrize("command", ["disconnect", "clear_calibration"])
def test_pending_clear_or_disconnect_hides_shape_preview_without_completed_ack(controller, command):
    official = controller(upstream_checkout="UNUSED")
    old = snapshot()
    assert _publish(official._mailbox, old, 0)
    assert official.latest_snapshot()["display_geometry"]
    assert official.submit(command)
    # Queue receipt alone cannot prove the worker cleared the old shape state.
    assert _publish(official._mailbox, old, official._command_id)
    value = official.latest_snapshot()
    shape = value["calibration"]["shape_calibration"]
    assert value["geometry"] is None and value["display_geometry"] is None
    assert not shape["candidate_available"] and not shape["busy"]
    assert shape["contacts"] == {} and shape["active_category"] is None
    assert shape["preview_mode"] == "current" and shape["preview_revision"] == 7
    assert shape["applied_path"] == "approved_shape.json" and shape["shape_calibrated"]
    assert value["calibration"]["clear_generation"] == 0
    assert old["calibration"]["shape_calibration"]["candidate_available"]
    assert official._cached["calibration"]["shape_calibration"]["candidate_available"]


def test_ordinary_disconnected_read_preserves_explicit_loaded_profile(controller):
    official = controller(upstream_checkout="UNUSED")
    loaded = shape_state(status="loaded", busy=False, active_category=None,
                         shape_calibrated=True, candidate_available=False)
    assert _publish(official._mailbox, snapshot(connected=False, shape=loaded), 0)
    value = official.latest_snapshot()
    assert value["calibration"]["shape_calibration"] == loaded
    assert value["geometry"] is None and value["display_geometry"] is None
    assert value["display_pose"] is None


def test_disconnected_polling_preserves_frozen_task_and_offline_optimization_candidate(controller):
    official = controller(upstream_checkout="UNUSED")
    loaded = shape_state(status="candidate", busy=False, active_category=None,
        task_loaded=True, task_path="touch_task.json", can_optimize=True, can_capture=False,
        can_save_task=True, can_load_task=True, can_apply=False,
        contacts={str(category): {"frames": 20, "confirmed": True, "saved": True}
                  for category in range(2, 6)}, saved_groups=["2", "3", "4", "5"],
        unsaved_groups=[], storage_error=None, storage_message="已保存")
    assert _publish(official._mailbox, snapshot(connected=False, shape=loaded), 0)
    for _ in range(3):
        value = official.latest_snapshot()
        assert value["calibration"]["shape_calibration"] == loaded
        assert value["geometry"] is None and value["display_geometry"] is None
        assert value["display_pose"] is None and not value["runtime"]["connected"]


def test_transient_stale_pose_does_not_erase_shape_candidate_or_claim_completion():
    value = snapshot()
    shape = deepcopy(value["calibration"]["shape_calibration"])
    hidden = _hide(value, "missing_or_stale_root")
    assert hidden["geometry"] is None
    assert hidden["calibration"]["shape_calibration"] == shape


class ShapeRuntime(Runtime):
    """Command updates may have no new IMU sample, but must still publish."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.shape = shape_state(busy=False)
        self.revision = 0

    def snapshot_revision(self):
        return self.revision

    def submit(self, name, **args):
        if name in SHAPE_COMMAND_ARGS:
            self.shape["last_command"] = name
            if name == "preview_shape":
                self.shape["preview_mode"] = args["mode"]
            self.shape["preview_revision"] += 1
        self.revision += 1
        return super().submit(name, **args)

    def latest_snapshot(self):
        value = super().latest_snapshot()
        value["calibration"]["shape_calibration"] = deepcopy(self.shape)
        return value


def test_spawn_publishes_shape_only_changes_and_parent_receives_independent_copy(controller):
    value = controller(runtime_factory=ShapeRuntime)
    # Exercise official proxy routing with fake child resources only; the fake
    # runtime does not construct an actual official model or access hardware.
    value._official = True
    value.start()
    initial = wait_for(lambda: value.latest_snapshot()["calibration"].get("shape_calibration"))
    assert initial["preview_revision"] == 7
    assert value.submit("preview_shape", mode="baseline")
    changed = wait_for(lambda: (s if (s := value.latest_snapshot())["calibration"].get(
        "shape_calibration", {}).get("preview_mode") == "baseline" else None))
    assert changed["calibration"]["shape_calibration"]["preview_revision"] == 8
    assert not changed["runtime"]["connected"]
    changed["calibration"]["shape_calibration"]["contacts"].clear()
    assert value.latest_snapshot()["calibration"]["shape_calibration"]["contacts"]


class RejectShapeRuntime(ShapeRuntime):
    def submit(self, name, **args):
        if name in {"optimize_shape", "retry_calibration_save", "retry_calibration_save_to",
                    "load_shape_task", "save_shape_task"}:
            return False
        return super().submit(name, **args)


@pytest.mark.parametrize("command,args", [("optimize_shape", {}), ("retry_calibration_save", {}),
    ("retry_calibration_save_to", {"directory": "another-directory"}),
    ("load_shape_task", {"path": "task.json"}), ("save_shape_task", {"path": "task.json"})])
def test_child_shape_queue_rejection_is_not_a_completed_execution_or_auto_retry(controller, command, args):
    value = controller(runtime_factory=RejectShapeRuntime)
    value._official = True
    value.start()
    wait_for(lambda: value.latest_snapshot()["calibration"].get("shape_calibration"))
    assert value.submit(command, **args)
    rejected = wait_for(lambda: (s if (s := value.latest_snapshot())["runtime"].get(
        "shape_rejected_count") == 1 else None))
    runtime = rejected["runtime"]
    assert runtime.get("shape_command_generation", 0) == 0
    assert "未自动重试" in runtime["shape_command_error"]
    assert runtime["error"] == runtime["shape_command_error"]
    assert rejected["calibration"]["shape_calibration"]["preview_revision"] == 7
    time.sleep(.08)
    assert value.latest_snapshot()["runtime"]["shape_rejected_count"] == 1
    assert value.submit("preview_shape", mode="baseline")
    accepted = wait_for(lambda: (s if (s := value.latest_snapshot())["calibration"].get(
        "shape_calibration", {}).get("preview_mode") == "baseline" else None))
    assert accepted["runtime"]["shape_command_error"] is None


class ExecutionErrorShapeRuntime(ShapeRuntime):
    """An accepted command executes and fails; both channels own the error."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.execution_generation, self.execution_error = 0, None

    def submit(self, name, **args):
        if name in SHAPE_COMMAND_ARGS:
            self.execution_generation += 1
            self.execution_error = "fixture executed shape profile validation failed" if name == "apply_shape_candidate" else None
            self.shape["error"] = self.execution_error
        return super().submit(name, **args)

    def latest_snapshot(self):
        value = super().latest_snapshot()
        value["runtime"].update(shape_command_generation=self.execution_generation,
                                shape_command_error=self.execution_error)
        return value

    def latest_input_snapshot(self):
        return dict(runtime=dict(connected=False, session_id=None,
            published_monotonic_s=time.monotonic(), shape_command_generation=self.execution_generation,
            shape_command_error=self.execution_error), orientation=None)


def test_spawn_preserves_executed_shape_error_in_both_independent_mailboxes(controller):
    value = controller(runtime_factory=ExecutionErrorShapeRuntime)
    value._official = True
    value.start()
    wait_for(lambda: value.latest_snapshot()["calibration"].get("shape_calibration"))
    assert value.submit("apply_shape_candidate")
    expected = "fixture executed shape profile validation failed"

    def both(generation, error):
        data = value.latest_snapshot()
        runtimes = (data["runtime"], data["input_snapshot"]["runtime"])
        return data if all(runtime.get("shape_command_generation") == generation
            and runtime.get("shape_command_error") == error for runtime in runtimes) else None

    failed = wait_for(lambda: both(1, expected))
    assert failed["calibration"]["shape_calibration"]["error"] == expected
    assert failed["runtime"]["shape_rejected_count"] == 0
    assert failed["input_snapshot"]["runtime"]["shape_rejected_count"] == 0
    assert value._input_sequence > 0, "verify the independent input mailbox, not pose fallback"
    # A subsequent accepted execution clears its own error on both paths.
    assert value.submit("preview_shape", mode="current")
    succeeded = wait_for(lambda: both(2, None))
    assert succeeded["calibration"]["shape_calibration"]["error"] is None
    assert not succeeded["runtime"]["connected"]
