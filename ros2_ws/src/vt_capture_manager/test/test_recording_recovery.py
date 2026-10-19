"""Exercise real manager/controller/recorder boundaries without ROS or processes.

Only OS process creation, disk availability and selected failure points are
injected. Contexts and receipts are produced by the real serialized controller.
"""
from concurrent.futures import Future
from dataclasses import asdict
from itertools import count
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from vt_capture_manager.config import ManagerConfig
from vt_capture_manager.management import ManagementService
from vt_capture_manager.model import SnapshotModel
from vt_realsense_capture.config import load_config
from vt_realsense_capture.controller import CaptureController
from vt_realsense_capture.controller_node import ManagedRecorder
from vt_realsense_capture.managed import ManagedCommandFact
from vt_realsense_capture.recorder import RecorderProcess
from vt_realsense_capture.storage import FileStorage


CAPTURE = Path(__file__).parents[2] / "vt_realsense_capture"


def wire_context(context):
    value = asdict(context)
    value["topics"] = list(value["topics"])
    return value


def resolved(value):
    future = Future()
    future.set_result(value)
    return future


class Faults:
    boundary = ""
    stop = ""
    health_unknown = False
    lost_receipt = False


class Child:
    """A process handle; no Popen, signal, or real PID is used."""
    pid = 4242
    returncode = None

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        self.returncode = 0
        return self.returncode


class FaultStorage(FileStorage):
    def __init__(self, root, faults):
        self.root, self.faults = root, faults

    def disk_free_bytes(self, path):
        return 0 if self.faults.boundary == "session_disk" and path != self.root else 1024**4

    def write_qos(self, *args, **kwargs):
        if self.faults.boundary == "qos":
            raise OSError("injected QoS write failure")
        return super().write_qos(*args, **kwargs)


class InjectedProcess(RecorderProcess):
    def __init__(self, *args, faults, **kwargs):
        self.faults = faults
        super().__init__(*args, **kwargs)

    def start(self):
        result = super().start()
        if self.faults.boundary == "after_spawn":
            raise OSError("injected failure after process creation")
        return result

    def poll(self):
        if self.faults.health_unknown:
            raise OSError("injected health failure")
        return super().poll()

    def stop(self):
        if self.faults.stop == "raise":
            raise OSError("injected stop adapter failure")
        return super().stop()


class ControllerAdapter:
    def __init__(self, controller, faults):
        self.controller, self.faults = controller, faults
        self.commands = []

    def get_snapshot(self):
        return resolved(wire_context(self.controller.capture_context()))

    def command(self, wire):
        self.commands.append(wire)
        receipt = self.controller.accept_managed(ManagedCommandFact(**wire))
        if self.faults.lost_receipt:
            return Future()
        value = asdict(receipt)
        value["context"] = wire_context(receipt.context)
        return resolved(value)


@pytest.fixture
def rig(tmp_path):
    faults = Faults()
    root = tmp_path / "capture"
    root.mkdir()
    constructed = []
    children = []

    def popen(*_args, **_kwargs):
        if faults.boundary == "popen":
            raise FileNotFoundError("injected missing ros2 executable")
        child = Child()
        children.append(child)
        return child

    def signal(*_args):
        if faults.stop == "uncertain":
            raise OSError("injected signal failure")

    def process_factory(*args, **kwargs):
        if faults.boundary == "construct":
            raise OSError("injected recorder construction failure")
        process = InjectedProcess(*args, **kwargs, faults=faults,
                                  popen_factory=popen, killpg=signal)
        constructed.append(process)
        return process

    sessions = count()
    controller = CaptureController(
        config=load_config(CAPTURE / "config/cameras.yaml"), output_root=root,
        clock=SimpleNamespace(monotonic_ns=lambda: 10_000_000_000),
        storage=FaultStorage(root, faults),
        recorder_factory=lambda: ManagedRecorder(CAPTURE / "config/mcap_writer_options.yaml",
                                                  process_factory=process_factory),
        event_sink=lambda _: None, status_sink=lambda _: None, session_sink=lambda _: None,
        session_id_factory=lambda: f"session-{next(sessions)}",
    )
    adapter = ControllerAdapter(controller, faults)
    model = SnapshotModel(ManagerConfig((), quality_enabled=False),
                          disk_usage=lambda _: SimpleNamespace(free=1024**4))
    service = ManagementService(model, adapter, enabled=True, state_dir=tmp_path / "manager", timeout_s=.01)
    yield SimpleNamespace(controller=controller, service=service, faults=faults, root=root,
                          constructed=constructed, children=children, adapter=adapter)
    service.close()


def controller_command(controller, request_id, command=1):
    context = controller.capture_context()
    return ManagedCommandFact(request_id, command, context.controller_instance_id,
        context.session_id, context.state_revision, "trial" if command == 1 else "", 30 if command == 1 else 0)


def create_batch(service):
    response = service.dispatch(dict(version=2, request_id="batch", command="batch.create", params={"label": "test"}))
    return response["snapshot"]["management"]["batches"][0]["id"]


def start(rig, batch, request_id="start"):
    context = rig.controller.capture_context()
    return rig.service.dispatch(dict(version=2, request_id=request_id, command="capture.start", params=dict(
        batch_id=batch, label="trial", duration_s=30, expected_controller_instance_id=context.controller_instance_id,
        expected_state_revision=context.state_revision)))


def execute(service):
    item = service._commands.get_nowait()
    try:
        service._execute(*item)
    finally:
        service._commands.task_done()


def test_historical_controller_start_id_is_rejected_without_adopting_old_session(rig):
    assert rig.controller.accept_managed(controller_command(rig.controller, "historical-start")).accepted
    assert rig.controller.accept_managed(controller_command(rig.controller, "historical-stop", 2)).accepted
    assert rig.service.poll_once()
    response = start(rig, create_batch(rig.service), "historical-start")
    assert response["operation"]["state"] == "rejected"
    assert response["operation"]["code"] == "request_id_conflict"
    assert response["operation"]["trial_id"] == ""
    assert rig.service.store.view()["trials"] == []
    assert rig.service._commands.empty() and rig.adapter.commands == []
    assert len(rig.children) == 1


@pytest.mark.parametrize("previous_state", [0, 5, 6], ids=["idle", "complete", "invalid"])
@pytest.mark.parametrize("boundary", ["qos", "session_disk", "construct", "popen"])
def test_definite_start_failure_preserves_artifacts_and_allows_new_intent(rig, previous_state, boundary):
    if previous_state != 0:
        rig.faults.boundary = "after_spawn" if previous_state == 6 else ""
        rig.controller.accept_managed(controller_command(rig.controller, "previous"))
        rig.faults.boundary = ""
        if previous_state == 6:
            rig.controller.tick()
        else:
            rig.controller.accept_managed(controller_command(rig.controller, "previous-stop", 2))
        assert rig.controller.capture_context().state == previous_state
    rig.faults.boundary = boundary
    assert rig.service.poll_once()
    batch = create_batch(rig.service)
    trial_id = start(rig, batch)["operation"]["trial_id"]
    execute(rig.service)
    trial = rig.service.store.trial(trial_id)
    assert trial["state"] == "FAILED"
    assert trial["session_id"] and trial["output_path"]
    assert rig.service.store.operation("start")["state"] == "rejected"
    assert rig.service.store.quality_job(trial_id) is None
    assert rig.service.store.details(trial_id)["metadata"]["finish"] is None
    record_path = Path(trial["output_path"]).parent / "controller_session.json"
    failed_bytes = record_path.read_bytes()
    assert json.loads(failed_bytes)["lifecycle"] == "start_failed"
    for _ in range(3):
        assert rig.service.poll_once()
    assert rig.service.snapshot(2)["capabilities"]["record"]
    assert len(rig.adapter.commands) == 1
    assert rig.service.store.trial(trial_id)["state"] == "FAILED"
    rig.faults.boundary = ""
    next_trial = start(rig, batch, "next-start")["operation"]["trial_id"]
    execute(rig.service)
    assert rig.service.store.trial(next_trial)["state"] == "RECORDING"
    assert record_path.read_bytes() == failed_bytes
    assert rig.controller.shutdown()


def test_lost_failed_start_receipt_is_reconciled_without_resending(rig):
    rig.faults.boundary = "popen"
    rig.faults.lost_receipt = True
    rig.service.poll_once()
    trial_id = start(rig, create_batch(rig.service))["operation"]["trial_id"]
    execute(rig.service)
    assert rig.service.store.trial(trial_id)["state"] == "UNKNOWN"
    assert rig.service.poll_once()
    assert rig.service.store.trial(trial_id)["state"] == "FAILED"
    assert rig.service.store.operation("start")["state"] == "rejected"
    assert rig.service.snapshot(2)["capabilities"]["record"]
    assert len(rig.adapter.commands) == 1


@pytest.mark.parametrize("health_unknown", [False, True], ids=["alive", "uncertain"])
@pytest.mark.parametrize("stop_fault", ["uncertain", "raise"])
def test_partial_spawn_keeps_process_ownership_until_termination(rig, health_unknown, stop_fault):
    rig.faults.boundary = "after_spawn"
    rig.faults.health_unknown = health_unknown
    rig.faults.stop = stop_fault
    rig.service.poll_once()
    batch = create_batch(rig.service)
    trial_id = start(rig, batch)["operation"]["trial_id"]
    execute(rig.service)
    context = rig.controller.capture_context()
    assert context.state == 4 and context.recorder_alive
    trial = rig.service.store.trial(trial_id)
    assert trial["state"] == "STOPPING" and trial["session_id"] == context.session_id
    assert len(rig.children) == len(rig.constructed) == 1
    assert rig.constructed[0].pid == rig.children[0].pid
    journal = Path(context.output_path).parent / "controller_session.json"
    assert json.loads(journal.read_text())["lifecycle"] == "finalizing"
    assert not rig.service.snapshot(2)["capabilities"]["record"]
    assert start(rig, batch, "blocked")["operation"]["code"] == "active_session"
    rig.controller.tick()
    assert not rig.controller.shutdown()
    record = json.loads(journal.read_text())
    assert record["lifecycle"] == "finalizing"
    assert not record["finalization"]["termination_confirmed"]
    assert record["finalization"]["possibly_alive"]
    assert rig.service.poll_once()
    assert not rig.service.snapshot(2)["capabilities"]["record"]
    rig.faults.stop = ""
    rig.faults.health_unknown = False
    rig.controller.tick()
    assert rig.service.poll_once()
    assert rig.service.store.trial(trial_id)["state"] == "FAILED"
    assert rig.service.snapshot(2)["capabilities"]["record"]
    record = json.loads(journal.read_text())
    assert record["lifecycle"] == "start_failed"
    assert record["startup_error"]["code"] == "recorder_spawn_failed"
    assert record["finalization"]["termination_confirmed"]
    assert record["finalization"]["reason"] == "Recorder startup failed"
    assert len(rig.adapter.commands) == 1


def test_rejected_receipt_for_live_recording_cannot_be_resolved_by_another_start(rig):
    def fail_started_publication(event):
        if event.code == "recorder_started":
            raise OSError("injected start event publisher failure")

    rig.controller._event_sink = fail_started_publication
    rig.service.poll_once()
    trial_id = start(rig, create_batch(rig.service))["operation"]["trial_id"]
    execute(rig.service)
    owned = rig.service.store.trial(trial_id)
    assert owned["state"] == "RECORDING"
    assert rig.service.store.operation("start")["state"] == "rejected"
    # The local manager misses the terminal observation of its owned session.
    rig.controller.accept_managed(controller_command(rig.controller, "external-stop", 2))
    rig.faults.boundary = "popen"
    rejected = rig.controller.accept_managed(controller_command(rig.controller, "external-start"))
    assert not rejected.accepted and rejected.context.state == 0
    assert rig.service.poll_once()
    unresolved = rig.service.store.trial(trial_id)
    assert unresolved["state"] == "UNKNOWN"
    assert unresolved["session_id"] == owned["session_id"]
    assert unresolved["output_path"] == owned["output_path"]
    assert not rig.service.snapshot(2)["capabilities"]["record"]
    assert rig.service.store.details(trial_id)["metadata"]["finish"] is None
