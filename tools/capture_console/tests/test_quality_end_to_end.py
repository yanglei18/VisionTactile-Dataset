"""Real IPC/quality subprocess integration; no ROS, Recorder or hardware.

The bag fixture deliberately contains metadata.yaml only, NOT a recorded MCAP.
Image entries exercise metadata counts only. No image payload, per-frame
validity, hardware rate or successful physical capture is claimed by this test.
"""

from concurrent.futures import Future
from copy import deepcopy
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

from vt_capture_console.backend.ipc_client import IpcBackend, fetch_snapshot
from vt_capture_manager.config import ManagerConfig
from vt_capture_manager.management import ManagementService
from vt_capture_manager.model import SnapshotModel
from vt_capture_manager.quality_runner import QualityRunner
from vt_capture_manager.server import SnapshotServer


IMAGE_TOPIC = "/fixture/camera/image"
MISSING_TOPIC = "/fixture/missing_camera/image"


def _finished(value):
    future = Future()
    future.set_result(value)
    return future


class FakeAdapter:
    """In-memory controller contexts only; cannot launch a recording process."""

    def __init__(self, bag):
        self.bag = str(bag)
        self.commands = []
        self.store = None
        self._lock = threading.Lock()
        self.context = dict(
            controller_instance_id="metadata-fixture-controller", state_revision=1,
            state=0, session_id="", start_request_id="", session_label="",
            output_path="", recorder_alive=False, elapsed_sec=0.,
            planned_duration_sec=0, topics=[IMAGE_TOPIC, MISSING_TOPIC],
            max_duration_sec=300, detail="Test fixture only; not a real Recorder",
        )

    def get_snapshot(self):
        with self._lock:
            return _finished(deepcopy(self.context))

    def command(self, wire):
        # The manager must durably dispatch before any adapter action.
        assert self.store.operation(wire["request_id"])["phase"] == "DISPATCHED"
        with self._lock:
            self.commands.append(deepcopy(wire))
            assert wire["expected_controller_instance_id"] == self.context["controller_instance_id"]
            self.context["state_revision"] += 1
            if wire["command"] == 1:
                self.context.update(
                    state=3, session_id="fixture-session-" + wire["request_id"],
                    start_request_id=wire["request_id"], session_label=wire["session_label"],
                    output_path=self.bag, recorder_alive=True,
                    planned_duration_sec=wire["planned_duration_sec"],
                    detail="Simulated recording context; no Recorder was launched",
                )
            else:
                assert wire["command"] == 2
                assert wire["expected_session_id"] == self.context["session_id"]
                self.context.update(state=5, recorder_alive=False, elapsed_sec=1.,
                                    detail="Simulated COMPLETE; bag quality is separate")
            return _finished(dict(accepted=True, code="accepted", detail="Fake adapter only",
                                  context=deepcopy(self.context)))


class ObservedQualityRunner(QualityRunner):
    """Hold one real child launch so COMPLETE != quality success is observable."""

    def __init__(self):
        super().__init__(python_executable=sys.executable, timeout_s=15.)
        self.entered = threading.Event()
        self.release = threading.Event()
        self.invocations = []

    def _execute(self, argv, cancel):
        self.invocations.append(list(argv))
        self.entered.set()
        if not self.release.wait(10.):
            raise TimeoutError("Test did not release the quality child")
        # No subprocess mock: execute the actual isolated quality CLI.
        return super()._execute(argv, cancel)


def _wait(check, description, *, timeout=15.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = check()
        if value:
            return value
        time.sleep(.01)
    raise AssertionError("Timed out waiting for " + description)


def _trial(backend, trial_id, **expected):
    for trial in backend.snapshot()["management"]["trials"]:
        if trial["id"] == trial_id and all(trial[key] == value for key, value in expected.items()):
            return trial
    return None


def _receipt(backend, request_id):
    operation = backend.latest_operation
    if operation and operation["request_id"] == request_id and operation["code"] != "QUEUED":
        assert operation["state"] == "accepted", operation
        return operation
    return None


def test_real_v3_ipc_metadata_freeze_quality_child_and_v2_compatibility(tmp_path):
    bag = tmp_path / "metadata-only-fixture-NOT-recorded-MCAP"
    bag.mkdir()
    # JSON is a YAML subset. Deliberately no bag data files or rosbag writer.
    metadata = dict(rosbag2_bagfile_information=dict(
        storage_identifier="mcap", relative_file_paths=[],
        duration=dict(nanoseconds=1_000_000_000),
        starting_time=dict(nanoseconds_since_epoch=1_000_000_000), message_count=30,
        topics_with_message_count=[dict(topic_metadata=dict(
            name=IMAGE_TOPIC, type="sensor_msgs/msg/Image"), message_count=30)],
    ))
    metadata_path = bag / "metadata.yaml"
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
    bag_before = metadata_path.read_bytes()
    model = SnapshotModel(
        ManagerConfig((), quality_enabled=True, quality_timeout_s=15.),
        disk_usage=lambda _: SimpleNamespace(free=123456789),
    )
    adapter, runner = FakeAdapter(bag), ObservedQualityRunner()
    service = ManagementService(model, adapter, enabled=True, state_dir=tmp_path / "state",
                                poll_interval_s=.02, timeout_s=.1, quality_runner=runner)
    adapter.store = service.store
    server = SnapshotServer(model, tmp_path / "private" / "manager.sock", management=service)
    backend = None
    try:
        service.start()
        server.start()
        backend = IpcBackend(server.path)
        _wait(lambda: backend.supports_details and backend.snapshot()["capabilities"]["record"],
              "healthy v3 management")

        batch_request = backend.submit("batch.create", {"label": "Metadata-only integration"})
        _wait(lambda: _receipt(backend, batch_request), "batch receipt")
        batch_id = backend.snapshot()["management"]["batches"][0]["id"]
        context = backend.snapshot()["management"]["controller"]
        start_request = backend.submit("capture.start", dict(
            batch_id=batch_id, label="No hardware or real MCAP", duration_s=30,
            expected_controller_instance_id=context["controller_instance_id"],
            expected_state_revision=context["state_revision"],
            task_name="metadata-only 流程验收", operator_name="test-operator",
        ))
        start_receipt = _wait(lambda: _receipt(backend, start_request), "START receipt")
        trial_id = start_receipt["trial_id"]
        recording = _wait(lambda: _trial(backend, trial_id, state="RECORDING"), "owned recording context")
        assert recording["quality"] == "NOT_ASSESSED"
        assert not runner.invocations
        assert recording["output_path"] == str(bag)
        assert recording["session_id"] == "fixture-session-" + start_request
        assert "task_name" not in adapter.commands[0] and "operator_name" not in adapter.commands[0]

        context = backend.snapshot()["management"]["controller"]
        stop_request = backend.submit("capture.stop", dict(
            trial_id=trial_id, expected_controller_instance_id=context["controller_instance_id"],
            expected_session_id=recording["session_id"],
        ))
        _wait(lambda: _receipt(backend, stop_request), "STOP receipt")
        _wait(lambda: runner.entered.is_set(), "queued real quality work")
        _wait(lambda: _trial(backend, trial_id, state="COMPLETE", quality="RUNNING"),
              "Recorder COMPLETE independently from unfinished quality")
        backend.request_details(trial_id)
        _wait(lambda: backend.details_status["state"] == "ready", "frozen metadata details")
        early = backend.latest_details
        assert early["quality"]["state"] == "RUNNING" and early["quality"]["verdict"] == ""
        start_metadata = early["metadata"]["start"]
        assert start_metadata["task_name"] == "metadata-only 流程验收"
        assert start_metadata["operator_name"] == "test-operator"
        assert start_metadata["controller_before_start"]["session_id"] == ""
        finish = early["metadata"]["finish"]
        assert finish["controller"]["session_id"] == recording["session_id"]
        assert finish["controller"]["recorder_alive"] is False
        assert finish["recorder_stopped_confirmed"] is True
        frozen_files = {key: Path(value).read_bytes() for key, value in early["artifacts"].items()}
        assert all(frozen_files.values())

        runner.release.set()
        _wait(lambda: _trial(backend, trial_id, state="COMPLETE", quality="WARNINGS"),
              "real quality subprocess warning report")
        # Fast snapshots update summaries, not the explicitly fetched cache.
        assert backend.latest_details["quality"]["state"] == "RUNNING"
        previous_receipt = backend.latest_operation
        backend.request_details(trial_id)
        _wait(lambda: backend.details_status["state"] == "ready", "completed quality details")
        details = backend.latest_details
        assert backend.latest_operation == previous_receipt
        quality = details["quality"]
        assert quality["state"] == "COMPLETED" and quality["verdict"] == "warnings"
        assert quality["attempt"] == 1
        report_path = Path(quality["report_path"])
        assert not report_path.is_relative_to(bag)
        report = json.loads(report_path.read_text(encoding="utf-8"))
        assert report["status"] == "warnings"
        assert report["topics"][IMAGE_TOPIC]["inspection"] == "metadata_only"
        assert report["topics"][IMAGE_TOPIC]["recorded_message_count"] == 30
        assert report["summary"]["inspected_topic_count"] == 0
        assert report["summary"]["metadata_only_topic_count"] == 1
        assert any(w["code"] == "expected_topic_missing" and w.get("topic") == MISSING_TOPIC
                   for w in report["warnings"])
        assert report["runner_provenance"]["expected_topics"] == sorted([IMAGE_TOPIC, MISSING_TOPIC])
        for key in ("summary", "warnings", "topics"):
            assert quality["report"][key] == report[key]
        assert details["metadata"] == early["metadata"]
        assert {key: Path(value).read_bytes() for key, value in details["artifacts"].items()} == frozen_files
        assert len(runner.invocations) == 1
        assert runner.invocations[0][1:3] == ["-m", "vt_capture_manager.quality_runner"]
        assert [wire["command"] for wire in adapter.commands] == [1, 2]
        assert metadata_path.read_bytes() == bag_before
        assert sorted(path.name for path in bag.iterdir()) == ["metadata.yaml"]

        # Old clients remain strict and readable: no detail payload expansion.
        old = fetch_snapshot(server.path, version=2)
        assert old["management"]["trials"][0]["quality"] == "WARNINGS"
        assert "trial_details" not in old and "metadata" not in old["management"]["trials"][0]
        assert set(old) == set(fetch_snapshot(server.path, version=3))
        legacy = fetch_snapshot(server.path)
        assert "management" not in legacy and legacy["capabilities"]["record"] is False
    finally:
        runner.release.set()
        if backend is not None:
            backend.close()
        server.close()
        service.close()
