"""Real local IPC and owned child processes; no ROS, recorder, or hardware.

Topic counts/freshness are synthetic observations supplied by this test. The
Python child only prints its PID and sleeps; RUNNING is not hardware evidence.
"""

from copy import deepcopy
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from vt_capture_console.backend.ipc_client import IpcBackend, fetch_snapshot
from vt_capture_core.protocol import CALIBRATION_PROTOCOL_VERSION
from vt_capture_manager.config import ManagerConfig
from vt_capture_manager.device_processes import ProcessSpec, ProcessSupervisor
from vt_capture_manager.management import ManagementService
from vt_capture_manager.model import SnapshotModel
from vt_capture_manager.server import SnapshotServer


TOPICS = ("/fixture/process/first", "/fixture/process/second")
GROUP_ID = "harmless_fixture"


class FakeObservations:
    """Atomic test-owned graph state; never import or contact a ROS graph."""

    def __init__(self):
        self._lock = threading.Lock()
        self._topics = {topic: {"publishers": 0, "fresh": False} for topic in TOPICS}

    def __call__(self):
        with self._lock:
            return deepcopy(self._topics)

    def set(self, *, publishers, fresh, topics=TOPICS):
        with self._lock:
            for topic in topics:
                self._topics[topic] = {"publishers": publishers, "fresh": fresh}


def _wait(check, description, *, timeout=8.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = check()
        if value:
            return value
        time.sleep(.01)
    raise AssertionError("Timed out waiting for " + description)


def _group(backend, state, **expected):
    for group in backend.snapshot()["process_control"]["groups"]:
        if (group["id"] == GROUP_ID and group["state"] == state
                and all(group[key] == value for key, value in expected.items())):
            return group
    return None


def _receipt(backend, request_id, *, state, code):
    operation = backend.latest_operation
    if (operation and operation["request_id"] == request_id
            and operation["state"] == state and operation["code"] == code):
        return operation
    return None


@pytest.mark.skipif(sys.platform != "linux", reason="Linux process-group identity uses /proc")
def test_current_ipc_device_lifecycle_dedup_external_guard_and_close_detaches(tmp_path):
    model = SnapshotModel(
        ManagerConfig((), quality_enabled=False), instance_id="fixture-device-manager",
        disk_usage=lambda _: SimpleNamespace(free=123456789),
    )
    service = ManagementService(model, enabled=False, state_dir=tmp_path / "unused-recorder-state")
    observations = FakeObservations()
    # Install a clean SIGINT exit before the marker so STOP can be verified
    # without relying on Python's noisy default KeyboardInterrupt behaviour.
    argv = (sys.executable, "-u", "-c",
            "import os, signal, sys, time; "
            "signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); "
            "print('harmless-fixture-pid=' + str(os.getpid()), flush=True); "
            "time.sleep(60)")
    spec = ProcessSpec(GROUP_ID, "Harmless Python fixture", "tracker", argv, TOPICS,
                       ("synthetic-test-resource",), startup_timeout_s=30.)
    supervisor = ProcessSupervisor(
        (spec,), instance_id=model.instance_id, state_dir=tmp_path / "process-state",
        observe=observations, allowed=service.lifecycle_allowed, event=model.event,
        poll_interval_s=.02, interrupt_timeout_s=.5, terminate_timeout_s=.5, kill_timeout_s=.5,
    )
    service.processes = supervisor
    server = SnapshotServer(model, tmp_path / "ipc" / "manager.sock", management=service)
    backend = None
    params = {"group_id": GROUP_ID, "expected_manager_instance_id": model.instance_id}
    try:
        service.start()
        supervisor.start()
        server.start()
        backend = IpcBackend(server.path)
        _wait(lambda: backend.supports_device_control, "negotiated current device capability")
        assert backend._version == CALIBRATION_PROTOCOL_VERSION
        initial = backend.snapshot()
        assert initial["instance_id"] == params["expected_manager_instance_id"]
        assert initial["capabilities"] == {"record": False, "device_control": True,
                                           "capture_plan": False, "calibration": False}
        assert not initial["management"]["enabled"] and initial["management"]["controller"] is None
        assert initial["devices"] == [] and service.store is None and service._quality is None
        assert _group(backend, "STOPPED")["can_start"]
        assert list(supervisor.logs.glob("*.log")) == []  # Connecting never launches.

        old = fetch_snapshot(server.path, version=3)
        assert "process_control" not in old and not old["capabilities"]["device_control"]
        assert not old["management"]["enabled"] and not old["capabilities"]["record"]

        wrong = backend.submit("device.start", {**params, "expected_manager_instance_id": "old-manager"},
                               "wrong-manager")
        _wait(lambda: _receipt(backend, wrong, state="rejected", code="manager_instance_mismatch"),
              "manager-instance fencing")
        assert list(supervisor.logs.glob("*.log")) == []

        start = backend.submit("device.start", params, "start-once")
        _wait(lambda: _receipt(backend, start, state="accepted", code="started"),
              "asynchronous launch receipt through snapshot polling")
        starting = _wait(lambda: _group(backend, "STARTING", can_stop=True), "owned child awaiting topic evidence")
        first_child = supervisor._groups[GROUP_ID].process
        assert first_child is not None and first_child.poll() is None
        assert starting["owned"] and starting["pid"] == first_child.pid and starting["can_stop"]
        first_log = Path(starting["log_path"])
        _wait(lambda: f"harmless-fixture-pid={first_child.pid}" in first_log.read_text(), "child marker")

        # Replaying a completed request exercises server deduplication, not
        # merely the client's in-flight request coalescing.
        assert backend.submit("device.start", params, start) == start
        _wait(lambda: _receipt(backend, start, state="accepted", code="started"), "deduplicated receipt")
        assert supervisor._groups[GROUP_ID].process is first_child
        assert list(supervisor.logs.glob("*.log")) == [first_log]
        assert first_log.read_text().count("harmless-fixture-pid=") == 1

        observations.set(publishers=1, fresh=True, topics=TOPICS[:1])
        one_topic = fetch_snapshot(server.path, version=4)["process_control"]["groups"][0]
        assert one_topic["state"] == "STARTING"  # Every expected topic is required.
        observations.set(publishers=1, fresh=True)
        running = _wait(lambda: _group(backend, "RUNNING"), "all synthetic topics fresh")
        assert running["pid"] == first_child.pid and running["owned"]
        assert running["can_stop"] and running["can_restart"] and not running["can_start"]

        observations.set(publishers=0, fresh=False)
        stop = backend.submit("device.stop", params, "stop-owned")
        _wait(lambda: _receipt(backend, stop, state="accepted", code="stopped"), "asynchronous stop receipt")
        stopped = _wait(lambda: _group(backend, "STOPPED"), "owned process exit")
        _wait(lambda: not supervisor._groups[GROUP_ID].pending, "STOP command settlement")
        assert first_child.poll() == 0
        assert not stopped["owned"] and stopped["pid"] is None and not stopped["can_stop"]

        # Publishers without an owned child are external, never an adoptable
        # process. Neither STOP nor a duplicate START may affect them.
        observations.set(publishers=1, fresh=True)
        external = _wait(lambda: _group(backend, "EXTERNAL"), "synthetic external publishers")
        assert not external["owned"] and external["pid"] is None
        assert not any(external[key] for key in ("can_start", "can_stop", "can_restart"))
        for command, code in (("device.start", "external_publishers"), ("device.stop", "not_owned")):
            identifier = backend.submit(command, params)
            _wait(lambda: _receipt(backend, identifier, state="rejected", code=code), "external ownership guard")
        assert len(list(supervisor.logs.glob("*.log"))) == 1
        assert supervisor._groups[GROUP_ID].process is first_child and first_child.poll() == 0

        observations.set(publishers=0, fresh=False)
        _wait(lambda: _group(backend, "STOPPED"), "external publishers removed")
        detach = backend.submit("device.start", params, "start-to-detach")
        _wait(lambda: _receipt(backend, detach, state="accepted", code="started"), "second explicit launch")
        child = supervisor._groups[GROUP_ID].process
        assert child is not None and child is not first_child and child.poll() is None
        assert len(list(supervisor.logs.glob("*.log"))) == 2
        backend.close()
        assert not backend._thread.is_alive()
        assert child.poll() is None  # Window/client close does not send STOP.
        still_running = fetch_snapshot(server.path, version=4)["process_control"]["groups"][0]
        assert still_running["owned"] and still_running["pid"] == child.pid
        assert still_running["state"] == "STARTING"
    finally:
        if backend is not None:
            backend.close()
        observations.set(publishers=0, fresh=False)
        try:
            # Explicitly clean up our harmless owned child before detaching
            # the supervisor. No process scans, global kills or external PIDs.
            _wait(lambda: not supervisor._groups[GROUP_ID].pending, "fixture command settlement")
            child = supervisor._groups[GROUP_ID].process
            if child is not None and child.poll() is None:
                cleanup = supervisor.command("device.stop", params, "explicit-fixture-cleanup")
                assert cleanup["state"] == "accepted", cleanup
                _wait(lambda: child.poll() is not None, "explicit fixture STOP cleanup")
        finally:
            server.close()
            supervisor.close()
            service.close()
