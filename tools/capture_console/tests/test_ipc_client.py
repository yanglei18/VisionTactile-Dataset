import copy
import socket
import threading
import time

import pytest

from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.backend.ipc_client import IpcBackend, fetch_snapshot
from vt_capture_console.backend import ipc_client
from vt_capture_core import (
    ControllerSnapshot, ControlSnapshot, ManagementSnapshot, ProcessGroupSnapshot,
    ProcessControlSnapshot, DeviceControlSnapshot,
)
from vt_capture_core.protocol import decode_message, encode_message


def serve_once(path, transform, *, version=1):
    ready = threading.Event()
    def serve():
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(path))
            server.listen(1)
            ready.set()
            with server.accept()[0] as connection:
                request = decode_message(connection.recv(4096))
                snapshot = process_snapshot() if version == 4 else control_snapshot() if version >= 2 else DemoBackend().snapshot()
                if version < 4:
                    snapshot.pop("process_control", None)
                snapshot["mode"] = "live"
                message = dict(version=version, request_id=request["request_id"], ok=True, snapshot=snapshot)
                connection.sendall(encode_message(transform(message)))
    worker = threading.Thread(target=serve, daemon=True)
    worker.start()
    assert ready.wait(2)
    return worker


def test_fetch_uses_versioned_protocol(tmp_path):
    path = tmp_path / "backend.sock"
    worker = serve_once(path, lambda m: m)
    assert fetch_snapshot(path)["mode"] == "live"
    worker.join(2)


def test_fetch_v2_over_actual_local_socket(tmp_path):
    path = tmp_path / "v2.sock"
    worker = serve_once(path, lambda m: m, version=2)
    assert fetch_snapshot(path, version=2)["management"]["controller"]["controller_instance_id"] == "controller"
    worker.join(2)


def test_fetch_rejects_response_version_mismatch(tmp_path):
    path = tmp_path / "v1.sock"
    worker = serve_once(path, lambda m: m)
    with pytest.raises(ValueError, match="版本"):
        fetch_snapshot(path, version=2)
    worker.join(2)


def test_live_rejects_demo_response(tmp_path):
    path = tmp_path / "backend.sock"
    def corrupt(message):
        message["snapshot"]["mode"] = "demo"
        return message
    worker = serve_once(path, corrupt)
    with pytest.raises(ValueError, match="演示"):
        fetch_snapshot(path)
    worker.join(2)


def test_disconnection_marks_history_stale_without_mutating_cached_snapshot(tmp_path):
    backend = IpcBackend(tmp_path / "missing.sock")
    backend.close()
    snapshot = DemoBackend().snapshot()
    snapshot["mode"] = "live"
    snapshot["resources"]["disk_free_bytes"] = 10_000_000
    original = copy.deepcopy(snapshot)
    with backend._lock:
        backend._snapshot = snapshot
        backend._received = time.monotonic() - 2
        backend._error = "断开"
    current = backend.snapshot()
    assert all(d["status"] == "stale" for d in current["devices"])
    assert current["recording"]["state"] == "UNAVAILABLE"
    assert current["resources"]["disk_free_bytes"] is None
    assert current["devices"][0]["preview"] == original["devices"][0]["preview"]
    assert snapshot == original
    assert not backend._thread.is_alive()


def test_live_disconnected_window_disables_control(qapp, tmp_path):
    from vt_capture_console.ui.main_window import MainWindow
    backend = IpcBackend(tmp_path / "missing.sock")
    window = MainWindow(backend)
    try:
        assert not window.start_button.isEnabled()
        assert not window.stop_button.isEnabled()
        assert window.last_snapshot["mode"] == "live"
    finally:
        window.close()


def control_snapshot():
    value = ControlSnapshot("live", "manager", management=ManagementSnapshot(
        enabled=True, controller=ControllerSnapshot("controller"))).to_dict()
    value["capabilities"]["record"] = True
    return value


def process_snapshot():
    value = DeviceControlSnapshot("live", "manager", process_control=ProcessControlSnapshot(
        enabled=True, groups=[ProcessGroupSnapshot(
            "cameras", "Cameras", "camera", state="RUNNING", owned=True, pid=123,
            detail="alive", log_path="/logs/cameras.log", can_stop=True,
            can_restart=True, topics=["/camera/image_raw"])]),
        management=ManagementSnapshot(enabled=False)).to_dict()
    value["capabilities"]["device_control"] = True
    return value


@pytest.fixture
def idle_backend(monkeypatch, tmp_path):
    monkeypatch.setattr(IpcBackend, "_run", lambda self: None)
    backend = IpcBackend(tmp_path / "test.sock")
    backend._accept_snapshot(control_snapshot(), 2)
    yield backend
    backend.close()


def operation(identifier, state="accepted", trial_id="trial"):
    return {"request_id": identifier, "state": state, "code": "QUEUED" if state == "accepted" else "FAILED",
            "detail": "accepted intent, not a claim that recording started", "trial_id": trial_id}


def test_submit_is_bounded_deduplicated_and_detaches_params(idle_backend):
    backend = idle_backend
    params = {"label": "first"}
    assert backend.submit("batch.create", params, "same") == "same"
    params["label"] = "mutated"
    assert backend.submit("batch.create", {"label": "first"}, "same") == "same"
    assert backend._commands.qsize() == 1
    with pytest.raises(ValueError, match="request_id"):
        backend.submit("batch.create", {"label": "different"}, "same")
    for index in range(ipc_client.MAX_PENDING_COMMANDS - 1):
        backend.submit("batch.create", {"label": "other"}, f"request-{index}")
    backend.submit("batch.create", {"label": "too many"}, "overflow")
    assert backend.latest_operation["code"] == "QUEUE_FULL"
    assert backend.latest_operation["state"] == "rejected"
    assert len(backend._pending) == ipc_client.MAX_PENDING_COMMANDS
    assert backend._commands.get_nowait()["params"]["label"] == "first"


@pytest.mark.parametrize("command,params", [
    ("shell", {"cmd": "echo"}), ("capture.start", {}),
    ("capture.start", {"batch_id": "b", "label": "t", "duration_s": True,
        "expected_controller_instance_id": "c", "expected_state_revision": 0}),
])
def test_invalid_submit_never_queues(idle_backend, command, params):
    with pytest.raises(ValueError):
        idle_backend.submit(command, params)
    assert idle_backend._commands.empty()


def test_command_reply_beats_queued_snapshot_and_returns_detached_receipt(idle_backend, monkeypatch):
    backend = idle_backend
    identifier = backend.submit("batch.create", {"label": "batch"}, "request")
    assert backend.latest_operation["code"] == "QUEUED"
    assert backend.latest_operation["state"] == "unknown"
    receipt = operation(identifier, trial_id="")
    snapshot = control_snapshot()
    snapshot["management"]["operation"] = receipt
    monkeypatch.setattr(ipc_client, "_exchange", lambda *args, **kwargs: {
        "version": 2, "request_id": identifier, "ok": True, "snapshot": snapshot, "operation": receipt})
    backend._send_command(backend._commands.get_nowait())
    assert backend.latest_operation == receipt
    assert backend.snapshot()["management"]["operation"] == receipt
    copy_of_receipt = backend.latest_operation
    copy_of_receipt["state"] = "rejected"
    assert backend.latest_operation["state"] == "accepted"


def test_timeout_is_unknown_not_auto_retried_and_can_reconcile_matching_receipt(idle_backend, monkeypatch):
    backend = idle_backend
    calls = []
    def timeout(path, request, **kwargs):
        calls.append(copy.deepcopy(request))
        raise TimeoutError("no reply")
    monkeypatch.setattr(ipc_client, "_exchange", timeout)
    identifier = backend.submit("batch.create", {"label": "batch"}, "unchanged-id")
    backend._send_command(backend._commands.get_nowait())
    assert len(calls) == 1 and calls[0]["request_id"] == identifier
    assert backend._commands.empty() and not backend._pending
    assert backend.latest_operation["state"] == "unknown"
    assert backend.latest_operation["code"] == "TRANSPORT_UNKNOWN"
    healthy = control_snapshot()
    backend._accept_snapshot(healthy, 2)
    assert backend.latest_operation["state"] == "unknown"
    healthy["management"]["operation"] = operation(identifier, trial_id="recovered-trial")
    backend._accept_snapshot(healthy, 2)
    assert backend.latest_operation["trial_id"] == "recovered-trial"
    assert backend.latest_operation["state"] == "accepted"
    assert len(calls) == 1


def test_v1_fallback_never_grants_control_even_if_server_advertises_record(idle_backend, monkeypatch):
    calls = []
    readonly = DemoBackend().snapshot()
    readonly["mode"] = "live"
    readonly["capabilities"]["record"] = True
    def fetch(path, timeout=.6, *, version=1):
        calls.append(version)
        if version >= 2:
            raise ValueError("v1 only")
        return readonly
    monkeypatch.setattr(ipc_client, "fetch_snapshot", fetch)
    idle_backend._poll()
    assert calls == [4, 3, 2, 1]
    assert not idle_backend.snapshot()["capabilities"]["record"]
    assert not idle_backend.snapshot()["management"]["enabled"]
    idle_backend.submit("batch.create", {"label": "batch"}, "readonly")
    assert idle_backend.latest_operation["code"] == "READ_ONLY_PROTOCOL"
    assert idle_backend._commands.empty()
    idle_backend._poll()
    assert calls == [4, 3, 2, 1, 1]


def test_disconnected_management_keeps_history_but_not_live_controller_or_active_trial(idle_backend):
    snapshot = control_snapshot()
    snapshot["resources"]["disk_free_bytes"] = 999
    snapshot["management"]["trials"] = [{"id": "trial", "batch_id": "batch", "label": "trial", "state": "RECORDING",
        "session_id": "session", "output_path": "/data/history", "duration_s": 30., "elapsed_s": 1.,
        "quality": "NOT_ASSESSED", "detail": "original", "created_at": "2026-10-07T00:00:00Z"}]
    idle_backend._accept_snapshot(snapshot, 2)
    idle_backend._received -= 2
    value = idle_backend.snapshot()
    assert value["management"]["controller"] is None
    assert value["management"]["enabled"] is False
    assert value["management"]["trials"][0]["state"] == "UNKNOWN"
    assert value["management"]["trials"][0]["output_path"] == "/data/history"
    assert "历史" in value["management"]["trials"][0]["detail"]
    assert snapshot["management"]["trials"][0]["state"] == "RECORDING"
    assert not value["capabilities"]["record"] and value["resources"]["disk_free_bytes"] is None
    # A verified downgrade is read-only but must not erase this window's history.
    older_server = DemoBackend().snapshot()
    older_server["mode"] = "live"
    idle_backend._accept_snapshot(older_server, 1)
    downgraded = idle_backend.snapshot()["management"]
    assert downgraded["trials"][0]["output_path"] == "/data/history"
    assert downgraded["trials"][0]["state"] == "UNKNOWN"
    assert "只读协议" in downgraded["trials"][0]["detail"]
    assert downgraded["controller"] is None and not downgraded["enabled"]


def test_queued_request_not_sent_after_disconnect(idle_backend, monkeypatch):
    idle_backend.submit("batch.create", {"label": "batch"}, "queued")
    idle_backend._error = "disconnected"
    monkeypatch.setattr(ipc_client, "_exchange", lambda *args, **kwargs: pytest.fail("must not send"))
    idle_backend._send_command(idle_backend._commands.get_nowait())
    assert idle_backend.latest_operation["state"] == "rejected"
    assert idle_backend.latest_operation["code"] == "BACKEND_UNAVAILABLE"


def test_control_network_call_is_background_and_submit_does_not_wait(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    callers = []
    monkeypatch.setattr(ipc_client, "fetch_snapshot", lambda *args, **kwargs: control_snapshot())
    def exchange(path, request, **kwargs):
        callers.append(threading.get_ident())
        entered.set()
        assert release.wait(2)
        return {"version": 2, "request_id": request["request_id"], "ok": True,
                "snapshot": control_snapshot(), "operation": operation(request["request_id"])}
    monkeypatch.setattr(ipc_client, "_exchange", exchange)
    backend = IpcBackend(tmp_path / "unused.sock")
    try:
        deadline = time.monotonic() + 2
        while not backend.snapshot()["management"]["enabled"] and time.monotonic() < deadline:
            threading.Event().wait(.005)
        started = time.monotonic()
        backend.submit("batch.create", {"label": "batch"}, "background")
        assert time.monotonic() - started < .2
        assert entered.wait(2)
        assert callers == [backend._thread.ident]
        assert callers[0] != threading.get_ident()
        assert backend.latest_operation["state"] == "unknown"
    finally:
        release.set()
        backend.close()
    assert not backend._thread.is_alive()


def trial_details(trial_id="trial"):
    return {"trial_id": trial_id, "metadata": {"start": None, "finish": None},
            "quality": {"state": "NOT_ASSESSED", "verdict": "", "detail": "", "attempt": 0,
                        "report_path": "", "report": None},
            "artifacts": {"start_metadata_path": "", "finish_metadata_path": ""}}


def detail_reply(request, *, instance="manager"):
    snapshot = control_snapshot()
    snapshot["instance_id"] = instance
    return {"version": 3, "request_id": request["request_id"], "ok": True, "snapshot": snapshot,
            "trial_details": trial_details(request["params"]["trial_id"])}


@pytest.fixture
def detailed_backend(idle_backend):
    idle_backend._accept_snapshot(control_snapshot(), 3)
    return idle_backend


def test_fetch_v3_over_actual_socket_and_snapshot_does_not_carry_details(tmp_path):
    path = tmp_path / "v3.sock"
    worker = serve_once(path, lambda message: message, version=3)
    assert fetch_snapshot(path, version=3)["management"]["enabled"]
    worker.join(2)
    path = tmp_path / "wrong-details.sock"
    worker = serve_once(path, lambda message: dict(message, trial_details=trial_details()), version=3)
    with pytest.raises(ValueError, match="详情"):
        fetch_snapshot(path, version=3)
    worker.join(2)


def test_v3_to_v2_fallback_preserves_recording_but_rejects_new_features(idle_backend, monkeypatch):
    calls = []
    def fetch(path, timeout=.6, *, version=1):
        calls.append(version)
        if version >= 3:
            raise ValueError("v2 only")
        return control_snapshot()
    monkeypatch.setattr(ipc_client, "fetch_snapshot", fetch)
    idle_backend._poll()
    assert calls == [4, 3, 2]
    assert not idle_backend.supports_details
    assert idle_backend.snapshot()["capabilities"]["record"]
    identifier = idle_backend.request_details("trial")
    assert idle_backend.details_status == {"trial_id": "trial", "request_id": identifier,
        "state": "unsupported", "error": "当前后台不支持试次详情，需要 v3"}
    idle_backend.submit("quality.retry", {"trial_id": "trial"}, "retry")
    assert idle_backend.latest_operation["code"] == "UNSUPPORTED_FEATURE"
    params = {"batch_id": "b", "label": "t", "duration_s": 1, "expected_controller_instance_id": "c", "expected_state_revision": 0}
    idle_backend.submit("capture.start", {**params, "task_name": "new"}, "metadata")
    assert idle_backend.latest_operation["code"] == "UNSUPPORTED_FEATURE"
    idle_backend.submit("capture.start", params, "start")
    assert idle_backend._commands.get_nowait()["version"] == 2
    idle_backend.submit("capture.stop", {"trial_id": "trial", "expected_controller_instance_id": "c", "expected_session_id": "s"}, "stop")
    assert idle_backend._commands.get_nowait()["version"] == 2
    idle_backend._poll()
    assert calls == [4, 3, 2, 2]


def test_details_cache_is_selected_only_detached_and_does_not_overwrite_operation(detailed_backend, monkeypatch):
    backend = detailed_backend
    backend.submit("batch.create", {"label": "batch"}, "control")
    original_operation = backend.latest_operation
    def exchange(path, request, **kwargs):
        reply = detail_reply(request)
        reply["snapshot"]["management"]["operation"] = operation("another-client")
        return reply
    monkeypatch.setattr(ipc_client, "_exchange", exchange)
    identifier = backend.request_details("selected")
    assert backend.request_details("selected") == identifier  # Coalesce same in-flight selection.
    assert backend.latest_details is None and backend.details_status["state"] == "loading"
    backend._send_details(*backend._pending_details)
    assert backend.latest_operation == original_operation
    assert backend.latest_details["trial_id"] == "selected"
    assert backend.details_status["state"] == "ready"
    result = backend.latest_details
    result["quality"]["state"] = "ERROR"
    assert backend.latest_details["quality"]["state"] == "NOT_ASSESSED"
    backend.request_details("different")
    assert backend.latest_details is None


def test_late_details_cannot_replace_a_new_selection(detailed_backend, monkeypatch):
    backend = detailed_backend
    backend.request_details("old")
    old_request = backend._pending_details
    def old_response(path, request, **kwargs):
        backend.request_details("new")
        return detail_reply(request)
    monkeypatch.setattr(ipc_client, "_exchange", old_response)
    backend._send_details(*old_request)
    assert backend.latest_details is None
    assert backend.details_status["trial_id"] == "new"
    assert backend.details_status["state"] == "loading"
    monkeypatch.setattr(ipc_client, "_exchange", lambda path, request, **kwargs: detail_reply(request))
    backend._send_details(*backend._pending_details)
    assert backend.latest_details["trial_id"] == "new"


def test_detail_requests_have_only_one_pending_slot(detailed_backend):
    for index in range(100):
        detailed_backend.request_details(f"trial-{index}")
    assert detailed_backend._pending_details[0]["params"] == {"trial_id": "trial-99"}
    assert detailed_backend._commands.empty()
    assert detailed_backend.latest_operation is None


def test_cleared_selection_discards_inflight_response(detailed_backend, monkeypatch):
    detailed_backend.request_details("trial")
    pending = detailed_backend._pending_details
    def exchange(path, request, **kwargs):
        detailed_backend.clear_details()
        return detail_reply(request)
    monkeypatch.setattr(ipc_client, "_exchange", exchange)
    detailed_backend._send_details(*pending)
    assert detailed_backend.latest_details is None
    assert detailed_backend._pending_details is None
    assert detailed_backend.details_status == {"trial_id": "", "request_id": "", "state": "idle", "error": ""}


def test_details_disconnect_stays_stale_after_reconnect_until_refetched(detailed_backend, monkeypatch):
    backend = detailed_backend
    monkeypatch.setattr(ipc_client, "_exchange", lambda path, request, **kwargs: detail_reply(request))
    backend.request_details("trial")
    backend._send_details(*backend._pending_details)
    assert backend.supports_details and backend.details_status["state"] == "ready"
    backend._error = "disconnected"
    assert not backend.supports_details
    assert backend.latest_details["trial_id"] == "trial"
    assert backend.details_status["state"] == "stale"
    backend._accept_snapshot(control_snapshot(), 3)
    assert backend.supports_details and backend.details_status["state"] == "stale"
    backend.request_details("trial")
    backend._send_details(*backend._pending_details)
    assert backend.details_status["state"] == "ready"
    backend._received -= 2
    assert backend.details_status["state"] == "stale"


@pytest.mark.parametrize("kind", ["other-trial", "other-manager", "missing", "rejected", "timeout"])
def test_bad_detail_replies_cannot_create_fresh_cache(detailed_backend, monkeypatch, kind):
    def exchange(path, request, **kwargs):
        if kind == "timeout":
            raise TimeoutError("no reply")
        reply = detail_reply(request, instance="replacement" if kind == "other-manager" else "manager")
        if kind == "other-trial":
            reply["trial_details"]["trial_id"] = "wrong"
        elif kind == "missing":
            reply.pop("trial_details")
        elif kind == "rejected":
            reply.update(ok=False, error="not found")
        return reply
    monkeypatch.setattr(ipc_client, "_exchange", exchange)
    detailed_backend.request_details("trial")
    detailed_backend._send_details(*detailed_backend._pending_details)
    assert detailed_backend.latest_details is None
    assert detailed_backend.details_status["state"] == "error"
    assert detailed_backend.details_status["error"]
    assert detailed_backend.latest_operation is None


def test_quality_retry_uses_operation_queue_and_does_not_overwrite_details(detailed_backend, monkeypatch):
    backend = detailed_backend
    backend._latest_details = trial_details()
    original_details = backend.latest_details
    backend._snapshot["capabilities"]["record"] = False  # QC does not require recorder controls.
    identifier = backend.submit("quality.retry", {"trial_id": "trial"}, "qc-retry")
    request = backend._commands.get_nowait()
    assert request["command"] == "quality.retry" and request["version"] == 3
    calls = []
    def timeout(path, request, **kwargs):
        calls.append(request["request_id"])
        raise TimeoutError("unconfirmed")
    monkeypatch.setattr(ipc_client, "_exchange", timeout)
    backend._send_command(request)
    assert calls == [identifier]
    assert backend.latest_operation["state"] == "unknown"
    assert backend.latest_details == original_details
    assert backend._commands.empty()


def test_control_commands_take_priority_over_pending_detail_request(tmp_path, monkeypatch):
    original_run = IpcBackend._run
    start, detailed = threading.Event(), threading.Event()
    order = []
    def wait_then_run(self):
        assert start.wait(2)
        original_run(self)
    monkeypatch.setattr(IpcBackend, "_run", wait_then_run)
    monkeypatch.setattr(ipc_client, "fetch_snapshot", lambda *args, **kwargs: control_snapshot())
    def exchange(path, request, **kwargs):
        order.append((request["command"], threading.get_ident()))
        if request["command"] == "trial.details":
            detailed.set()
            return detail_reply(request)
        return {"version": 3, "request_id": request["request_id"], "ok": True, "snapshot": control_snapshot(),
                "operation": operation(request["request_id"])}
    monkeypatch.setattr(ipc_client, "_exchange", exchange)
    backend = IpcBackend(tmp_path / "unused.sock")
    try:
        backend._accept_snapshot(control_snapshot(), 3)
        backend.request_details("trial")
        backend.submit("batch.create", {"label": "batch"}, "command")
        start.set()
        assert detailed.wait(2)
    finally:
        start.set()
        backend.close()
    assert [name for name, _ in order] == ["batch.create", "trial.details"]
    assert all(identifier == backend._thread.ident for _, identifier in order)


@pytest.fixture
def process_backend(idle_backend):
    idle_backend._accept_snapshot(process_snapshot(), 4)
    return idle_backend


def test_fetch_v4_over_actual_socket(tmp_path):
    path = tmp_path / "v4.sock"
    worker = serve_once(path, lambda message: message, version=4)
    snapshot = fetch_snapshot(path, version=4)
    assert snapshot["process_control"]["enabled"]
    assert snapshot["process_control"]["groups"][0]["id"] == "cameras"
    worker.join(2)


def test_v4_is_preferred_and_v3_fallback_keeps_details(idle_backend, monkeypatch):
    calls = []
    def current(path, timeout=.6, *, version=1):
        calls.append(version)
        return process_snapshot()
    monkeypatch.setattr(ipc_client, "fetch_snapshot", current)
    idle_backend._poll()
    assert calls == [4]
    assert idle_backend.supports_device_control and idle_backend.supports_details
    def older(path, timeout=.6, *, version=1):
        calls.append(version)
        if version == 4:
            raise ValueError("v3 only")
        return control_snapshot()
    monkeypatch.setattr(ipc_client, "fetch_snapshot", older)
    idle_backend._poll()
    assert calls == [4, 4, 3]
    assert not idle_backend.supports_device_control and idle_backend.supports_details
    idle_backend.request_details("trial")
    assert idle_backend._pending_details[0]["version"] == 3
    idle_backend._poll()
    assert calls == [4, 4, 3, 3]
    idle_backend._next_v2_probe = 0
    monkeypatch.setattr(ipc_client, "fetch_snapshot", current)
    idle_backend._poll()
    assert calls[-1] == 4 and idle_backend.supports_device_control


@pytest.mark.parametrize("command", ["device.start", "device.stop", "device.restart"])
def test_device_actions_use_shared_queue_without_recording_management(process_backend, command):
    backend = process_backend
    assert not backend.snapshot()["management"]["enabled"]
    assert not backend.snapshot()["capabilities"]["record"]
    params = {"group_id": "cameras", "expected_manager_instance_id": "manager"}
    assert backend.submit(command, params, "device-command") == "device-command"
    params["group_id"] = "changed"
    assert backend.submit(command, {"group_id": "cameras", "expected_manager_instance_id": "manager"}, "device-command") == "device-command"
    assert backend._commands.qsize() == 1
    assert backend.latest_operation["state"] == "unknown"
    assert backend.latest_operation["code"] == "QUEUED"
    request = backend._commands.get_nowait()
    assert request["version"] == 4 and request["command"] == command
    assert request["params"]["group_id"] == "cameras"


@pytest.mark.parametrize("version", [1, 2, 3])
def test_older_protocol_never_grants_device_control(process_backend, version):
    snapshot = control_snapshot()
    if version == 1:
        snapshot.pop("management")
    snapshot["capabilities"]["device_control"] = True
    process_backend._accept_snapshot(snapshot, version)
    assert not process_backend.supports_device_control
    process_backend.submit("device.start", {"group_id": "cameras", "expected_manager_instance_id": "manager"})
    assert process_backend.latest_operation["code"] == "UNSUPPORTED_FEATURE"
    assert process_backend._commands.empty()
    current = process_backend.snapshot()
    assert not current["capabilities"]["device_control"]
    assert not current["process_control"]["enabled"]
    group = current["process_control"]["groups"][0]
    assert group["state"] == "UNKNOWN" and group["pid"] == 123
    assert not any(group[key] for key in ("can_start", "can_stop", "can_restart"))
    assert process_backend._process_history["groups"][0]["state"] == "RUNNING"


@pytest.mark.parametrize("unavailable,code", [
    ("disabled", "PROCESS_CONTROL_DISABLED"), ("capability", "CONTROL_UNAVAILABLE"),
    ("stale", "BACKEND_UNAVAILABLE"), ("disconnected", "BACKEND_UNAVAILABLE"),
    ("closed", "CLIENT_CLOSED"),
])
def test_device_commands_fail_closed_and_cached_actions_disable(process_backend, unavailable, code):
    backend = process_backend
    if unavailable == "disabled":
        backend._snapshot["process_control"]["enabled"] = False
    elif unavailable == "capability":
        backend._snapshot["capabilities"]["device_control"] = False
    elif unavailable == "stale":
        backend._received -= 2
    elif unavailable == "disconnected":
        backend._error = "disconnected"
    else:
        backend.close()
    assert not backend.supports_device_control
    backend.submit("device.stop", {"group_id": "cameras", "expected_manager_instance_id": "manager"})
    assert backend.latest_operation["code"] == code
    assert backend._commands.empty()
    snapshot = backend.snapshot()
    group = snapshot["process_control"]["groups"][0]
    assert not any(group[key] for key in ("can_start", "can_stop", "can_restart"))
    if unavailable in {"stale", "disconnected", "closed"}:
        assert group["state"] == "UNKNOWN"
        assert not snapshot["process_control"]["enabled"]
    assert backend._snapshot["process_control"]["groups"][0]["state"] == "RUNNING"


def test_queued_device_command_not_sent_after_downgrade(process_backend, monkeypatch):
    process_backend.submit("device.restart", {"group_id": "cameras", "expected_manager_instance_id": "manager"})
    process_backend._accept_snapshot(control_snapshot(), 3)
    monkeypatch.setattr(ipc_client, "_exchange", lambda *args, **kwargs: pytest.fail("must not send"))
    process_backend._send_command(process_backend._commands.get_nowait())
    assert process_backend.latest_operation["state"] == "rejected"
    assert not process_backend._pending


def test_device_timeout_never_retries_and_snapshot_reconciles_shared_receipt(process_backend, monkeypatch):
    calls = []
    def timeout(path, request, **kwargs):
        calls.append(request)
        raise TimeoutError("no reply")
    monkeypatch.setattr(ipc_client, "_exchange", timeout)
    process_backend.submit("device.restart", {"group_id": "cameras", "expected_manager_instance_id": "manager"}, "restart")
    process_backend._send_command(process_backend._commands.get_nowait())
    assert len(calls) == 1
    assert process_backend.latest_operation["state"] == "unknown"
    assert process_backend.latest_operation["code"] == "TRANSPORT_UNKNOWN"
    assert not process_backend._pending and process_backend._commands.empty()
    assert process_backend.snapshot()["process_control"]["groups"][0]["state"] == "UNKNOWN"
    snapshot = process_snapshot()
    snapshot["management"]["operation"] = operation("restart", trial_id="")
    process_backend._accept_snapshot(snapshot, 4)
    assert process_backend.latest_operation["state"] == "accepted"
    assert process_backend.latest_operation["trial_id"] == ""
    assert process_backend.supports_device_control


@pytest.mark.parametrize("before,after", [(4, 4), (3, 4), (4, 3)])
def test_details_use_current_negotiated_protocol_without_overwriting_device_receipt(process_backend, monkeypatch, before, after):
    backend = process_backend
    backend.submit("device.start", {"group_id": "cameras", "expected_manager_instance_id": "manager"}, "device")
    original = backend.latest_operation
    backend._accept_snapshot(process_snapshot() if before == 4 else control_snapshot(), before)
    backend.request_details("trial")
    assert backend._pending_details[0]["version"] == before
    backend._accept_snapshot(process_snapshot() if after == 4 else control_snapshot(), after)
    calls = []
    def exchange(path, request, **kwargs):
        calls.append(request["version"])
        return {"version": request["version"], "request_id": request["request_id"], "ok": True,
                "snapshot": process_snapshot() if after == 4 else control_snapshot(),
                "trial_details": trial_details()}
    monkeypatch.setattr(ipc_client, "_exchange", exchange)
    backend._send_details(*backend._pending_details)
    assert calls == [after] and backend._version == after
    assert backend.latest_details["trial_id"] == "trial" and backend.details_status["state"] == "ready"
    assert backend.latest_operation == original
