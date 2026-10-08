from copy import deepcopy

from PySide6.QtCore import Qt
from PySide6.QtTest import QTest

from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.ui.main_window import MainWindow
from vt_capture_console.viewmodels.recording import recording_access


class ManagedBackend:
    mode = "live"
    path = "/test-only/manager.sock"

    def __init__(self):
        self.value = DemoBackend().snapshot()
        self.value.update(mode="live")
        self.value["capabilities"]["record"] = True
        self.value["management"] = {
            "enabled": True,
            "controller": {"controller_instance_id": "controller-a", "state_revision": 7,
                           "state": 0, "session_id": "", "recorder_alive": False},
            "batches": [{"id": "b1", "label": "硬件测试", "created_at": "2026-01-01T00:00:00Z"}],
            "trials": [], "operation": None,
        }
        self.latest_operation = None
        self.commands = []
        self.closed = False

    def snapshot(self):
        return deepcopy(self.value)

    def submit(self, command, params, request_id=None):
        request_id = request_id or f"request-{len(self.commands)}"
        self.commands.append((request_id, command, deepcopy(params)))
        self.latest_operation = dict(request_id=request_id, state="unknown", code="QUEUED", detail="queued", trial_id="")
        return request_id

    def confirm_recording(self):
        self.value["management"]["controller"].update(state=3, state_revision=8, session_id="s1", recorder_alive=True)
        self.value["management"]["trials"] = [{"id": "t1", "batch_id": "b1", "label": "采集试次",
            "state": "RECORDING", "session_id": "s1", "output_path": "/test-only/session/bag",
            "duration_s": 30., "elapsed_s": 1., "quality": "NOT_ASSESSED", "detail": "recording", "created_at": "2026-01-01T00:00:00Z"}]
        self.value["recording"].update(state="RECORDING", session_id="s1", elapsed_s=1., duration_s=30.)
        self.latest_operation.update(state="accepted", code="started", trial_id="t1")

    def close(self):
        self.closed = True


def test_explicit_start_is_bound_and_queued_is_not_success(qapp):
    backend = ManagedBackend()
    window = MainWindow(backend)
    try:
        assert backend.commands == []
        assert window.start_button.isEnabled()
        window.start_button.click()
        assert len(backend.commands) == 1
        _, command, params = backend.commands[0]
        assert command == "capture.start"
        assert params == dict(batch_id="b1", label="采集试次", duration_s=30,
                              expected_controller_instance_id="controller-a", expected_state_revision=7)
        assert window.pending_request
        assert not window.start_button.isEnabled()
        window.refresh()
        window.start_button.click()
        assert len(backend.commands) == 1
        backend.confirm_recording()
        window.refresh()
        assert not window.pending_request
        assert window.stop_button.isEnabled()
        assert window.trial_table.item(0, 4).text() == "未执行"
    finally:
        window.close()
    assert backend.closed
    assert len(backend.commands) == 1  # Closing a window never sends STOP.


def test_stop_targets_owned_session_only(qapp):
    backend = ManagedBackend()
    backend.latest_operation = dict(request_id="old", state="accepted", code="started", detail="", trial_id="t1")
    backend.confirm_recording()
    window = MainWindow(backend)
    try:
        assert window.stop_button.isEnabled()
        window.stop_button.click()
        assert backend.commands[-1][1:] == ("capture.stop", dict(trial_id="t1",
            expected_controller_instance_id="controller-a", expected_session_id="s1"))
        assert window.pending_request
    finally:
        window.close()


def test_external_session_has_no_stop_access():
    backend = ManagedBackend()
    backend.value["management"]["controller"].update(state=3, session_id="external", recorder_alive=True)
    assert recording_access(backend.value, backend.value["management"]) == (False, False)


def test_unknown_outcome_does_not_retry(qapp):
    backend = ManagedBackend()
    window = MainWindow(backend)
    try:
        window.start_button.click()
        backend.latest_operation.update(state="unknown", code="OUTCOME_UNKNOWN", detail="响应超时；尚不能判断是否录制")
        backend.value["capabilities"]["record"] = False
        backend.value["management"]["controller"] = None
        window.refresh()
        assert not window.pending_request
        assert not window.start_button.isEnabled()
        for _ in range(5):
            window.refresh()
        assert len(backend.commands) == 1
        assert "OUTCOME_UNKNOWN" in window.operation_notice.text()
    finally:
        window.close()


def test_demo_batch_metadata_remains_in_memory(window, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    window.navigation.button(1).click()
    window.batch_name.setText("拣选动作")
    QTest.mouseClick(window.create_batch_button, Qt.MouseButton.LeftButton)
    assert window.batch_selector.currentText() == "拣选动作"
    window.trial_name.setText("第一次抓取")
    window.start_button.click()
    assert window.backend.trials[0]["label"] == "第一次抓取"
    assert window.trial_table.item(0, 1).text() == "拣选动作"
    window.stop_button.click()
    assert window.backend.state == "COMPLETE"
    assert list(tmp_path.iterdir()) == []


def test_other_client_batch_cannot_complete_queued_request(qapp):
    backend = ManagedBackend()
    window = MainWindow(backend)
    try:
        window.batch_name.setText("我的批次")
        window.create_batch_button.click()
        request = window.pending_request
        backend.value["management"]["batches"].insert(0, {
            "id": "other-client-batch", "label": "其他窗口的批次", "created_at": "2026-01-01T00:00:01Z"})
        window.refresh()
        assert window.pending_request == request
        assert window.batch_selector.currentData() == "b1"
        assert not window.start_button.isEnabled()

        backend.value["management"]["batches"].insert(0, {
            "id": "my-batch", "label": "我的批次", "created_at": "2026-01-01T00:00:02Z"})
        backend.latest_operation.update(state="accepted", code="batch_created")
        window.refresh()
        assert window.pending_request is None
        assert window.batch_selector.currentData() is None
        window.start_button.click()
        assert len(backend.commands) == 1  # Must select a batch explicitly.
        window.batch_selector.setCurrentIndex(window.batch_selector.findData("my-batch"))
        window.start_button.click()
        assert backend.commands[-1][2]["batch_id"] == "my-batch"
    finally:
        window.close()


def test_changed_backend_without_old_trial_does_not_lock_ui_forever(qapp):
    backend = ManagedBackend()
    window = MainWindow(backend)
    try:
        window.start_button.click()
        backend.latest_operation.update(state="accepted", code="intent_saved", trial_id="old-trial")
        window.refresh()
        assert window.pending_request
        backend.value["instance_id"] = "new-manager-instance"
        backend.value["management"]["controller"]["controller_instance_id"] = "new-controller-instance"
        window.refresh()
        assert window.pending_request is None
        assert window.create_batch_button.isEnabled()
        assert "待核实" in window.operation_notice.text()
        assert len(backend.commands) == 1
    finally:
        window.close()
