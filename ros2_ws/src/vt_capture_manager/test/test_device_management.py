"""Device maintenance is independent from, but cannot race, managed recording."""
from copy import deepcopy
from types import SimpleNamespace

from vt_capture_core import encode_message, decode_message
from vt_capture_manager.config import ManagerConfig
from vt_capture_manager.management import ManagementService, receipt
from vt_capture_manager.model import SnapshotModel
from vt_capture_manager.server import SnapshotServer


def context(**changes):
    value = dict(controller_instance_id="controller", state_revision=1, state=0, session_id="",
                 start_request_id="", session_label="", output_path="", recorder_alive=False,
                 elapsed_sec=0., planned_duration_sec=0, topics=[], max_duration_sec=300, detail="idle")
    value.update(changes)
    return value


def service(tmp_path, **kwargs):
    model = SnapshotModel(ManagerConfig((), quality_enabled=False), disk_usage=lambda _: SimpleNamespace(free=100))
    return ManagementService(model, state_dir=tmp_path / "state", **kwargs)


class FakeProcesses:
    busy = False

    def snapshot(self):
        return dict(enabled=True, groups=[])

    def command(self, command, params, identifier):
        self.busy = True
        self.last_command = (command, deepcopy(params))
        return receipt(identifier, "accepted", "queued", "Synthetic process only")

    def operation(self, identifier):
        return receipt(identifier, "accepted", "queued", "Synthetic process only")


def test_device_capability_separate_from_recorder_and_wire_versions(tmp_path):
    manager = service(tmp_path)
    try:
        assert manager.snapshot(4)["process_control"] == dict(enabled=False, groups=[])
        manager.processes = FakeProcesses()
        for version in (1, 2, 3, 4):
            snapshot = manager.snapshot(version)
            encode_message(dict(version=version, request_id="snap", ok=True, snapshot=snapshot))
            assert snapshot["capabilities"]["device_control"] == (version == 4)
            assert not snapshot["capabilities"]["record"]
            assert ("process_control" in snapshot) == (version == 4)
        response = manager.dispatch(dict(version=4, request_id="start-device", command="device.start",
            params=dict(group_id="test", expected_manager_instance_id=manager.model.instance_id)))
        assert response["ok"]
        assert manager.processes.last_command[0] == "device.start"
        assert manager.store is None
    finally:
        manager.close()


def test_no_device_optin_rejects_control(tmp_path):
    manager = service(tmp_path)
    try:
        response = manager.dispatch(dict(version=4, request_id="start-device", command="device.start",
            params=dict(group_id="test", expected_manager_instance_id=manager.model.instance_id)))
        assert not response["ok"]
        assert response["operation"]["code"] == "device_control_disabled"
    finally:
        manager.close()


def test_recorder_state_guards_maintenance_including_external_sessions(tmp_path):
    now = [1.]
    manager = service(tmp_path, monotonic=lambda: now[0])
    try:
        assert manager.lifecycle_allowed("device.start")[0]
        manager.controller_present = lambda: True
        assert not manager.lifecycle_allowed("device.start")[0]
        manager.reconcile(context())
        assert manager.lifecycle_allowed("device.start")[0]
        for state in (1, 2, 3, 4):
            manager.reconcile(context(state=state))
            assert not manager.lifecycle_allowed("device.stop")[0]
        manager.reconcile(context(state=6, recorder_alive=True))
        assert not manager.lifecycle_allowed("device.restart")[0]
        now[0] += 10.
        manager.controller_present = lambda: False
        assert not manager.lifecycle_allowed("device.stop")[0]
        manager.reconcile(context(state=5, recorder_alive=False))
        assert manager.lifecycle_allowed("device.stop")[0]
        now[0] += 10.
        assert manager.lifecycle_allowed("device.start")[0]
    finally:
        manager.close()


def test_capture_start_rejected_during_device_transition(tmp_path):
    manager = service(tmp_path, enabled=True)
    try:
        manager.reconcile(context())
        manager.processes = FakeProcesses()
        manager.processes.busy = True
        batch = manager.dispatch(dict(version=4, request_id="batch", command="batch.create", params=dict(label="test")))
        batch_id = batch["snapshot"]["management"]["batches"][0]["id"]
        response = manager.dispatch(dict(version=4, request_id="record", command="capture.start",
            params=dict(batch_id=batch_id, label="test", duration_s=1,
                        expected_controller_instance_id="controller", expected_state_revision=1)))
        assert response["operation"]["code"] == "device_transition"
        assert not manager.store.active_trials()
    finally:
        manager.close()


def test_unmanaged_server_can_respond_to_v4_snapshot(tmp_path):
    manager = service(tmp_path)
    try:
        server = SnapshotServer(manager.model, tmp_path / "unused.sock")
        value = decode_message(server._reply(encode_message(dict(version=4, request_id="snapshot", command="snapshot", params={}))))
        assert value["snapshot"]["process_control"] == dict(enabled=False, groups=[])
    finally:
        manager.close()
