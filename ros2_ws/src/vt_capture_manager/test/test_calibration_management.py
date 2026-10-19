"""The existing admission lock also owns calibration and activation admission."""
from types import SimpleNamespace
from copy import deepcopy

from vt_capture_core import decode_message, encode_message
from vt_capture_core.calibration_protocol import empty_calibration
from vt_capture_manager.management import ManagementService
from vt_capture_manager.model import SnapshotModel
from vt_capture_manager.config import ManagerConfig
from test_management import FakeAdapter, start


class FakeCalibration:
    busy = True
    closed = False

    def snapshot(self):
        return empty_calibration() | dict(enabled=True, busy=self.busy)

    def command(self, command, params, request_id):
        return dict(accepted=True, code="cancel_requested", detail="Stopping own recorder", session_id="session")

    def close(self):
        self.closed = True


def service(tmp_path):
    model = SnapshotModel(ManagerConfig((), quality_enabled=False), disk_usage=lambda _: SimpleNamespace(free=100))
    return ManagementService(model, enabled=True, state_dir=tmp_path / "state")


def test_v6_snapshot_and_legacy_wire_are_separate(tmp_path):
    manager = service(tmp_path)
    manager.calibration = FakeCalibration()
    try:
        for version in range(1, 7):
            value = manager.snapshot(version)
            assert ("calibration" in value) == (version == 6)
            response = dict(version=version, request_id="r", ok=True, snapshot=value)
            assert decode_message(encode_message(response)) == response
        assert manager.snapshot(6)["capabilities"]["calibration"]
    finally:
        manager.close()


def test_busy_calibration_blocks_device_maintenance(tmp_path):
    manager = service(tmp_path)
    manager.calibration = FakeCalibration()
    try:
        allowed, detail = manager.lifecycle_allowed("stop")
        assert not allowed and detail
        assert manager.calibration_allowed()[0]
    finally:
        manager.close()


def test_calibration_dispatch_uses_existing_receipt_envelope(tmp_path):
    manager = service(tmp_path)
    task = manager.calibration = FakeCalibration()
    try:
        result = manager.dispatch(dict(version=6, request_id="r", command="calibration.cancel",
            params=dict(expected_manager_instance_id=manager.model.instance_id, session_id="session")))
        assert result["operation"]["state"] == "accepted"
        assert decode_message(encode_message(result)) == result
    finally:
        manager.close()
    assert task.closed


def test_calibration_and_durable_capture_intent_exclude_each_other(tmp_path):
    manager = service(tmp_path)
    adapter = manager.adapter = FakeAdapter()
    adapter.store = manager.store
    task = manager.calibration = FakeCalibration()
    task.active_documents = lambda: []
    try:
        assert manager.poll_once()
        rejected = start(manager, adapter, identifier="during-calibration")
        assert rejected["operation"]["code"] == "calibration_busy"
        assert not manager.store.active_trials() and manager._commands.empty()
        task.busy = False
        accepted = start(manager, adapter, identifier="after-calibration")
        assert accepted["operation"]["state"] == "accepted"
        # Admission closes when the intent is durable, before ROS sees START.
        assert not adapter.commands
        assert not manager.calibration_allowed()[0]
    finally:
        manager.close()


def test_capture_freezes_calibration_version_at_start(tmp_path):
    manager = service(tmp_path)
    adapter = manager.adapter = FakeAdapter()
    adapter.store = manager.store
    task = manager.calibration = FakeCalibration()
    task.busy = False
    active = [(dict(camera=dict(name="synthetic")), dict(revision="original"))]
    task.active_documents = lambda: deepcopy(active)
    try:
        assert manager.poll_once()
        result = start(manager, adapter)
        assert result["operation"]["state"] == "accepted"
        trial = result["operation"]["trial_id"]
        active[0][1]["revision"] = "future"
        frozen = manager.store.details(trial)["metadata"]["start"]["calibration"]
        assert frozen == [dict(document=active[0][0], manifest=dict(revision="original"))]
    finally:
        manager.close()


def test_device_transition_blocks_calibration_admission(tmp_path):
    manager = service(tmp_path)
    try:
        manager.processes = SimpleNamespace(busy=True)
        assert not manager.calibration_allowed()[0]
        manager.processes.busy = False
        assert manager.calibration_allowed()[0]
    finally:
        manager.processes = None
        manager.close()
