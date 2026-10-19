import fcntl
import os
from types import SimpleNamespace

from vt_capture_core import encode_message
from vt_capture_manager.config import ManagerConfig
from vt_capture_manager.management import ManagementService
from vt_capture_manager.model import SnapshotModel


def test_desktop_apply_excludes_mutation_but_keeps_snapshot_available(tmp_path):
    path = tmp_path / "private" / ".desktop-apply.lock"
    model = SnapshotModel(ManagerConfig((), quality_enabled=False), disk_usage=lambda _: SimpleNamespace(free=100))
    manager = ManagementService(model, enabled=True, state_dir=tmp_path / "state", admission_lock=path)
    fd = os.open(path, os.O_RDWR)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        snapshot = manager.dispatch(dict(version=4, request_id="read", command="snapshot", params={}))
        assert snapshot["ok"]
        encode_message(snapshot)
        blocked = manager.dispatch(dict(version=4, request_id="new", command="batch.create", params=dict(label="test")))
        assert blocked["operation"]["code"] == "desktop_configuration_applying"
        assert not blocked["ok"]
        assert manager.store.operation("new") is None
        fcntl.flock(fd, fcntl.LOCK_UN)
        accepted = manager.dispatch(dict(version=4, request_id="new", command="batch.create", params=dict(label="test")))
        assert accepted["ok"]
    finally:
        os.close(fd)
        manager.close()
