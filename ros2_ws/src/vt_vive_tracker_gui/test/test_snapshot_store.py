from dataclasses import FrozenInstanceError

import pytest

from vt_vive_tracker.visualization_model import (
    FIXED_ROLES,
    RoleSnapshot,
    VisualHealth,
)
from vt_vive_tracker_gui.snapshot_store import LatestSnapshotStore


def snapshots(rate):
    return tuple(
        RoleSnapshot(role, VisualHealth.OFFLINE, None, (), rate, None)
        for role in FIXED_ROLES
    )


def test_store_starts_empty_and_only_exposes_latest_version():
    store = LatestSnapshotStore()
    assert store.latest() is None

    first = store.publish(snapshots(1.0))
    second = store.publish(snapshots(2.0))

    assert first.version == 1
    assert second.version == 2
    assert store.latest() == second
    assert store.latest().roles[0].receive_rate_hz == 2.0


def test_stored_snapshot_is_immutable():
    value = LatestSnapshotStore().publish(snapshots(1.0))
    with pytest.raises(FrozenInstanceError):
        value.version = 9


def test_store_rejects_wrong_role_order():
    value = tuple(reversed(snapshots(1.0)))
    with pytest.raises(ValueError, match="fixed role order"):
        LatestSnapshotStore().publish(value)


def test_diagnostic_is_a_separate_overwriting_single_slot():
    store = LatestSnapshotStore()
    first = store.publish_diagnostic("waiting", 10)
    second = store.publish_diagnostic("bad frame", 20)
    assert first.version == 1
    assert second.version == 2
    assert store.latest_diagnostic() == second
    assert store.latest() is None
