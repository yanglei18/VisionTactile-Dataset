"""Every mutation is source-bound before device/recording handlers run."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from vt_capture_core import ProtocolError, decode_message, encode_message
from vt_capture_core.recipes import Recipe
from vt_capture_core.software_identity import empty_identity
from vt_capture_manager.config import ManagerConfig
from vt_capture_manager.management import ManagementService
from vt_capture_manager.model import SnapshotModel


def software(**changes):
    return dict(source_root="/candidate", source_revision="a" * 40,
                source_sha256="b" * 64, configuration_path="/candidate/settings.json") | changes


@pytest.fixture
def manager():
    model = SnapshotModel(ManagerConfig((), quality_enabled=False), disk_usage=lambda _: SimpleNamespace(free=100))
    service = ManagementService(model, software=software())
    yield service
    service.close()


def mutation(name):
    if name.startswith("device."):
        params = dict(group_id="hand", expected_manager_instance_id="manager")
    elif name == "hand.command":
        params = dict(hand="left", expected_manager_instance_id="manager", expected_service_instance_id="service",
                      expected_session_id=None, expected_profile_version=None, name="capture", args={"step": "neutral"})
    elif name == "calibration.cancel":
        params = dict(expected_manager_instance_id="manager", session_id="calibration")
    elif name == "batch.create":
        params = dict(label="sample")
    elif name == "capture.start":
        params = dict(batch_id="batch", label="sample", duration_s=30, expected_controller_instance_id="controller",
            expected_state_revision=1, recipe=Recipe("sample", duration_s=30).to_dict(),
            expected_topics_fingerprint="c" * 64)
    elif name == "capture.stop":
        params = dict(trial_id="trial", expected_controller_instance_id="controller", expected_session_id="session")
    else:
        params = dict(trial_id="trial")
    return dict(version=8, request_id="request", command=name, params=params, expected_software=software())


@pytest.mark.parametrize("name", ["device.start", "device.stop", "device.restart", "hand.command",
                                    "calibration.cancel", "batch.create", "capture.start", "capture.stop", "quality.retry"])
def test_wrong_source_rejected_before_any_command_handler_or_desktop_admission(manager, monkeypatch, name):
    request = mutation(name)
    request["expected_software"]["source_sha256"] = "c" * 64
    monkeypatch.setattr(manager, "_dispatch", lambda _: pytest.fail("command handler must not run"))
    manager._desktop_admission = SimpleNamespace(acquire=lambda: pytest.fail("must reject before admission"), close=lambda: None)
    reply = manager.dispatch(request)
    assert reply["ok"] is False
    assert reply["operation"]["code"] == "software_identity_mismatch"
    assert reply["operation"]["state"] == "rejected"
    assert reply["snapshot"]["software"] == software()
    assert decode_message(encode_message(reply)) == reply


@pytest.mark.parametrize("field,value", [
    ("source_root", "/other"), ("source_revision", "c" * 40),
    ("source_sha256", "c" * 64), ("configuration_path", "/other/settings.json"),
])
def test_all_four_binding_fields_are_enforced(manager, monkeypatch, field, value):
    request = mutation("batch.create")
    request["expected_software"][field] = value
    monkeypatch.setattr(manager, "_dispatch", lambda _: pytest.fail("stale candidate entered mutation handler"))
    assert manager.dispatch(request)["operation"]["code"] == "software_identity_mismatch"


def test_matching_source_reaches_normal_admission_only_once(manager, monkeypatch):
    calls = []
    sentinel = object()
    monkeypatch.setattr(manager, "_dispatch", lambda request: calls.append(deepcopy(request)) or sentinel)
    request = mutation("batch.create")
    assert manager.dispatch(request) is sentinel
    assert calls == [request]


def test_manager_copies_boot_identity_and_snapshot_cannot_relabel_it(manager):
    supplied = software()
    model = SnapshotModel(ManagerConfig((), quality_enabled=False), disk_usage=lambda _: SimpleNamespace(free=100))
    service = ManagementService(model, software=supplied)
    try:
        supplied["source_sha256"] = "d" * 64
        exposed = service.snapshot(8)
        exposed["software"]["source_root"] = "/false-claim"
        assert service.snapshot(8)["software"] == software()
    finally:
        service.close()


@pytest.mark.parametrize("version", range(1, 9))
def test_snapshot_wire_changes_only_at_v8(manager, version):
    snapshot = manager.snapshot(version)
    assert ("software" in snapshot) == (version == 8)
    reply = dict(version=version, request_id="read", ok=True, snapshot=snapshot)
    if version >= 2:
        reply["operation"] = None
    assert decode_message(encode_message(reply)) == reply


@pytest.mark.parametrize("version", range(2, 8))
def test_legacy_mutations_keep_previous_admission_contract(manager, monkeypatch, version):
    request = mutation("batch.create")
    request["version"] = version
    request.pop("expected_software")
    calls = []
    monkeypatch.setattr(manager, "_dispatch", lambda value: calls.append(value) or "legacy")
    assert manager.dispatch(request) == "legacy"
    assert calls == [request]


def test_unknown_source_cannot_write_even_when_both_sides_are_unknown(manager, monkeypatch):
    manager.software = empty_identity()
    request = mutation("batch.create")
    request["expected_software"] = empty_identity()
    monkeypatch.setattr(manager, "_dispatch", lambda _: pytest.fail("unknown source may not mutate"))
    with pytest.raises(ProtocolError, match="read-only"):
        manager.dispatch(request)
    readonly = manager.snapshot(8)
    assert readonly["software"] == empty_identity()
