"""Managed UI preserves scoped tactile settings and uses only cached USB identity."""
import json
import threading
import time
from types import SimpleNamespace

import pytest
from PySide6 import QtWidgets

from vt_capture_console.ui.hand import managed_panel as module
from vt_capture_console.backend.tactile_reference_store import (
    TactileDeviceIdentity, TactileDefaultProfile, TactileReferenceStore,
)
from vt_tactile_capture.display_response import ReferenceProfile, ResponseSettings


class Panel(QtWidgets.QWidget):
    def __init__(self, settings, resources, session, tactile, **kwargs):
        super().__init__()
        self.session, self.tactile = session, tactile
        self.title, self.language_combo = QtWidgets.QLabel(), QtWidgets.QComboBox()
        self.calibration_toggle = QtWidgets.QCheckBox()
        assert kwargs["owns_session"] is False
    def start(self): pass
    def shutdown(self):
        self.session.close()
        self.tactile.close()


@pytest.fixture
def pane(qapp, tmp_path, monkeypatch):
    monkeypatch.setattr(module, "_Settings", lambda hand: SimpleNamespace(data_root=tmp_path, hand=hand))
    monkeypatch.setattr(module, "RemoteHandSession", lambda *a: SimpleNamespace(close=lambda: None))
    value = module.ManagedHandPane("left", object(), panel_factory=Panel)
    value.timer.stop()
    value._metadata = dict(control_socket="/never-opened.sock")
    yield value
    value.shutdown()
    value.deleteLater()
    qapp.processEvents()


def reference():
    return ReferenceProfile((1000.,) * 400, (3.,) * 400, (True,) * 400)


@pytest.mark.parametrize("scope", ["saved", "default", "none"])
def test_scoped_reference_settings_survive_panel_open(pane, tmp_path, scope):
    identity = TactileDeviceIdentity(123, 456, "left-device")
    store = TactileReferenceStore(tmp_path / "tactile-references", hand="left")
    resources = dict(tactile_device_identity=identity)
    settings = ResponseSettings(display_range_adc=1700., direction="decrease")
    if scope == "saved":
        store.save(store.capture(identity, reference(), settings))
    elif scope == "default":
        preset = tmp_path / "preset.json"
        profile = TactileDefaultProfile(reference(), ResponseSettings(900.), "2026-11-08T00:00:00+00:00", "left")
        preset.write_text(json.dumps(profile.to_dict()))
        resources["response_profile"] = preset
        store.save_default_settings(settings)
    before = {p: p.read_bytes() for p in tmp_path.rglob("*.json")}
    pane._build(resources)
    assert pane.tactile.response_settings == (ResponseSettings(3000.) if scope == "none" else settings)
    assert {p: p.read_bytes() for p in tmp_path.rglob("*.json")} == before


def test_measured_reference_is_saved_for_the_configured_usb_identity(pane, tmp_path):
    from test_tactile_managed import sample, NOW
    identity = TactileDeviceIdentity(123, 456, "left-device")
    pane._build(dict(tactile_device_identity=identity))
    source = pane.tactile
    source.observe(sample(hand="left"), now_ns=NOW)
    assert source.bind_device_identity(source.identity_request, identity, now_ns=NOW)
    source.start_reference(now_ns=NOW)
    for index in range(1, 31):
        now = NOW + index * 100_000_000
        source.observe(sample(index + 1, now, "left"), now_ns=now)
        source.poll(now_ns=now)
    assert source.response_status.phase == "ready"
    assert source.response_status.reference_source == "measured"
    assert source.reference_persistence_error == ""
    profile, scope = TactileReferenceStore(tmp_path / "tactile-references", hand="left").load(identity)
    assert scope == "saved" and profile.device == identity
    assert profile.settings == ResponseSettings(3000.)


def test_changing_configured_tactile_port_reloads_device_scoped_reference(pane):
    calls = []
    pane._loader.close()
    pane._loader = SimpleNamespace(request=lambda root, **kwargs: calls.append((root, kwargs)) or len(calls),
                                   close=lambda: None)
    metadata = dict(resources="/bundle", control_socket="/hand.sock", glove_python="/python",
                    tactile_socket="/tactile.sock", tactile_port="/first")
    pane.update_manager(dict(hand_sessions={"left": metadata}))
    pane.update_manager(dict(hand_sessions={"left": dict(metadata)}))
    metadata = dict(metadata, tactile_port="/second")
    pane.update_manager(dict(hand_sessions={"left": metadata}))
    assert calls == [("/bundle", dict(interpreter="/python", tactile_port="/first")),
                     ("/bundle", dict(interpreter="/python", tactile_port="/second"))]


@pytest.mark.parametrize("resolution_phase", ["before", "during", "after"])
def test_page_open_before_usb_can_save_reference_without_rebuilding_panel(
        pane, tmp_path, monkeypatch, resolution_phase):
    from test_tactile_managed import sample, NOW
    from vt_capture_console.backend.hand_session import TactileIdentityResolver
    from vt_tactile_capture import host_clock
    identity = TactileDeviceIdentity(123, 456, "appeared-later")
    present, calls, now = [None], [], [NOW]
    def resolve(port):
        calls.append((port, threading.get_ident()))
        return present[0]
    monkeypatch.setattr(module, "TactileIdentityResolver", lambda: TactileIdentityResolver(resolve))
    monkeypatch.setattr(host_clock, "monotonic_ns", lambda: now[0])
    pane._metadata["tactile_port"] = "/same-path"
    pane._build(dict(tactile_device_identity=None))
    panel, source, imu = pane.panel, pane.tactile, pane.session
    pane._poll()
    assert calls == []  # No sampling or port probing while disconnected.

    present[0] = identity
    source.observe(sample(hand="left"), now_ns=now[0])
    def resolve_current_session():
        deadline = time.monotonic() + 2
        while source.identity_request is not None and time.monotonic() < deadline:
            pane._poll()
            time.sleep(.005)
        assert source.identity_request is None
    if resolution_phase == "before":
        resolve_current_session()
    source.start_reference(now_ns=now[0])
    for index in range(1, 31):
        now[0] = NOW + index * 100_000_000
        source.observe(sample(index + 1, now[0], "left"), now_ns=now[0])
        source.poll(now_ns=now[0])
        if resolution_phase == "during" and index == 5:
            progress = source.response_status.progress
            resolve_current_session()
            assert source.response_status.phase == "collecting"
            assert source.response_status.progress == progress
    if resolution_phase == "after":
        assert source.reference_persistence_error
        resolve_current_session()
    saved, scope = TactileReferenceStore(tmp_path / "tactile-references", hand="left").load(identity)
    assert saved.device == identity and scope == "saved"
    assert not source.reference_persistence_error
    assert pane.panel is panel and pane.session is imu and pane.tactile is source
    assert len(calls) == 1 and calls[0][0] == "/same-path"
    assert calls[0][1] != threading.get_ident()


@pytest.mark.parametrize("change", ["epoch", "reconnect"])
def test_new_device_on_same_port_never_uses_previous_device_reference(pane, tmp_path, change):
    from test_tactile_managed import sample, NOW
    first = TactileDeviceIdentity(123, 456, "first")
    second = TactileDeviceIdentity(123, 456, "second")
    store = TactileReferenceStore(tmp_path / "tactile-references", hand="left")
    store.save(store.capture(first, reference(), ResponseSettings(1700.)))
    second_reference = ReferenceProfile((2000.,) * 400, (3.,) * 400, (True,) * 400)
    store.save(store.capture(second, second_reference, ResponseSettings(2200.)))
    pane._build(dict(tactile_device_identity=first))
    source, panel = pane.tactile, pane.panel
    source.observe(sample(hand="left"), now_ns=NOW)
    old_request = source.identity_request
    assert source.bind_device_identity(old_request, first, now_ns=NOW)
    assert source.response_settings == ResponseSettings(1700.)
    assert source.response_status.phase == "ready"

    now = NOW + 100_000_000
    if change == "reconnect":
        source.observe(sample(2, now, "left", connected=False), now_ns=now)
    source.observe(sample(1, now, "left", value=2000, stream_epoch=2 if change == "epoch" else 1), now_ns=now)
    assert source.response_status.phase == "unreferenced"
    assert source.response_settings == ResponseSettings(3000.)
    assert not source.bind_device_identity(old_request, first, now_ns=now)
    assert source.bind_device_identity(source.identity_request, second, now_ns=now)
    assert source.response_settings == ResponseSettings(2200.)
    assert source.response_status.phase == "ready"
    assert source.response_status.active_cells == 0
    assert source._model.export_reference().baseline_adc == (2000.,) * 400
    assert pane.panel is panel
