"""Stored-reference behavior through actual source/assembler/response APIs."""
import pytest

from vt_capture_console.backend.tactile_reference_store import TactileReferenceStore
from vt_capture_console.backend.tactile_serial import TactileSerialSource
from vt_tactile_capture.display_response import ResponseSettings
from vt_tactile_capture.protocol import Block
from vt_tactile_capture.serial_source import Batch

from test_tactile_reference_store import DEVICE, OTHER, default_profile, profile, write_profile
from test_tactile_serial import Worker, NOW


STEP = 100_000_000


def deliver(source, sequence, stamp, *, baseline=1000, pressed=None, delta=100,
            session="reference-test", epoch=1):
    blocks = tuple(Block(row, group, 0, tuple(
        baseline + (delta if group + 8 * index < 20
                    and row * 20 + group + 8 * index == pressed else 0)
        for index in range(10))) for row in range(20) for group in range(8))
    source._worker.queue.append(Batch(
        hand="right", session_id=session, stream_epoch=epoch, sequence=sequence,
        first_packet_index=sequence * len(blocks), host_monotonic_ns=stamp,
        host_realtime_ns=stamp + 100_000_000_000, connected=True, time_valid=True,
        reason="ok", blocks=blocks,
    ))
    return source.poll(stamp)


def connect(store, *, device=DEVICE, port="COM3", identities=None):
    provider = identities or (lambda _: device)
    source = TactileSerialSource(worker_factory=Worker, reference_store=store,
                                 identity_provider=provider)
    source.connect(port)
    return source


def calibrate(source, *, baseline=1500, now=NOW, sequence_start=0):
    deliver(source, sequence_start, now, baseline=baseline)
    source.start_reference(now)
    for step in range(1, 31):
        view = deliver(source, sequence_start + step, now + step * STEP, baseline=baseline)
    assert view.response.phase == "ready"
    assert view.response.reference_source == "measured"
    return now + 30 * STEP


@pytest.fixture
def preset_store(tmp_path):
    preset = tmp_path / "preset.json"
    write_profile(preset, profile())
    return TactileReferenceStore(tmp_path / "user", preset_path=preset)


@pytest.fixture
def default_store(tmp_path):
    preset = tmp_path / "default.json"
    write_profile(preset, default_profile())
    return TactileReferenceStore(tmp_path / "user", preset_path=preset)


def test_migration_to_a_different_com_port_uses_same_usb_preset(preset_store):
    for port in ("COM3", "COM19"):
        source = connect(preset_store, port=port)
        try:
            view = deliver(source, 0, NOW)
            assert view.response.reference_source == "preset"
            assert view.response.reference_cells == view.valid_cells == 400
            assert view.frame["reference_ready"] is True
            assert view.frame["values"] == [0.] * 400
        finally:
            assert source.close()


def test_connecting_while_pressed_preserves_response_without_automatic_zero(preset_store):
    source = connect(preset_store)
    try:
        first = deliver(source, 0, NOW, pressed=21)
        assert first.response.reference_source == "preset"
        assert first.frame["values"][21] > 0
        assert sum(v > 0 for v in first.frame["values"]) == 1
        for sequence in range(1, 61):
            view = deliver(source, sequence, NOW + sequence * STEP, pressed=21)
            assert view.frame["values"] == first.frame["values"]
        released = deliver(source, 61, NOW + 61 * STEP)
        assert released.frame["values"] == [0.] * 400
        assert not preset_store.directory.exists(), "Connection must not acquire/save a new reference"
    finally:
        assert source.close()


def test_different_device_uses_manual_reference_instead_of_the_default(preset_store):
    source = connect(preset_store, device=OTHER)
    try:
        view = deliver(source, 0, NOW, pressed=21)
        assert view.response.phase == "unreferenced"
        assert not view.frame["reference_ready"]
        now = calibrate(source, baseline=1600, now=NOW + STEP, sequence_start=1)
        assert preset_store.load(OTHER)[0].reference.baseline_adc[0] == 1600
        assert preset_store.load(DEVICE)[1] == "preset"
        assert deliver(source, 32, now + STEP, baseline=1600, pressed=21).frame["values"][21] > 0
    finally:
        assert source.close()


def test_corrupt_reference_is_diagnostic_and_manual_zero_still_works(preset_store):
    preset_store.preset_path.write_text("{invalid")
    source = connect(preset_store)
    try:
        view = deliver(source, 0, NOW)
        assert view.response.phase == "unreferenced"
        assert "preset reference" in source.diagnostic_log
        calibrate(source, now=NOW + STEP, sequence_start=1)
        assert preset_store.load(DEVICE)[1] == "saved"
        assert source.reference_persistence_error == ""
    finally:
        assert source.close()


def test_manual_zero_and_settings_survive_restart_and_override_preset(preset_store):
    source = connect(preset_store)
    try:
        now = calibrate(source, baseline=1500)
        source.configure_response(display_range_adc=400, direction="increase")
        expected = deliver(source, 31, now + STEP, baseline=1500, pressed=21)
        assert source.reference_persistence_error == ""
    finally:
        assert source.close()
    # Construct a fresh store as well as a fresh source; reuse no in-memory state.
    store = TactileReferenceStore(preset_store.directory, preset_path=preset_store.preset_path)
    restarted = connect(store, port="COM23")
    try:
        actual = deliver(restarted, 0, now + 2 * STEP, baseline=1500, pressed=21,
                         session="restarted")
        assert actual.response.reference_source == "saved"
        assert restarted.response_settings == ResponseSettings(400., "increase")
        assert actual.frame == expected.frame
        assert restarted.disconnect()
        restarted.connect("COM27")
        reconnected = deliver(restarted, 0, now + 3 * STEP, baseline=1500,
                              pressed=21, session="reconnected")
        assert reconnected.frame == expected.frame
        assert reconnected.response.reference_source == "saved"
    finally:
        assert restarted.close()


def test_reference_write_failure_keeps_live_reference_and_reports_once(preset_store, monkeypatch):
    calls = []

    def fail(profile):
        calls.append(profile)
        raise OSError("simulated disk full")

    monkeypatch.setattr(preset_store, "save", fail)
    source = connect(preset_store)
    try:
        now = calibrate(source, baseline=1500)
        assert source.reference_persistence_error
        assert "disk full" in source.diagnostic_log
        assert len(calls) == 1
        for sequence in range(31, 35):
            view = deliver(source, sequence, NOW + sequence * STEP, baseline=1500, pressed=21)
            assert view.response.reference_source == "measured"
            assert view.frame["values"][21] > 0
            assert sum(v > 0 for v in view.frame["values"]) == 1
        assert len(calls) == 1, "Polling must not repeatedly write a failing filesystem"
        assert preset_store.load(DEVICE)[1] == "preset"
    finally:
        assert source.close()


def test_settings_save_after_disk_recovers_uses_latest_measured_reference(preset_store, monkeypatch):
    save = preset_store.save

    def fail(_):
        raise OSError("temporarily unavailable")

    monkeypatch.setattr(preset_store, "save", fail)
    source = connect(preset_store)
    try:
        calibrate(source, baseline=1500)
        assert source.reference_persistence_error
        monkeypatch.setattr(preset_store, "save", save)
        source.configure_response(display_range_adc=600)
        saved, origin = preset_store.load(DEVICE)
        assert origin == "saved"
        assert saved.reference.baseline_adc == (1500.,) * 400
        assert saved.settings.display_range_adc == 600
        assert source.reference_persistence_error == ""
    finally:
        assert source.close()


@pytest.mark.parametrize("save_fails", [False, True])
def test_identified_device_stream_reset_reuses_latest_reference_without_old_contact(
        preset_store, monkeypatch, save_fails):
    if save_fails:
        def fail(_):
            raise OSError("simulated disk full")
        monkeypatch.setattr(preset_store, "save", fail)
    source = connect(preset_store)
    try:
        now = calibrate(source, baseline=1500)
        assert deliver(source, 31, now + STEP, baseline=1500, pressed=21).response.active_cells == 1
        # A new transport epoch does not mean a different USB device; however,
        # hysteresis from the old epoch must never resurrect a contact.
        view = deliver(source, 32, now + 2 * STEP, baseline=1500, pressed=21,
                       delta=2, epoch=2)
        assert view.response.reference_source == ("measured" if save_fails else "saved")
        assert view.frame["reference_ready"]
        assert view.frame["values"] == [0.] * 400
    finally:
        assert source.close()


def test_unknown_usb_identity_keeps_manual_reference_session_only(preset_store):
    source = connect(preset_store, device=None)
    try:
        now = calibrate(source)
        assert "唯一标识" in source.reference_persistence_error
        assert not preset_store.directory.exists()
        assert deliver(source, 31, now + STEP, baseline=1500, pressed=21).frame["values"][21] > 0
        assert source.disconnect()
        source.connect("COM19")
        view = deliver(source, 0, now + 2 * STEP, baseline=1500, session="new")
        assert view.response.phase == "unreferenced"
    finally:
        assert source.close()


def test_device_identity_lookup_failure_does_not_prevent_serial_use(preset_store):
    def unavailable(_):
        raise OSError("USB enumeration unavailable")

    source = connect(preset_store, identities=unavailable)
    try:
        view = deliver(source, 0, NOW)
        assert view.valid_cells == 400
        assert view.response.phase == "unreferenced"
        assert "USB enumeration unavailable" in source.diagnostic_log
    finally:
        assert source.close()


@pytest.mark.parametrize("device", [DEVICE, OTHER, None])
def test_new_board_or_unknown_usb_uses_default_and_preserves_an_existing_press(default_store, device):
    source = connect(default_store, device=device)
    try:
        assert source.poll(NOW).frame is None, "A reference cannot fabricate live data"
        first = deliver(source, 0, NOW, pressed=21)
        assert first.valid_cells == first.response.reference_cells == 400
        assert first.response.reference_source == "default"
        assert first.frame["values"][21] > 0
        assert sum(value > 0 for value in first.frame["values"]) == 1
        for sequence in range(1, 61):
            current = deliver(source, sequence, NOW + sequence * STEP, pressed=21)
            assert current.frame["values"] == first.frame["values"]
        released = deliver(source, 61, NOW + 61 * STEP)
        assert released.frame["values"] == [0.] * 400
        assert not default_store.directory.exists(), "Connecting must not acquire or save a reference"
        assert source.poll(NOW + 62 * STEP + 1).frame is None, "Default reference cannot keep stale data live"
    finally:
        assert source.close()


def test_usb_lookup_failure_still_loads_explicit_default(default_store):
    def unavailable(_):
        raise OSError("USB enumeration unavailable")

    source = connect(default_store, identities=unavailable)
    try:
        view = deliver(source, 0, NOW, pressed=21)
        assert view.response.reference_source == "default"
        assert view.frame["values"][21] > 0
        assert "USB enumeration unavailable" in source.diagnostic_log
    finally:
        assert source.close()


def test_shared_display_settings_survive_restart_without_calibrating_a_board(default_store):
    source = connect(default_store, device=OTHER)
    try:
        deliver(source, 0, NOW, pressed=21)
        source.configure_response(display_range_adc=500., direction="increase")
        expected = deliver(source, 1, NOW + STEP, pressed=21)
        assert expected.response.reference_source == "default"
        assert sorted(p.name for p in default_store.directory.iterdir()) == ["default-settings.json"]
        assert source.reference_persistence_error == ""
    finally:
        assert source.close()
    store = TactileReferenceStore(default_store.directory, preset_path=default_store.preset_path)
    for identity in (DEVICE, OTHER, None):
        restarted = connect(store, device=identity, port="COM27")
        try:
            actual = deliver(restarted, 0, NOW + 2 * STEP, pressed=21)
            assert actual.response.reference_source == "default"
            assert actual.frame == expected.frame
            assert restarted.response_settings == ResponseSettings(500., "increase")
        finally:
            assert restarted.close()


def test_optional_zero_on_new_board_saves_only_that_board_and_overrides_default(default_store):
    # The other board already has an independent measured reference.
    previous = profile(baseline=1700., settings=ResponseSettings(700., "both"))
    default_store.save(previous)
    preset_bytes = default_store.preset_path.read_bytes()
    source = connect(default_store, device=OTHER)
    try:
        source.configure_response(display_range_adc=500.)
        now = calibrate(source, baseline=1600)
        source.configure_response(display_range_adc=300., direction="decrease")
        expected = deliver(source, 31, now + STEP, baseline=1600, pressed=21, delta=-100)
        assert expected.frame["values"][21] > 0
        assert default_store.load(DEVICE) == (previous, "saved")
        assert default_store.preset_path.read_bytes() == preset_bytes
    finally:
        assert source.close()
    restarted = connect(default_store, device=OTHER, port="COM19")
    try:
        actual = deliver(restarted, 0, now + 2 * STEP, baseline=1600, pressed=21, delta=-100)
        assert actual.response.reference_source == "saved" and actual.frame == expected.frame
        assert restarted.response_settings == ResponseSettings(300., "decrease")
        assert default_store.load(None)[1] == "default"
        assert default_store.load(None)[0].settings.display_range_adc == 500.
    finally:
        assert restarted.close()


def test_default_settings_write_failure_keeps_live_response_and_origin(default_store, monkeypatch):
    def fail(_):
        raise OSError("simulated disk full")

    monkeypatch.setattr(default_store, "save_default_settings", fail)
    source = connect(default_store)
    try:
        deliver(source, 0, NOW, pressed=21)
        source.configure_response(display_range_adc=500.)
        view = deliver(source, 1, NOW + STEP, pressed=21)
        assert view.response.reference_source == "default"
        assert view.frame["levels"] == [0., 500.]
        assert view.frame["values"][21] > 0
        assert source.reference_persistence_error and "disk full" in source.diagnostic_log
        assert not default_store.directory.exists()
    finally:
        assert source.close()


def test_unknown_usb_manual_reference_is_not_replaced_by_default_when_settings_change(default_store):
    source = connect(default_store, device=None)
    try:
        now = calibrate(source, baseline=1600)
        source.configure_response(display_range_adc=500.)
        current = deliver(source, 31, now + STEP, baseline=1600, pressed=21)
        assert current.response.reference_source == "measured"
        assert sum(value > 0 for value in current.frame["values"]) == 1
        assert "唯一标识" in source.reference_persistence_error
        assert not default_store.directory.exists()
        assert source.disconnect()
        source.connect("COM19")
        reconnected = deliver(source, 0, now + 2 * STEP, baseline=1000, session="new")
        assert reconnected.response.reference_source == "default"
        assert reconnected.frame["values"] == [0.] * 400
    finally:
        assert source.close()
