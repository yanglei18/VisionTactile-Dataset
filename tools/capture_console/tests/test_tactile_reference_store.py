"""Device-bound reference documents and atomic local persistence, without ports."""
import json
from types import SimpleNamespace

import pytest

from vt_capture_console.backend import tactile_reference_store as module
from vt_capture_console.backend.tactile_reference_store import (
    TactileDefaultProfile, TactileDeviceIdentity, TactileDeviceProfile, TactileReferenceStore,
    read_device_profile, read_reference_profile,
)
from vt_tactile_capture.display_response import ReferenceProfile, ResponseSettings


DEVICE = TactileDeviceIdentity(0x0483, 0x5740, "tactile-test-one")
OTHER = TactileDeviceIdentity(0x0483, 0x5740, "tactile-test-two")


def profile(*, device=DEVICE, baseline=1000., settings=None):
    return TactileDeviceProfile(
        device, ReferenceProfile([baseline] * 400, [3.] * 400, [True] * 400),
        settings or ResponseSettings(), "2026-10-27T04:00:00+00:00",
    )


def write_profile(path, value):
    path.write_text(json.dumps(value.to_dict(), ensure_ascii=False, allow_nan=False))


def default_profile(*, baseline=1000., settings=None):
    measured = profile(baseline=baseline, settings=settings)
    return TactileDefaultProfile(measured.reference, measured.settings, measured.captured_at_utc)


def test_document_round_trip_contains_no_com_port_or_local_path(tmp_path):
    original = profile(settings=ResponseSettings(750., "decrease"))
    path = tmp_path / "device-reference.json"
    write_profile(path, original)
    assert read_device_profile(path) == original
    document = json.loads(path.read_text())
    assert document["protocol"] == "vt_tactile_adc400_v1"
    assert document["hand"] == "right"
    assert document["device"] == DEVICE.to_dict()
    assert "port" not in document and "path" not in document


@pytest.mark.parametrize("changes", [
    {"vendor_id": True}, {"vendor_id": -1}, {"product_id": 65536},
    {"serial_number": None}, {"serial_number": " "},
    {"serial_number": "line\nbreak"}, {"serial_number": "x" * 129},
])
def test_identity_requires_a_real_stable_usb_identifier(changes):
    with pytest.raises(ValueError):
        TactileDeviceIdentity(**(DEVICE.to_dict() | changes))


@pytest.mark.parametrize("field,value", [
    ("schema_version", True), ("schema_version", 2),
    ("protocol", "unknown"), ("hand", "left"),
    ("device", {}), ("device", None),
    ("settings", {"display_range_adc": 0, "direction": "both"}),
    ("settings", {"display_range_adc": 1000, "direction": "guess"}),
    ("reference", {}), ("captured_at_utc", "2026-10-27T04:00:00"),
])
def test_invalid_or_incompatible_documents_are_not_accepted(field, value):
    document = profile().to_dict()
    document[field] = value
    with pytest.raises(ValueError):
        TactileDeviceProfile.from_dict(document)


def test_oversized_profile_is_rejected_before_parsing(tmp_path):
    path = tmp_path / "large.json"
    path.write_bytes(b" " * (module.MAX_PROFILE_BYTES + 1))
    with pytest.raises(ValueError, match="too large"):
        read_device_profile(path)


def test_saved_profile_takes_priority_over_bundled_preset(tmp_path):
    preset = tmp_path / "preset.json"
    write_profile(preset, profile())
    store = TactileReferenceStore(tmp_path / "user", preset_path=preset)
    saved = profile(baseline=1400., settings=ResponseSettings(500., "increase"))
    store.save(saved)
    loaded, source = store.load(DEVICE)
    assert loaded == saved and source == "saved"
    assert read_device_profile(preset).reference.baseline_adc[0] == 1000.
    assert not list(store.directory.glob(".reference-*.tmp"))


def test_wrong_device_is_not_loaded_even_under_the_expected_filename(tmp_path):
    preset = tmp_path / "preset.json"
    write_profile(preset, profile(device=OTHER))
    store = TactileReferenceStore(tmp_path / "user", preset_path=preset)
    store.directory.mkdir()
    write_profile(store.directory / (DEVICE.key + ".json"), profile(device=OTHER))
    assert store.load(DEVICE) is None
    assert store.load(None) is None


@pytest.mark.parametrize("content", [b"{broken", b"\xff\xfe", b"{}"])
def test_corrupt_saved_reference_falls_back_to_valid_matching_preset(tmp_path, content):
    preset = tmp_path / "preset.json"
    fallback = profile()
    write_profile(preset, fallback)
    store = TactileReferenceStore(tmp_path / "user", preset_path=preset)
    store.directory.mkdir()
    (store.directory / (DEVICE.key + ".json")).write_bytes(content)
    assert store.load(DEVICE) == (fallback, "preset")
    assert len(store.diagnostics) == 1 and "saved reference" in store.diagnostics[0]


def test_failed_atomic_replace_preserves_previous_reference(tmp_path, monkeypatch):
    store = TactileReferenceStore(tmp_path / "user")
    previous = profile()
    store.save(previous)
    path = store.directory / (DEVICE.key + ".json")
    before = path.read_bytes()

    def fail_replace(*_):
        raise OSError("simulated disk failure")

    monkeypatch.setattr(module.os, "replace", fail_replace)
    with pytest.raises(OSError, match="disk failure"):
        store.save(profile(baseline=1600.))
    assert path.read_bytes() == before
    assert store.load(DEVICE) == (previous, "saved")
    assert not list(store.directory.glob(".reference-*.tmp"))


def test_usb_enumeration_identity_is_independent_of_com_number(monkeypatch):
    import serial
    from serial.tools import list_ports

    def do_not_open(*_args, **_kwargs):
        raise AssertionError("Identity lookup must not open or probe the port")

    monkeypatch.setattr(serial, "Serial", do_not_open)
    for selected in ("COM3", "COM19"):
        monkeypatch.setattr(list_ports, "comports", lambda: [
            SimpleNamespace(device=selected, vid=DEVICE.vendor_id, pid=DEVICE.product_id,
                            serial_number=DEVICE.serial_number),
        ])
        assert module.device_identity_for_port(selected) == DEVICE
        assert module.device_identity_for_port("COM_UNKNOWN") is None


def test_device_without_usb_serial_has_no_port_based_identity_fallback(monkeypatch):
    from serial.tools import list_ports

    monkeypatch.setattr(list_ports, "comports", lambda: [
        SimpleNamespace(device="COM3", vid=DEVICE.vendor_id, pid=DEVICE.product_id,
                        serial_number=None),
    ])
    assert module.device_identity_for_port("COM3") is None


def test_explicit_layout_default_has_no_device_and_cannot_be_read_as_a_measurement(tmp_path):
    path = tmp_path / "default.json"
    expected = default_profile()
    write_profile(path, expected)
    assert read_reference_profile(path) == expected
    document = json.loads(path.read_text())
    assert "device" not in document
    assert document["scope"] == "layout" and document["layout"] == "right_adc20x20_v1"
    with pytest.raises(ValueError):
        read_device_profile(path)


@pytest.mark.parametrize("field,value", [
    ("schema_version", True), ("schema_version", 2.), ("scope", "device"),
    ("protocol", "other_protocol"), ("hand", "left"), ("layout", "other_layout"),
    ("device", DEVICE.to_dict()), ("settings", {}), ("settings", None),
    ("reference", {}), ("captured_at_utc", "2026-10-27T04:00:00"),
])
def test_layout_default_rejects_ambiguous_or_incompatible_scope(field, value):
    document = default_profile().to_dict()
    document[field] = value
    with pytest.raises(ValueError):
        TactileDefaultProfile.from_dict(document)


def test_device_measurements_take_priority_over_cross_board_default(tmp_path):
    preset = tmp_path / "preset.json"
    shared = default_profile()
    write_profile(preset, shared)
    store = TactileReferenceStore(tmp_path / "user", preset_path=preset)
    for identity in (DEVICE, OTHER, None):
        assert store.load(identity) == (shared, "default")
    measured = profile(baseline=1500.)
    store.save(measured)
    assert store.load(DEVICE) == (measured, "saved")
    assert store.load(OTHER) == store.load(None) == (shared, "default")
    assert read_reference_profile(preset) == shared


def test_layout_document_in_device_path_does_not_become_a_saved_measurement(tmp_path):
    store = TactileReferenceStore(tmp_path / "user")
    store.directory.mkdir()
    write_profile(store.directory / (DEVICE.key + ".json"), default_profile())
    assert store.load(DEVICE) is None
    assert len(store.diagnostics) == 1
    with pytest.raises(ValueError):
        store.save(default_profile())


def test_display_preferences_never_contain_a_reference_or_override_device_measurements(tmp_path):
    preset = tmp_path / "preset.json"
    shared = default_profile()
    write_profile(preset, shared)
    store = TactileReferenceStore(tmp_path / "user", preset_path=preset)
    measured = profile(settings=ResponseSettings(800., "decrease"))
    store.save(measured)
    settings = ResponseSettings(600., "increase")
    store.save_default_settings(settings)
    document = json.loads((store.directory / "default-settings.json").read_text())
    assert "reference" not in document and "device" not in document
    assert store.load(DEVICE) == (measured, "saved")
    for identity in (OTHER, None):
        loaded, source = store.load(identity)
        assert source == "default" and loaded.settings == settings
        assert loaded.reference == shared.reference
    assert read_reference_profile(preset) == shared


@pytest.mark.parametrize("content", ["{broken", '{"settings":{}}',
    '{"schema_version":1,"scope":"layout","protocol":"other",'
    '"hand":"right","layout":"right_adc20x20_v1",'
    '"settings":{"display_range_adc":600,"direction":"both"}}'])
def test_corrupt_default_preferences_do_not_block_reference_loading(tmp_path, content):
    preset = tmp_path / "preset.json"
    shared = default_profile()
    write_profile(preset, shared)
    store = TactileReferenceStore(tmp_path / "user", preset_path=preset)
    store.directory.mkdir()
    (store.directory / "default-settings.json").write_text(content)
    assert store.load(None) == (shared, "default")
    assert len(store.diagnostics) == 1 and "default display settings" in store.diagnostics[0]
