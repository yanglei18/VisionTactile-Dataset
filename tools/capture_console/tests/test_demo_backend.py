import pytest

from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_core.protocol import encode_message, decode_message


def test_demo_snapshot_has_ten_explicit_synthetic_devices():
    backend = DemoBackend(clock=lambda: 10.0)
    snapshot = backend.snapshot()
    assert snapshot["mode"] == "demo"
    assert len(snapshot["devices"]) == 10
    assert all(d["preview"]["demo"] for d in snapshot["devices"])
    assert not snapshot["capabilities"]["record"]
    assert snapshot["resources"]["disk_free_bytes"] is None
    message = dict(version=1, request_id="test", ok=True, snapshot=snapshot)
    assert decode_message(encode_message(message)) == message


def test_demo_recording_is_in_memory_and_autocompletes(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    now = [10.0]
    backend = DemoBackend(clock=lambda: now[0])
    backend.start_recording(2)
    backend.start_recording(2)
    assert len(backend.trials) == 1
    assert backend.snapshot()["recording"]["state"] == "RECORDING"
    now[0] += 3
    assert backend.snapshot()["recording"]["state"] == "COMPLETE"
    assert backend.trials[0]["duration"] == 2
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize("duration", [0, -1, 301])
def test_demo_rejects_invalid_duration(duration):
    with pytest.raises(ValueError):
        DemoBackend().start_recording(duration)


def test_demo_hands_and_scenarios():
    backend = DemoBackend()
    backend.hands = "right"
    backend.scenario = "stale"
    devices = backend.snapshot()["devices"]
    assert len(devices) == 8
    assert all(d["status"] == "stale" for d in devices if d["kind"] in ("glove", "tactile"))
    assert all(d["side"] != "left" for d in devices)
