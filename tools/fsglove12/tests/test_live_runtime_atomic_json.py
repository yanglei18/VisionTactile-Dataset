"""Atomic session JSON publication; no reader, hardware or timing benchmark."""
import json
import threading

import pytest

from vt_fsglove12 import live_runtime


@pytest.mark.parametrize("existing", [False, True])
def test_json_target_is_published_only_after_complete_serialization(tmp_path, monkeypatch, existing):
    target = tmp_path / "summary.json"
    previous = '{"previous": true}\n'
    if existing:
        target.write_text(previous)
    original_dump = json.dump
    entered, release = threading.Event(), threading.Event()
    errors = []
    value = {"完成": True, "frames": list(range(20))}

    def gated_dump(document, stream, **kwargs):
        original_dump(document, stream, **kwargs)
        stream.flush()
        entered.set()
        assert release.wait(2.)

    monkeypatch.setattr(live_runtime.json, "dump", gated_dump)

    def write():
        try:
            live_runtime._write_json(target, value)
        except BaseException as exc:
            errors.append(exc)

    thread = threading.Thread(target=write)
    thread.start()
    try:
        assert entered.wait(2.)
        if existing:
            assert target.read_text() == previous
        else:
            assert not target.exists()
        temporary = list(tmp_path.glob(".summary.json.*.tmp"))
        assert len(temporary) == 1 and temporary[0].parent == target.parent
    finally:
        release.set()
        thread.join(2.)
    assert not thread.is_alive()
    if existing:
        assert len(errors) == 1 and isinstance(errors[0], FileExistsError)
        assert target.read_text() == previous
    else:
        assert not errors
        text = target.read_text()
        assert json.loads(text) == value
        assert "完成" in text and text.endswith("\n")
    assert list(tmp_path.iterdir()) == [target]


@pytest.mark.parametrize("value,error", [({"bad": float("nan")}, ValueError),
                                        ({"bad": float("inf")}, ValueError),
                                        ({"bad": object()}, TypeError)])
@pytest.mark.parametrize("existing", [False, True])
def test_serialization_failure_keeps_old_json_and_removes_temporary(tmp_path, value, error, existing):
    target = tmp_path / "profile.json"
    previous = '{"valid": "old profile"}\n'
    if existing:
        target.write_text(previous)
    with pytest.raises(error):
        live_runtime._write_json(target, value)
    assert (target.read_text() == previous) if existing else not target.exists()
    assert list(tmp_path.iterdir()) == ([target] if existing else [])


def test_partial_temporary_write_failure_cannot_damage_existing_json(tmp_path, monkeypatch):
    target = tmp_path / "summary.json"
    previous = '{"valid": true}\n'
    target.write_text(previous)

    def failed_dump(document, stream, **kwargs):
        assert kwargs["allow_nan"] is False
        stream.write('{"partial":')
        stream.flush()
        raise OSError("simulated disk write failure")

    monkeypatch.setattr(live_runtime.json, "dump", failed_dump)
    with pytest.raises(OSError, match="disk write failure"):
        live_runtime._write_json(target, {"new": True})
    assert target.read_text() == previous
    assert list(tmp_path.iterdir()) == [target]


def test_link_failure_keeps_previous_file_and_cleans_complete_temporary(tmp_path, monkeypatch):
    target = tmp_path / "summary.json"
    previous = '{"old": true}\n'
    target.write_text(previous)

    def failed_link(temporary, destination):
        assert temporary.parent == destination.parent == tmp_path
        assert destination == target
        assert json.loads(temporary.read_text()) == {"new": True}
        raise OSError("simulated link failure")

    monkeypatch.setattr(live_runtime.os, "link", failed_link)
    with pytest.raises(OSError, match="link failure"):
        live_runtime._write_json(target, {"new": True})
    assert target.read_text() == previous
    assert list(tmp_path.iterdir()) == [target]
