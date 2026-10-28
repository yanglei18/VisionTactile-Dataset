from types import SimpleNamespace

from vt_hand_tactile_viewer import port_metadata as metadata


def entry(device="COM3", serial="board-a", **extra):
    return SimpleNamespace(device=device, serial_number=serial, vid=1155, pid=22336,
                           interface="CDC", location="1-2", **extra)


def setup_enumeration(monkeypatch, records, choices):
    from serial.tools import list_ports
    calls = []

    def native():
        calls.append("native")
        return records

    def choices_provider(*, native_ports):
        assert native_ports == records
        calls.append("choices")
        return choices

    monkeypatch.setattr(list_ports, "comports", native)
    monkeypatch.setattr(metadata, "list_serial_ports", choices_provider)
    return calls


def test_replacement_on_same_com_number_changes_identity(monkeypatch):
    records = [entry()]
    calls = setup_enumeration(monkeypatch, records, [dict(path="COM3", label="STM32")])
    first = metadata.list_port_candidates()[0]
    assert isinstance(first["identity"], str)
    records[0] = entry(serial="board-b")
    second = metadata.list_port_candidates()[0]
    assert first["path"] == second["path"] == "COM3"
    assert first["identity"] != second["identity"]
    assert calls == ["native", "choices", "native", "choices"]
    assert "role" not in first


def test_com_aliases_use_one_identity(monkeypatch):
    setup_enumeration(monkeypatch, [entry(device="com03")],
                      [dict(path=r"\\.\COM3", label="STM32")])
    assert metadata.list_port_candidates()[0]["identity"] is not None


def test_linux_symlink_uses_native_metadata_for_target(monkeypatch, tmp_path):
    tty = tmp_path / "ttyACM0"
    tty.touch()
    alias = tmp_path / "serial" / "by-id" / "usb-board-a"
    alias.parent.mkdir(parents=True)
    alias.symlink_to(tty)
    setup_enumeration(monkeypatch, [entry(device=str(tty))],
                      [dict(path=str(alias), label="usb-board-a")])
    result = metadata.list_port_candidates()[0]
    assert '"kind":"usb"' in result["identity"]
    assert result["path"] == str(alias)


def test_missing_serial_does_not_turn_location_or_com_into_identity(monkeypatch):
    setup_enumeration(monkeypatch, [entry(serial=None)],
                      [dict(path="COM3", label="STM32")])
    assert metadata.list_port_candidates()[0]["identity"] is None


def test_conflicting_native_aliases_remain_unknown(monkeypatch):
    setup_enumeration(monkeypatch, [entry(serial="a"), entry(serial="b")],
                      [dict(path="COM3", label="STM32")])
    assert metadata.list_port_candidates()[0]["identity"] is None


def test_by_id_fallback_uses_present_alias_and_tty_generation(monkeypatch, tmp_path):
    tty = tmp_path / "ttyACM0"
    tty.touch()
    alias = tmp_path / "serial" / "by-id" / "usb-board-a"
    alias.parent.mkdir(parents=True)
    alias.symlink_to(tty)
    setup_enumeration(monkeypatch, [], [dict(path=str(alias), label="usb-board-a")])
    first = metadata.list_port_candidates()[0]["identity"]
    replacement = tmp_path / "new-node"
    replacement.touch()
    replacement.replace(tty)
    assert first != metadata.list_port_candidates()[0]["identity"]
    tty.unlink()
    assert metadata.list_port_candidates()[0]["identity"] is None


def test_native_failure_keeps_enumerated_paths_but_no_fabricated_identity(monkeypatch):
    from serial.tools import list_ports

    def unavailable():
        raise OSError("metadata unavailable")

    monkeypatch.setattr(list_ports, "comports", unavailable)
    monkeypatch.setattr(metadata, "list_serial_ports", lambda **kwargs: [dict(path="COM9", label="manual")])
    assert metadata.list_port_candidates() == [dict(path="COM9", label="manual", identity=None)]


def test_windows_shared_provider_reuses_the_one_native_snapshot(monkeypatch):
    import serial
    from serial.tools import list_ports
    from serial.tools.list_ports_common import ListPortInfo
    from vt_capture_console.backend import tactile_serial

    calls = []
    port = ListPortInfo("COM12")
    port.description = "STM32"
    port.vid, port.pid, port.serial_number = 1155, 22336, "board-a"

    def native():
        calls.append("enumerate")
        assert len(calls) == 1
        return [port]

    def no_open(*args, **kwargs):
        raise AssertionError("metadata cannot open")

    monkeypatch.setattr(tactile_serial, "sys", SimpleNamespace(platform="win32"))
    monkeypatch.setattr(list_ports, "comports", native)
    monkeypatch.setattr(serial, "Serial", no_open)
    result = metadata.list_port_candidates()
    assert result[0]["path"] == "COM12"
    assert result[0]["label"] == "COM12 · STM32"
    assert '"serial":"board-a"' in result[0]["identity"]
    assert calls == ["enumerate"]
