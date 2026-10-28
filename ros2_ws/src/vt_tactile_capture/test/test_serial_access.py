"""Explicit discovery transport tests use fake objects and private PTYs only."""
import builtins
import os
import sys
from types import SimpleNamespace

import pytest

from vt_tactile_capture import serial_access as access
from vt_tactile_capture import serial_source


def test_capture_and_discovery_share_owner_error_type():
    assert serial_source.PortInUseError is access.PortInUseError


@pytest.mark.parametrize("options", [
    {"port": ""}, {"port": None}, {"baudrate": 0}, {"baudrate": True},
    {"baudrate": 115200.0}, {"read_timeout": 0}, {"read_timeout": float("nan")},
    {"read_timeout": float("inf")}, {"read_timeout": "1"}, {"passive": "yes"},
])
def test_bad_configuration_fails_before_import_or_open(options, monkeypatch):
    original = builtins.__import__

    def import_without_serial(name, *args, **kwargs):
        if name == "serial":
            raise AssertionError("must validate before serial import")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", import_without_serial)
    with pytest.raises(ValueError):
        access.open_serial(**(dict(port="fake", baudrate=115200) | options))


def test_passive_hooks_suppress_open_flush_lines_and_forbid_commands():
    calls = []

    class Base:
        def _reset_input_buffer(self):
            calls.append("flush")

        def _update_dtr_state(self):
            calls.append("dtr")

        def _update_rts_state(self):
            calls.append("rts")

    device = access._passive_class(Base)()
    device._reset_input_buffer()
    device._update_dtr_state()
    device._update_rts_state()
    assert not calls
    for method, args in (("write", (b"unsafe",)), ("reset_input_buffer", ()),
                         ("reset_output_buffer", ()), ("send_break", ())):
        with pytest.raises(RuntimeError, match="Protocol discovery cannot"):
            getattr(device, method)(*args)


def test_windows_passive_retains_native_open_without_posix_checks(monkeypatch):
    serial = pytest.importorskip("serial")
    received = []

    class Base:
        def __init__(self, **options):
            received.append(options)

        def close(self):
            received.append("closed")

    monkeypatch.setattr(serial, "Serial", Base)
    monkeypatch.setattr(access, "sys", SimpleNamespace(platform="win32"))
    monkeypatch.setattr(access, "_assert_port_free", lambda *a, **k: pytest.fail("no /proc on Windows"))
    original = builtins.__import__

    def no_posix(name, *args, **kwargs):
        if name in ("fcntl", "termios"):
            raise AssertionError("no POSIX on Windows")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", no_posix)
    device = access.open_serial("COM12", 115200, .02, passive=True)
    access.close_serial(device)
    assert received == [dict(port="COM12", baudrate=115200, timeout=.02, write_timeout=.02), "closed"]


@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="Linux private PTY")
def test_passive_private_pty_preserves_queued_bytes_and_releases_ownership():
    pytest.importorskip("serial")
    import pty
    import tty

    master, slave = pty.openpty()
    port = os.ttyname(slave)
    # Queue bytes before the discovery open. Default pyserial would flush them.
    tty.setraw(slave)
    os.write(master, b"queued-before-probe")
    os.close(slave)
    device = reopened = None
    try:
        device = access.open_serial(port, 115200, .1, passive=True)
        assert device.read(19) == b"queued-before-probe"
        with pytest.raises(access.PortInUseError):
            access.open_serial(port, 115200, .1, passive=True)
        access.close_serial(device)
        device = None
        reopened = access.open_serial(port, 115200, .1, passive=True)
        os.write(master, b"again")
        assert reopened.read(5) == b"again"
    finally:
        for opened in (device, reopened):
            if opened is not None:
                access.close_serial(opened)
        os.close(master)
