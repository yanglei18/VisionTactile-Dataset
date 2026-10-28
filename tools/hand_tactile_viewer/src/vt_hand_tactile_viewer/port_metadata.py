"""Enumeration metadata used only to invalidate protocol evidence on replacement.

A USB identity never supplies a protocol role. It associates independently
observed data with the current enumerated device. Missing or ambiguous metadata
is deliberately represented by None, never a COM/tty number masquerading as a
durable device identity. No device is opened here.
"""
from __future__ import annotations

import json
from pathlib import Path

from vt_capture_console.backend.tactile_serial import list_serial_ports

from .port_identity import port_identity


def _usb_identity(entry):
    serial = getattr(entry, "serial_number", None)
    vid, pid = getattr(entry, "vid", None), getattr(entry, "pid", None)
    if (not isinstance(serial, str) or not serial.strip()
            or any(type(value) is not int or not 0 <= value <= 65535 for value in (vid, pid))):
        return None
    # Interface distinguishes endpoints on one USB device; location invalidates
    # evidence when it moves. These fields are metadata, not a role registry.
    fields = dict(kind="usb", vid=vid, pid=pid, serial=serial.strip(),
                  interface=str(getattr(entry, "interface", None) or ""),
                  location=str(getattr(entry, "location", None) or ""))
    return json.dumps(fields, sort_keys=True, ensure_ascii=True, separators=(",", ":"))


def _by_id_identity(path):
    selected = Path(path)
    if selected.parent.name != "by-id" or selected.parent.parent.name != "serial":
        return None
    try:
        # The inode also changes when Linux recreates the tty during a replug.
        # Include the stable by-id alias rather than inferring its serial format.
        metadata = selected.stat()
    except OSError:
        return None
    return json.dumps(dict(kind="by-id", alias=selected.name, device=metadata.st_rdev,
                           inode=metadata.st_ino), sort_keys=True, separators=(",", ":"))


def list_port_candidates():
    """Return the shared port list plus optional current device identities."""
    try:
        from serial.tools import list_ports
        native = list(list_ports.comports())
    except (ImportError, OSError):
        native = []
    identities = {}
    for entry in native:
        path = str(getattr(entry, "device", "") or "")
        if path:
            identities.setdefault(port_identity(path), set()).add(_usb_identity(entry))
    candidates = []
    for candidate in list_serial_ports(native_ports=native):
        path = candidate["path"]
        matches = identities.get(port_identity(path), set())
        identity = next(iter(matches)) if len(matches) == 1 else None
        if not matches:
            identity = _by_id_identity(path)
        # Contradictory native identities must not fall back to a guessed alias.
        candidates.append(dict(candidate, identity=identity))
    return candidates
