"""Enumerate serial paths and retain an operator's chosen device identity.

Filesystem metadata is sufficient to replace a selected tty/by-path name with
its unique by-id alias. It cannot identify a device's protocol, hand, or role;
unconfigured and disconnected devices are therefore never assigned a port.
No serial library is imported and no device is opened, read, or configured.
"""
from __future__ import annotations

from pathlib import Path


DEV_ROOT = Path("/dev")


def _aliases(root, category):
    try:
        paths = sorted((root / "serial" / category).iterdir())
    except OSError:
        return
    for path in paths:
        try:
            if path.is_symlink():
                yield path, path.resolve(strict=True)
        except (OSError, RuntimeError):
            continue


def stable_serial_path(value, dev_root=None):
    """Promote only a present, explicitly chosen tty/by-path to one by-id alias.

    Existing by-id choices, missing devices, custom aliases, and ambiguous
    identities are preserved verbatim. In particular, the only available port
    is not evidence that it belongs to the requested hand or device protocol.
    """
    if not value:
        return value
    root = DEV_ROOT if dev_root is None else Path(dev_root)
    path = Path(value)
    selected_tty = path.parent == root and path.name.startswith(("ttyACM", "ttyUSB"))
    if not selected_tty and path.parent != root / "serial/by-path":
        return value
    try:
        target = path.resolve(strict=True)
    except (OSError, RuntimeError):
        return value
    matches = [alias for alias, device in _aliases(root, "by-id") if device == target]
    if len(matches) != 1:
        return value
    stable = str(matches[0])
    try:
        valid = len(stable.encode()) <= 4096 and not any(char in stable for char in ("\0", "\n", "\r"))
    except UnicodeError:
        valid = False
    return stable if valid else value


def serial_ports(dev_root=None):
    """List one path per present device, preferring by-id, then by-path."""
    root = DEV_ROOT if dev_root is None else Path(dev_root)
    choices, targets = [], set()
    for category in ("by-id", "by-path"):
        for path, target in _aliases(root, category):
            if target not in targets:
                targets.add(target)
                choices.append(dict(path=str(path), label=f"{path.name} → {target.name}"))
    for pattern in ("ttyACM*", "ttyUSB*"):
        for path in sorted(root.glob(pattern)):
            try:
                target = path.resolve(strict=True)
            except (OSError, RuntimeError):
                continue
            if target not in targets:
                targets.add(target)
                choices.append(dict(path=str(path), label=path.name + "（未指定设备身份）"))
    return choices
