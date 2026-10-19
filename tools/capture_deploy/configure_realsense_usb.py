#!/usr/bin/env python3
"""Check, apply, or roll back narrowly scoped RealSense USB access changes.

Only apply/rollback require administrator authentication. No camera is opened,
no process is stopped, and the usbfs memory setting lasts only this boot.
"""

import argparse
import fcntl
import grp
import json
import os
from pathlib import Path
import stat
import subprocess
import tempfile


RULE = Path("/etc/udev/rules.d/70-visiontactile-realsense-usb.rules")
STATE = Path("/var/lib/visiontactile-realsense-usb.json")
LOCK = Path("/run/visiontactile-realsense-usb.lock")
MEMORY = Path("/sys/module/usbcore/parameters/usbfs_memory_mb")
BOOT = Path("/proc/sys/kernel/random/boot_id")
USB = Path("/sys/bus/usb/devices")
MEMORY_MB = 64
PRODUCTS = {"0b5b", "1156"}
RULE_TEXT = (
    "# Managed by VisionTactile configure_realsense_usb.py\n"
    'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="8086", '
    'ATTR{idProduct}=="0b5b", MODE="0660", GROUP="plugdev", TAG+="uaccess"\n'
    'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="8086", '
    'ATTR{idProduct}=="1156", MODE="0660", GROUP="plugdev", TAG+="uaccess"\n'
)


def trusted(path, *, missing=False):
    """Reject symlinks and writable/non-root system path components."""
    for item in [*reversed(path.parents), path]:
        try:
            info = item.lstat()
        except FileNotFoundError:
            if missing and item == path:
                return
            raise
        if stat.S_ISLNK(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022:
            raise RuntimeError(f"Unsafe system path: {item}")
    if path.exists() and not stat.S_ISREG(path.lstat().st_mode):
        raise RuntimeError(f"Not a regular file: {path}")


def read_optional(path):
    trusted(path, missing=True)
    return path.read_text() if path.exists() else None


def atomic_write(path, text, mode):
    trusted(path, missing=True)
    fd, temporary = tempfile.mkstemp(prefix=".vt-realsense-", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            os.fchmod(stream.fileno(), mode)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def run(*args, input=None):
    return subprocess.run(
        args, input=input, text=True, capture_output=True, check=True, timeout=15,
        env={"PATH": "/usr/sbin:/usr/bin:/sbin:/bin", "LC_ALL": "C"},
    ).stdout


def devices():
    found = []
    for entry in sorted(USB.iterdir()):
        try:
            if (entry / "idVendor").read_text().strip() != "8086":
                continue
            product = (entry / "idProduct").read_text().strip()
            if product not in PRODUCTS:
                continue
            bus = int((entry / "busnum").read_text())
            number = int((entry / "devnum").read_text())
            node = Path(f"/dev/bus/usb/{bus:03d}/{number:03d}")
            info = node.lstat()
            if not stat.S_ISCHR(info.st_mode):
                raise RuntimeError(f"Unexpected USB node: {node}")
            resolved = entry.resolve(strict=True)
            if not resolved.is_relative_to("/sys/devices"):
                raise RuntimeError(f"Unexpected USB sysfs path: {resolved}")
            found.append({
                "sysfs": str(resolved), "node": str(node), "product": product,
                "serial": (entry / "serial").read_text().strip(),
                "uid": info.st_uid, "gid": info.st_gid,
                "mode": stat.S_IMODE(info.st_mode),
                "readable": os.access(node, os.R_OK),
                "writable": os.access(node, os.W_OK),
                "acl": run("/usr/bin/getfacl", "-c", "-n", "--", str(node)),
            })
        except FileNotFoundError:
            continue  # A disconnected device is not opened or reset.
    return found


def check():
    rule = read_optional(RULE)
    return {
        "command": "check", "uid": os.geteuid(), "groups": os.getgroups(),
        "usbfs_memory_mb": int(MEMORY.read_text()), "requested_memory_mb": MEMORY_MB,
        "rule_path": str(RULE), "rule_installed": rule == RULE_TEXT,
        "rule_conflict": rule is not None and rule != RULE_TEXT,
        "backup_present": STATE.exists(), "devices": devices(),
    }


def memory_write(value):
    trusted(MEMORY)
    fd = os.open(MEMORY, os.O_WRONLY | os.O_NOFOLLOW)
    try:
        os.write(fd, f"{value}\n".encode())
    finally:
        os.close(fd)
    if int(MEMORY.read_text()) != value:
        raise RuntimeError("usbfs memory setting verification failed")


def refresh(current):
    run("/usr/bin/udevadm", "control", "--reload-rules")
    for device in current:
        if Path(device["sysfs"]).exists():
            run("/usr/bin/udevadm", "trigger", "--action=change", device["sysfs"])
    run("/usr/bin/udevadm", "settle", "--timeout=10")


def load_state():
    raw = read_optional(STATE)
    if raw is None:
        return None
    state = json.loads(raw)
    if (state.get("version") != 1 or state.get("rule") not in (None, RULE_TEXT)
            or type(state.get("memory_mb")) is not int
            or not 0 <= state["memory_mb"] <= 1_000_000
            or type(state.get("applied_memory_mb")) is not int
            or not 0 <= state["applied_memory_mb"] <= 1_000_000):
        raise RuntimeError("Unrecognized backup; refusing changes")
    return state


def command_preflight():
    for executable in ("/usr/bin/udevadm", "/usr/bin/getfacl", "/usr/bin/setfacl"):
        trusted(Path(executable))


def restore(state):
    """Restore only recorded settings, retaining backup if any step fails."""
    command_preflight()
    rule = read_optional(RULE)
    if rule not in (RULE_TEXT, state["rule"]):
        raise RuntimeError("Rule changed outside this helper; refusing rollback")
    same_boot = state["boot_id"] == BOOT.read_text().strip()
    memory = int(MEMORY.read_text())
    if same_boot and memory not in (state["applied_memory_mb"], state["memory_mb"]):
        raise RuntimeError("usbfs memory changed outside this helper; refusing rollback")
    current = devices()
    if state["rule"] is None:
        RULE.unlink(missing_ok=True)
    else:
        atomic_write(RULE, state["rule"], state["rule_mode"])
    if same_boot:
        memory_write(state["memory_mb"])
    refresh(current)
    saved = {(d["product"], d["serial"]): d for d in state["devices"]}
    unrecorded = []
    # Removing a rule alone may leave the current device node's ACL/mode intact.
    for device in devices():
        original = saved.get((device["product"], device["serial"]))
        if original:
            node = device["node"]
            os.chown(node, original["uid"], original["gid"], follow_symlinks=False)
            os.chmod(node, original["mode"], follow_symlinks=False)
            run("/usr/bin/setfacl", "--set-file=-", "--", node, input=original["acl"])
        else:
            unrecorded.append(device["node"])
    if unrecorded:
        raise RuntimeError("Rule and memory restored, but USB devices connected after backup "
                           f"have unknown original permissions: {unrecorded}; backup retained")
    STATE.unlink()
    return {"command": "rollback", "restored": True, "memory_restored": same_boot}


def apply():
    grp.getgrnam("plugdev")
    command_preflight()
    rule = read_optional(RULE)
    if rule not in (None, RULE_TEXT):
        raise RuntimeError(f"Refusing to overwrite another rule: {RULE}")
    state = load_state()
    boot = BOOT.read_text().strip()
    if state is None:
        state = {
            "version": 1, "boot_id": boot, "memory_mb": int(MEMORY.read_text()),
            "rule": rule, "rule_mode": stat.S_IMODE(RULE.stat().st_mode) if rule else 0o644,
            "devices": devices(),
        }
    elif state["boot_id"] != boot:
        state["boot_id"] = boot
        state["memory_mb"] = int(MEMORY.read_text())
    current_memory = int(MEMORY.read_text())
    if current_memory not in (state["memory_mb"], state.get("applied_memory_mb")):
        raise RuntimeError("usbfs memory changed outside this helper; refusing apply")
    # Zero already means unlimited; never reduce a pre-existing administrator limit.
    state["applied_memory_mb"] = 0 if current_memory == 0 else max(MEMORY_MB, current_memory)
    atomic_write(STATE, json.dumps(state, indent=2) + "\n", 0o600)
    try:
        atomic_write(RULE, RULE_TEXT, 0o644)
        memory_write(state["applied_memory_mb"])
        refresh(devices())
    except BaseException as exc:
        try:
            restore(state)
        except BaseException as rollback_error:
            raise RuntimeError(f"Apply failed: {exc}; rollback incomplete: {rollback_error}; "
                               f"backup retained at {STATE}") from exc
        raise RuntimeError(f"Apply failed and original settings restored: {exc}") from exc
    return {"command": "apply", "applied": True, "backup_path": str(STATE),
            "usbfs_memory_mb": state["applied_memory_mb"], "persistent_memory_change": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("check", "apply", "rollback"))
    args = parser.parse_args(argv)
    try:
        if args.command == "check":
            result = check()
        else:
            if os.geteuid() != 0:
                raise RuntimeError("Administrator authentication is required for apply/rollback")
            trusted(LOCK, missing=True)
            fd = os.open(LOCK, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            with os.fdopen(fd, "w") as lock:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                state = load_state() if args.command == "rollback" else None
                result = apply() if args.command == "apply" else (
                    restore(state) if state else {"command": "rollback", "restored": False,
                                                  "reason": "No backup exists"})
        print(json.dumps(result, indent=2))
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, KeyError) as exc:
        print(json.dumps({"command": args.command, "error": str(exc)}, indent=2))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
