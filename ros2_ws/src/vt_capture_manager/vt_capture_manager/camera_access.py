"""Read-only camera launch admission after a confirmed Linux USB failure.

Missing cameras alone do not establish a controller failure. That diagnosis
requires missing configured devices and an unrecovered ``HC died`` event from
the current boot. Other absence/unknown cases have separate reasons. This guard
never opens USB devices, resets a controller or retries a launch.
"""
from __future__ import annotations

import json
from pathlib import Path
import re
import subprocess
import threading
import time

import yaml


_EVENT = re.compile(
    r"^xhci_hcd ([0-9a-f]{4}:[0-9a-f]{2}:[0-9a-f]{2}\.[0-7]): "
    r"(HC died; cleaning up|new USB bus registered, assigned bus number [0-9]+)$"
)
_JOURNAL_MATCH = r"^xhci_hcd [0-9a-f:.]+: (HC died; cleaning up|new USB bus registered, assigned bus number [0-9]+)$"


def failed_controllers(records, boot_id):
    """Reduce narrowly selected kernel events, independent of journal ordering.

    Re-registering the controller's USB bus clears its old failure. Monotonic
    timestamps avoid wall-clock adjustments and previous boots cannot latch it.
    Malformed or unrelated records provide no positive failure evidence.
    """
    latest = {}
    for record in records:
        if not isinstance(record, dict) or record.get("_BOOT_ID") != boot_id:
            continue
        message = record.get("MESSAGE")
        match = _EVENT.fullmatch(message) if isinstance(message, str) else None
        stamp = record.get("__MONOTONIC_TIMESTAMP")
        if match is None or not isinstance(stamp, str) or not stamp.isdigit():
            continue
        address, event = match.groups()
        value = (int(stamp), event.startswith("HC died"))
        if address not in latest or value > latest[address]:
            latest[address] = value
    return tuple(sorted(address for address, (_, failed) in latest.items() if failed))


def _configured_serials(path):
    try:
        with Path(path).open("rb") as stream:
            raw = stream.read(65537)
        if len(raw) > 65536:
            return ()
        document = yaml.safe_load(raw)
        cameras = document.get("cameras") if isinstance(document, dict) else None
        if not isinstance(cameras, list) or not 1 <= len(cameras) <= 32:
            return ()
        serials = []
        for camera in cameras:
            if not isinstance(camera, dict):
                return ()
            # RealSense sysfs can expose the ASIC serial rather than the serial
            # selected through the SDK; either is evidence of an attached unit.
            for key in ("serial", "asic_serial"):
                value = camera.get(key)
                if isinstance(value, str) and re.fullmatch(r"[0-9]{1,32}", value):
                    serials.append(value)
        return tuple(serials)
    except (OSError, ValueError, yaml.YAMLError):
        # The launch recipe owns configuration validation. An unreadable file
        # must never be presented as proof of a failed USB controller.
        return ()


def _enumerated_serials(root):
    try:
        devices = tuple(Path(root).iterdir())
    except OSError:
        return None
    serials = set()
    for device in devices:
        if ":" in device.name:  # USB interfaces do not identify extra cameras.
            continue
        try:
            if (device / "idVendor").read_text().strip() == "8086":
                serials.add((device / "serial").read_text().strip())
        except FileNotFoundError:
            continue
        except OSError:
            # Incomplete access/disappearing sysfs is not absence evidence.
            return None
    return serials


class CameraStartGuard:
    """Bound kernel-log reads while keeping USB presence checks current.

    Snapshot requests share a short cache; they cannot spawn a journal process
    on every GUI refresh. The command is kernel-only, current-boot, regex-filtered
    and bounded in records/time. No sudo, SDK or ROS calls are involved.
    """
    def __init__(self, *, usb_root="/sys/bus/usb/devices", boot_id_path="/proc/sys/kernel/random/boot_id",
                 cache_s=2., clock=time.monotonic, run=subprocess.run):
        self._usb_root = usb_root
        self._boot_id_path = boot_id_path
        self._cache_s, self._clock, self._run = cache_s, clock, run
        self._cache = {}
        self._lock = threading.Lock()

    def _controller_failures(self):
        try:
            boot_id = Path(self._boot_id_path).read_text().strip().replace("-", "")
            if re.fullmatch(r"[0-9a-f]{32}", boot_id) is None:
                return None
            result = self._run(
                ["journalctl", "--dmesg", "--boot=" + boot_id, "--no-pager", "--quiet",
                 "--grep=" + _JOURNAL_MATCH, "--lines=128", "--output=json",
                 "--output-fields=MESSAGE,__MONOTONIC_TIMESTAMP,_BOOT_ID"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=.35, check=False,
            )
            # Exit 1 can mean no matches or denied access; neither proves health.
            if result.returncode or len(result.stdout) > 262144:
                return None
            records = [json.loads(line) for line in result.stdout.splitlines() if line.strip()]
            return failed_controllers(records, boot_id)
        except (OSError, subprocess.TimeoutExpired, ValueError):
            return None

    def __call__(self, resources, *, refresh=False):
        paths = tuple(value.removeprefix("camera-config:") for value in resources
                      if value.startswith("camera-config:"))
        if "realsense-cameras" not in resources or not paths:
            return None
        key = tuple(resources)
        with self._lock:
            now = self._clock()
            cached = self._cache.get(key)
            if not refresh and cached is not None and now < cached[0]:
                return cached[1]
            problem = self._check(paths)
            if len(self._cache) >= 32:
                self._cache.clear()
            self._cache[key] = (self._clock() + self._cache_s, problem)
            return problem

    def _check(self, paths):
        expected = {serial for path in paths for serial in _configured_serials(path)}
        if not expected:
            return None
        enumerated = _enumerated_serials(self._usb_root)
        if enumerated is None:
            return ("camera_diagnostics_unavailable", "无法读取 USB 设备枚举状态；相机启动前检查未完成，已阻止启动。")
        if expected.intersection(enumerated):
            return None
        failed = self._controller_failures()
        if failed is None:
            return ("camera_diagnostics_unavailable",
                    "当前未枚举到配置的相机，且无法确认 USB 控制器状态；已阻止相机启动，避免驱动持续重试。")
        if not failed:
            return ("camera_unavailable", "当前未枚举到配置的相机；已阻止相机启动，相机重新连接后可再次启动。")
        addresses = ", ".join(failed)
        return ("usb_controller_failed",
                f"内核报告 USB 控制器 {addresses} 失效（HC died），当前未枚举到配置的相机。"
                "已阻止相机启动，避免驱动持续重试；控制器恢复且相机重新枚举后可再次启动。")
