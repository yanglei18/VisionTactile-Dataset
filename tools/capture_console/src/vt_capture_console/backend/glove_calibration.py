"""Observe a managed hand workflow and best-effort focus its existing window.

No subprocesses, serial IO, process signals, calibration commands, or acquisition
are issued here. X11 activation is an explicit user-requested focus hint only.
"""
from __future__ import annotations

from copy import deepcopy
import ctypes
import os
from pathlib import Path
import threading


_HANDS = {"left", "right"}
_FOCUS_STATES = {"STARTING", "RUNNING", "DEGRADED"}
_X11_LOCK = threading.Lock()
_FALLBACK = "标定进程可能已运行，请用 Alt+Tab 切换对应手的 FSGlove 窗口；不会重复启动采集。"


def glove_group(snapshot, hand):
    """Select by exact typed-stream topic, never by label or row position."""
    if hand not in _HANDS:
        raise ValueError("hand must be left or right")
    groups = (snapshot or {}).get("process_control", {}).get("groups", [])
    expected = f"/gloves/{hand}/state"
    matches = [g for g in groups if g.get("kind") == "glove" and expected in g.get("topics", [])
               and f"/gloves/{'left' if hand == 'right' else 'right'}/state" not in g.get("topics", [])]
    return deepcopy(matches[0]) if len(matches) == 1 else None


def inspect_status(snapshot, hand):
    """Describe observations without inventing a serial/calibration stage.

can_start is only manager permission, not permission to launch a second GUI.
The caller must additionally verify its configured profile has launch_gui=true.
"""
    group = glove_group(snapshot, hand)
    result = dict(state="UNAVAILABLE", message="后台状态不可确认；不会启动或接管手套进程。",
                  group=group, can_start=False, can_focus=False)
    if (not snapshot or snapshot.get("mode") != "live" or not snapshot.get("instance_id")
            or not snapshot.get("process_control", {}).get("enabled")
            or not snapshot.get("capabilities", {}).get("device_control")):
        return result
    if group is None:
        result.update(state="UNCONFIGURED", message="没有唯一匹配此手的姿态进程组；请先配置对应手模块。")
        return result
    state = group.get("state", "UNKNOWN")
    owned_pid = group.get("owned") is True and type(group.get("pid")) is int and group["pid"] > 0
    result["can_focus"] = owned_pid and state in _FOCUS_STATES
    if state == "EXTERNAL":
        result.update(state="EXTERNAL", message="此手已有外部发布程序；窗口关闭后后台仍可能运行。请退出该外部程序后重试。")
    elif state == "DISABLED":
        result.update(state="DISABLED", message="此手姿态模块未启用；请先在配置设备中启用。")
    elif state in {"STOPPED", "FAILED"} and not group.get("owned") and group.get("pid") is None:
        allowed = group.get("can_start") is True
        detail = group.get("detail")
        # Keep the manager's admission diagnostic intact for the existing UI
        # translation boundary. Never stringify arbitrary snapshot fields.
        blocked = detail if isinstance(detail, str) and detail.strip() else "此手进程已停止，但后台当前不允许启动；请核对录制与设备状态。"
        result.update(state="STARTABLE" if allowed else "STOPPED", can_start=allowed,
                      message="此手进程已停止；可通过托管启动打开标定窗口。" if allowed else blocked)
    elif state == "STOPPING":
        result.update(state="STOPPING", message="此手进程正在停止；等待退出，不会重复启动。")
    elif not owned_pid or state not in _FOCUS_STATES:
        result.update(state="UNKNOWN", message="此手进程归属或状态待核实；不会重复启动。")
    else:
        devices = [d for d in snapshot.get("devices", []) if d.get("kind") == "glove"
                   and d.get("side") == hand and d.get("topic") == f"/gloves/{hand}/state"]
        device = devices[0] if len(devices) == 1 else None
        if device and device.get("status") == "online":
            if device.get("preview", {}).get("valid") is True:
                result.update(state="VALID_DATA", message="有效手势正在更新；点击“标定”可回到原窗口。")
            else:
                result.update(state="INVALID_DATA", message="已收到状态，当前姿态尚未有效；点击“标定”检查输入和标定状态。")
        elif device and device.get("age_s") is not None:
            result.update(state="STALE_DATA", message="此手进程仍存在，但姿态消息已陈旧；可切换现有窗口检查，不会重启采集。")
        else:
            result.update(state="RUNNING_NO_DATA", message="节点已启动，尚无姿态数据。点击“标定”，在原窗口检查 IMU 连接与标定状态。")
    return result


calibration_action_state = inspect_status


def _read_process(pid):
    if type(pid) is not int or pid <= 0:
        return None
    try:
        directory = Path("/proc") / str(pid)
        if directory.stat().st_uid != os.getuid():
            return None
        with (directory / "stat").open("rb") as stream:
            raw = stream.read(8193)
        if len(raw) > 8192:
            return None
        fields = raw[raw.rindex(b")") + 2:].split()
        with (directory / "cmdline").open("rb") as stream:
            command = stream.read(32769)
        if len(command) > 32768 or fields[0] in {b"Z", b"X"}:
            return None
        return dict(pid=pid, ppid=int(fields[1]), pgid=int(fields[2]), session=int(fields[3]),
                    start_time=int(fields[19]), argv=tuple(part.decode("utf-8", errors="replace") for part in command.split(b"\0") if part))
    except (OSError, ValueError, IndexError):
        return None


def _calibration_command(argv, hand):
    if argv.count("-m") != 1 or argv.count("--hand") != 1:
        return False
    module = argv.index("-m")
    index = argv.index("--hand")
    return (tuple(argv[module + 1:module + 3]) == ("vt_fsglove12", "upstream-live")
            and index + 1 < len(argv) and argv[index + 1] == hand
            and not any(value.startswith("--hand=") for value in argv))


def _root_command(argv, hand):
    return any(Path(value).name == "ros2" and tuple(argv[i + 1:i + 4]) == ("launch", "vt_glove_bridge", "glove.launch.py")
               for i, value in enumerate(argv)) and f"hand:={hand}" in argv and f"hand:={'left' if hand == 'right' else 'right'}" not in argv


def _verified_chain(pid, root, hand):
    chain, visited = [], set()
    while pid > 0 and len(chain) < 32 and pid not in visited:
        visited.add(pid)
        process = _read_process(pid)
        if process is None:
            return None
        if not chain and not _calibration_command(process["argv"], hand):
            return None
        chain.append(process)
        if pid == root["pid"]:
            return chain if process == root else None
        pid = process["ppid"]
    return None


class _ClientData(ctypes.Union):
    _fields_ = [("b", ctypes.c_char * 20), ("s", ctypes.c_short * 10), ("l", ctypes.c_long * 5)]


class _ClientMessage(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("serial", ctypes.c_ulong), ("send_event", ctypes.c_int),
                ("display", ctypes.c_void_p), ("window", ctypes.c_ulong), ("message_type", ctypes.c_ulong),
                ("format", ctypes.c_int), ("data", _ClientData)]


class _Event(ctypes.Union):
    _fields_ = [("client", _ClientMessage), ("pad", ctypes.c_long * 24)]


class _X11:
    """Own display connection; bounded EWMH properties, no shell utilities."""
    def __init__(self):
        self.lib = ctypes.CDLL("libX11.so.6")
        void, ulong, integer = ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int
        functions = {
            "XOpenDisplay": ([ctypes.c_char_p], void), "XCloseDisplay": ([void], integer),
            "XDefaultRootWindow": ([void], ulong), "XInternAtom": ([void, ctypes.c_char_p, integer], ulong),
            "XGetWindowProperty": ([void, ulong, ulong, ctypes.c_long, ctypes.c_long, integer, ulong,
                ctypes.POINTER(ulong), ctypes.POINTER(integer), ctypes.POINTER(ulong), ctypes.POINTER(ulong),
                ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte))], integer),
            "XFree": ([void], integer), "XSync": ([void, integer], integer),
            "XSendEvent": ([void, ulong, integer, ctypes.c_long, ctypes.POINTER(_Event)], integer),
            "XSetErrorHandler": ([void], void),
        }
        for name, (arguments, result) in functions.items():
            function = getattr(self.lib, name)
            function.argtypes, function.restype = arguments, result
        self.display = self.lib.XOpenDisplay(None)
        if not self.display:
            raise OSError("X11 display unavailable")
        self.failed = False
        callback = ctypes.CFUNCTYPE(integer, void, void)
        self.handler = callback(self._error)
        self.previous_handler = self.lib.XSetErrorHandler(ctypes.cast(self.handler, void))
        self.root = self.lib.XDefaultRootWindow(self.display)

    def _error(self, _display, _event):
        self.failed = True
        return 0  # A vanished window must not invoke Xlib's fatal default handler.

    def close(self):
        try:
            self.lib.XSync(self.display, 0)
            self.lib.XCloseDisplay(self.display)
        finally:
            self.lib.XSetErrorHandler(self.previous_handler)

    def _property(self, window, name, expected_type):
        atom = self.lib.XInternAtom(self.display, name.encode("ascii"), 1)
        if not atom:
            return []
        actual, count, rest = ctypes.c_ulong(), ctypes.c_ulong(), ctypes.c_ulong()
        format = ctypes.c_int()
        data = ctypes.POINTER(ctypes.c_ubyte)()
        self.failed = False
        try:
            code = self.lib.XGetWindowProperty(self.display, window, atom, 0, 512, 0, expected_type,
                ctypes.byref(actual), ctypes.byref(format), ctypes.byref(count), ctypes.byref(rest), ctypes.byref(data))
            self.lib.XSync(self.display, 0)
            if code != 0 or self.failed or actual.value != expected_type or format.value != 32 or count.value > 512 or not data:
                return []
            values = ctypes.cast(data, ctypes.POINTER(ctypes.c_ulong))
            return [int(values[i]) for i in range(count.value)]
        finally:
            if data:
                self.lib.XFree(data)

    def window_pid(self, window):
        values = self._property(window, "_NET_WM_PID", 6)  # XA_CARDINAL
        return values[0] if len(values) == 1 and values[0] > 0 else None

    def windows(self):
        windows = self._property(self.root, "_NET_CLIENT_LIST_STACKING", 33)  # XA_WINDOW
        if not windows:
            windows = self._property(self.root, "_NET_CLIENT_LIST", 33)
        return [(window, self.window_pid(window)) for window in reversed(windows)]

    def activate(self, window):
        atom = self.lib.XInternAtom(self.display, b"_NET_ACTIVE_WINDOW", 1)
        if not atom:
            return False
        event = _Event()
        event.client.type, event.client.send_event, event.client.display = 33, 1, self.display
        event.client.window, event.client.message_type, event.client.format = window, atom, 32
        event.client.data.l[0] = 2  # Explicit user/pager request, not a device command.
        event.client.data.l[1] = 0  # CurrentTime; WM may decline focus stealing.
        self.failed = False
        sent = self.lib.XSendEvent(self.display, self.root, 0, (1 << 19) | (1 << 20), ctypes.byref(event))
        self.lib.XSync(self.display, 0)
        return bool(sent and not self.failed)


def focus_owned_calibration(group, hand):
    """Return (request_sent, message); do not claim the WM actually focused it."""
    if hand not in _HANDS:
        raise ValueError("hand must be left or right")
    if (not isinstance(group, dict) or group.get("kind") != "glove" or group.get("owned") is not True
            or group.get("state") not in _FOCUS_STATES or type(group.get("pid")) is not int or group["pid"] <= 0
            or f"/gloves/{hand}/state" not in group.get("topics", [])
            or f"/gloves/{'left' if hand == 'right' else 'right'}/state" in group.get("topics", [])):
        return False, "没有可确认归属的对应手标定进程；不会接管、启动或停止任何进程。"
    if os.environ.get("XDG_SESSION_TYPE", "").lower() == "wayland" or os.environ.get("WAYLAND_DISPLAY") or not os.environ.get("DISPLAY"):
        return False, "当前桌面不支持安全的 X11 前置。" + _FALLBACK
    root = _read_process(group["pid"])
    if (root is None or root["pgid"] != root["pid"] or root["session"] != root["pid"]
            or not _root_command(root["argv"], hand)):
        return False, "托管启动进程身份已不可确认。" + _FALLBACK
    connection = None
    with _X11_LOCK:
        try:
            connection = _X11()
            for window, pid in connection.windows():
                if type(pid) is not int or pid <= 0:
                    continue
                chain = _verified_chain(pid, root, hand)
                if not chain:
                    continue
                # Recheck PID ownership, birth identity and exact hand directly
                # before activation; recycled PID/window IDs must not be used.
                if connection.window_pid(window) != pid or _verified_chain(pid, root, hand) != chain:
                    continue
                if connection.activate(window):
                    return True, "已请求前置对应手的现有标定窗口；若未显示，请用 Alt+Tab 切换。未启动新采集。"
            return False, "未找到可核实的对应手窗口（可能仍在加载）。" + _FALLBACK
        except (OSError, AttributeError, ValueError):
            return False, "窗口系统不可用。" + _FALLBACK
        finally:
            if connection is not None:
                connection.close()
