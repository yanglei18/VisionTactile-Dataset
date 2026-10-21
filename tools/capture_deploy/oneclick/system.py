"""Fixed, privileged installation actions; never a general command service.

Only the explicit pkexec entry point starts this module. Build and Python
package installation remain in the ordinary user's process. System package
transactions finish before cancellation/timeout is reported, including when
the graphical installer disappears.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import pwd
import re
import selectors
import signal
import stat
import subprocess
import sys
import tempfile
import time
import urllib.request

from .contract import DEPENDENCIES_RELATIVE, SDK_VERSION, WRAPPER_VERSION


ROS_SOURCE_VERSION = "1.3.0"
ROS_SOURCE_URL = (
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/"
    "1.3.0/ros2-apt-source_1.3.0.noble_all.deb"
)
# Official ros-infrastructure/ros-apt-source release asset digest, 2026-10-21.
ROS_SOURCE_SHA256 = "f31d84adf5054c7d60ded0e82c0f776a77ea33af53d08f40b9ad7c94cca55296"
BASE_PACKAGES = (
    "git", "curl", "ca-certificates", "locales", "acl", "usbutils", "pkexec",
    "python3-venv", "python3-tk", "libegl1", "libgl1", "libxkbcommon0",
    "libxcb-cursor0", "fonts-noto-cjk", "build-essential", "cmake", "pkg-config",
    "libusb-1.0-0-dev", "libudev-dev", "libssl-dev", "libhidapi-hidraw0",
    "libhidapi-libusb0",
)
ROS_PACKAGES = (
    "ros-jazzy-desktop", "ros-dev-tools", "python3-colcon-common-extensions",
    "python3-rosdep", "ros-jazzy-ros2bag", "ros-jazzy-rosbag2-py",
    "ros-jazzy-rosbag2-storage-mcap", "python3-yaml", "python3-numpy",
    "python3-opencv", "python3-serial", "fio",
)
TRACKER_PRODUCTS = ("0350", "06a3")
TRACKER_RULE = "# Managed by VisionTactile one-click installer\n" + "".join(
    f'SUBSYSTEM=="hidraw", ATTRS{{idVendor}}=="0bb4", ATTRS{{idProduct}}=="{product}", '
    'MODE="0660", GROUP="plugdev", TAG+="uaccess"\n' for product in TRACKER_PRODUCTS
)
USBFS_HELPER = '''#!/usr/bin/python3
"""Raise only a finite usbfs limit below 64 MiB; preserve unlimited and larger limits."""
from pathlib import Path
import os
path = Path("/sys/module/usbcore/parameters/usbfs_memory_mb")
value = int(path.read_text().strip())
if 0 < value < 64:
    descriptor = os.open(path, os.O_WRONLY | os.O_NOFOLLOW)
    try:
        os.write(descriptor, b"64\\n")
    finally:
        os.close(descriptor)
'''
USBFS_SERVICE = """[Unit]
Description=VisionTactile minimum USB transfer memory
After=systemd-modules-load.service
ConditionPathExists=/sys/module/usbcore/parameters/usbfs_memory_mb

[Service]
Type=oneshot
ExecStart=/usr/bin/python3 -I /usr/local/lib/visiontactile/ensure-usbfs-memory.py

[Install]
WantedBy=multi-user.target
"""


class SystemError(RuntimeError):
    """User-facing diagnostic with a stable, non-sensitive code."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def system_environment():
    """Do not inherit user Python, proxy, loader, package, or credential settings."""
    return {
        "PATH": "/usr/sbin:/usr/bin:/sbin:/bin", "HOME": pwd.getpwuid(0).pw_dir,
        "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8", "DEBIAN_FRONTEND": "noninteractive",
        "PYTHONNOUSERSITE": "1", "ROS_DISTRO": "jazzy",
        "AMENT_PREFIX_PATH": "/opt/ros/jazzy",
    }


class CommandRunner:
    """Stream output without killing a running package-manager transaction."""

    def __init__(self, emit, cancelled=lambda: False):
        self.emit = emit
        self.cancelled = cancelled

    def run(self, arguments, *, check=True, timeout=7200):
        if self.cancelled():
            raise SystemError("cancelled", "已取消；当前系统事务已安全结束。")
        process = subprocess.Popen(
            arguments, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, env=system_environment(), start_new_session=True,
        )
        output = bytearray()
        late = False
        deadline = time.monotonic() + timeout
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            while selector.get_map():
                if not late and time.monotonic() > deadline:
                    late = True
                    self.emit({"kind": "log", "text":
                               "系统配置超过预计时间；正在等待当前事务结束，不会强制中断软件包安装。"})
                for key, _ in selector.select(timeout=1):
                    chunk = os.read(key.fileobj.fileno(), 8192)
                    if not chunk:
                        selector.unregister(key.fileobj)
                        continue
                    # Only fixed system commands are executed; no private Tracker
                    # file is an argument or a source of command output.
                    self.emit({"kind": "log", "text": chunk.decode("utf-8", "replace")})
                    if len(output) < 8 * 1024 * 1024:
                        output.extend(chunk)
        process.stdout.close()
        returncode = process.wait()
        if late:
            raise SystemError("command_timeout", "系统事务已结束，但执行时间超限；请检查日志后重试。")
        if self.cancelled():
            raise SystemError("cancelled", "已取消；当前系统事务已安全结束。")
        if check and returncode:
            raise SystemError("command_failed", "系统配置命令失败；请根据安装日志处理后重试。")
        return returncode, output.decode("utf-8", "replace")


def validate_identity(uid, repo_root):
    if os.geteuid() != 0 or uid <= 0 or os.environ.get("PKEXEC_UID") != str(uid):
        raise SystemError("identity_invalid", "请从普通用户的图形安装器授权系统安装。")
    account = pwd.getpwuid(uid)
    home = Path(account.pw_dir)
    root = Path(repo_root)
    if not home.is_absolute() or home == Path("/") or not root.is_absolute():
        raise SystemError("path_invalid", "安装目录无效。")
    root = root.resolve(strict=True)
    if root != Path(__file__).resolve().parents[3]:
        raise SystemError("path_invalid", "安装请求与当前交付程序目录不一致。")
    if not root.is_relative_to(home.resolve(strict=True)):
        raise SystemError("path_invalid", "程序必须安装在当前用户目录内。")
    info = root.stat()
    if info.st_uid != uid or info.st_mode & 0o022 or not root.is_dir():
        raise SystemError("path_invalid", "程序目录所属用户或权限不正确。")
    return account, root


def check_platform():
    values = {}
    for line in Path("/etc/os-release").read_text().splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value.strip('"')
    if (values.get("ID") != "ubuntu" or values.get("VERSION_ID") != "24.04"
            or platform.machine() != "x86_64" or sys.version_info[:2] != (3, 12)):
        raise SystemError("platform_unsupported", "此安装器需要 Ubuntu 24.04 x86_64 和系统 Python 3.12。")


class SystemActions:
    def __init__(self, account, repo_root, runner, *, system_root=Path("/"), system_uid=0):
        self.account, self.repo = account, Path(repo_root)
        self.home, self.runner = Path(account.pw_dir), runner
        self.system_root = Path(system_root)
        self.system_uid = system_uid
        dependencies = self.home / DEPENDENCIES_RELATIVE
        self.sdk = dependencies / f"librealsense-{SDK_VERSION}"
        self.driver = dependencies / f"drivers-{WRAPPER_VERSION}"

    def path(self, value):
        return self.system_root / value.lstrip("/")

    def run(self, *arguments, **kwargs):
        return self.runner.run(list(arguments), **kwargs)

    def apt(self, *packages):
        self.run("/usr/bin/apt-get", "-o", "DPkg::Lock::Timeout=120", "--no-remove",
                 "install", "-y", *packages)

    def user_directory(self, path):
        """Reject foreign/symlink build trees before privileged use."""
        if not path.is_relative_to(self.home):
            raise SystemError("path_invalid", "构建目录不在当前用户目录内。")
        current = self.home
        for part in (None, *path.relative_to(self.home).parts):
            if part is not None:
                current /= part
            info = current.lstat()
            if (not stat.S_ISDIR(info.st_mode) or info.st_uid != self.account.pw_uid
                    or info.st_mode & 0o022):
                raise SystemError("path_invalid", "构建目录包含链接、其他用户目录或可写共享目录。")

    def base(self):
        self.run("/usr/bin/apt-get", "update")
        self.apt("software-properties-common")
        self.run("/usr/bin/add-apt-repository", "-y", "universe")
        self.run("/usr/bin/apt-get", "update")
        self.apt(*BASE_PACKAGES)

    def ros(self):
        status, package_status = self.run("/usr/bin/dpkg-query", "-W", "-f=${Status}",
                                          "ros2-apt-source", check=False)
        if status or package_status.strip() != "install ok installed":
            for filename in ("ros2.list", "ros2.sources"):
                if self.path("/etc/apt/sources.list.d").joinpath(filename).exists():
                    raise SystemError("ros_source_conflict", "已有自定义 ROS 软件源；请维护人员确认后再安装。")
            with tempfile.TemporaryDirectory(prefix="vt-ros-source-") as directory:
                target = Path(directory) / "ros2-apt-source.deb"
                request = urllib.request.Request(ROS_SOURCE_URL, headers={"User-Agent": "VisionTactile-installer"})
                with urllib.request.urlopen(request, timeout=60) as response:
                    content = response.read(1024 * 1024 + 1)
                if (len(content) > 1024 * 1024
                        or hashlib.sha256(content).hexdigest() != ROS_SOURCE_SHA256):
                    raise SystemError("source_digest_invalid", "ROS 软件源下载校验失败；未执行安装。")
                target.write_bytes(content)
                self.run("/usr/bin/dpkg", "--install", str(target))
        self.run("/usr/bin/apt-get", "update")
        self.apt(*ROS_PACKAGES)
        if not self.path("/etc/ros/rosdep/sources.list.d/20-default.list").exists():
            self.run("/usr/bin/rosdep", "init")
        self.run("/usr/bin/rosdep", "update", "--rosdistro", "jazzy")

    def deps(self, target):
        if target not in ("driver", "main"):
            raise SystemError("request_invalid", "不支持此依赖安装目标。")
        if target == "driver":
            self.user_directory(self.driver / "src/realsense-ros")
            paths = [self.driver / "src/realsense-ros" / name for name in
                     ("realsense2_camera", "realsense2_camera_msgs", "realsense2_description")]
        else:
            paths = [self.repo / "ros2_ws/src"]
        for path in paths:
            self.user_directory(path)
        _, keys_text = self.run("/usr/bin/rosdep", "keys", "--from-paths", *map(str, paths),
                                "--ignore-src", "--rosdistro", "jazzy")
        skip = {"librealsense2", "realsense2_camera", "realsense2_camera_msgs"}
        keys = sorted(set(keys_text.split()) - skip)
        if any(not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.+-]*", key) for key in keys):
            raise SystemError("dependency_invalid", "依赖解析返回了无效的依赖名称。")
        if not keys:
            return
        _, resolved = self.run("/usr/bin/rosdep", "resolve", *keys, "--rosdistro", "jazzy")
        packages, groups, installer = set(), set(), None
        for line in resolved.splitlines():
            group = re.fullmatch(r"#ROSDEP\[([A-Za-z0-9][A-Za-z0-9_.+-]*)\]", line)
            if group:
                if group[1] not in keys or group[1] in groups:
                    raise SystemError("dependency_invalid", "依赖解析返回了未知或重复的分组。")
                groups.add(group[1])
                installer = None
            elif line.startswith("#"):
                installer = line[1:].strip()
                if installer != "apt":
                    raise SystemError("dependency_installer_invalid", "依赖要求非系统安装器；不会以管理员身份运行 Python 包安装。")
            elif line.strip():
                if installer != "apt":
                    raise SystemError("dependency_invalid", "依赖解析结果缺少系统安装器标识。")
                for package in line.split():
                    if not re.fullmatch(r"[a-z0-9][a-z0-9+.-]*(?::[a-z0-9]+)?", package):
                        raise SystemError("dependency_invalid", "依赖解析返回了无效的系统软件包名称。")
                    packages.add(package)
        if groups and groups != set(keys):
            raise SystemError("dependency_invalid", "依赖解析返回的分组不完整。")
        if packages:
            self.apt(*sorted(packages))

    def sdk_install(self):
        status, installed = self.run("/usr/bin/pkg-config", "--modversion", "realsense2", check=False)
        if status == 0:
            if installed.strip() != SDK_VERSION:
                raise SystemError("sdk_conflict", "电脑已有不同版本的 RealSense SDK；不会覆盖现有环境。")
            return
        for location in ("/usr/local/include/librealsense2", "/usr/include/librealsense2"):
            if self.path(location).exists():
                raise SystemError("sdk_conflict", "发现无法确认版本的 RealSense SDK；不会覆盖现有环境。")
        _, package_status = self.run(
            "/usr/bin/dpkg-query", "-W", "-f=${db:Status-Status} ${binary:Package}\\n",
            "librealsense2*", "ros-jazzy-librealsense2*", check=False)
        if any(line.startswith("installed ") for line in package_status.splitlines()):
            raise SystemError("sdk_conflict", "已有通过系统软件包安装的 RealSense SDK；不会混装或卸载。")
        build = self.sdk / "build"
        self.user_directory(build)
        cache = (build / "CMakeCache.txt").read_text()
        if (f"CMAKE_HOME_DIRECTORY:INTERNAL={self.sdk}\n" not in cache
                or "CMAKE_INSTALL_PREFIX:PATH=/usr/local\n" not in cache
                or "FORCE_RSUSB_BACKEND:BOOL=ON\n" not in cache):
            raise SystemError("sdk_build_invalid", "SDK 构建配置与安装器要求不一致。")
        self.run("/usr/bin/cmake", "--install", str(build))
        self.run("/usr/sbin/ldconfig")
        _, installed = self.run("/usr/bin/pkg-config", "--modversion", "realsense2")
        if installed.strip() != SDK_VERSION:
            raise SystemError("sdk_version_invalid", "SDK 安装后版本校验失败。")

    def managed_file(self, path, content, *, create=False):
        """Install only missing or byte-identical root-controlled configuration."""
        for parent in [part for part in [*reversed(path.parents), path]
                       if part.is_relative_to(self.system_root)]:
            try:
                info = parent.lstat()
            except FileNotFoundError:
                continue
            if (stat.S_ISLNK(info.st_mode) or info.st_uid != self.system_uid or info.st_mode & 0o022
                    or (parent == path and not stat.S_ISREG(info.st_mode))):
                raise SystemError("system_path_unsafe", "系统配置路径的权限或类型异常。")
        if path.exists():
            if path.read_text() != content:
                raise SystemError("configuration_conflict", "已有不同内容的系统配置；不会覆盖，请维护人员确认。")
            return
        if create:
            path.parent.mkdir(parents=True, exist_ok=True, mode=0o755)
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o644)
            with os.fdopen(fd, "w") as stream:
                os.fchmod(stream.fileno(), 0o644)
                stream.write(content)

    def permissions(self):
        # Preflight all conflicts before changing group membership or USB policy.
        targets = {
            self.path("/etc/udev/rules.d/70-visiontactile-tracker-hid.rules"): TRACKER_RULE,
            self.path("/usr/local/lib/visiontactile/ensure-usbfs-memory.py"): USBFS_HELPER,
            self.path("/etc/systemd/system/visiontactile-usbfs-memory.service"): USBFS_SERVICE,
        }
        for path, content in targets.items():
            self.managed_file(path, content)
        for directory in ("/etc/udev/rules.d", "/usr/lib/udev/rules.d"):
            for path in self.path(directory).glob("*.rules"):
                if path in targets or path.name == "70-visiontactile-realsense-usb.rules":
                    continue
                text = path.read_text(errors="replace").lower().replace("\\\n", "")
                if conflicts_with_devices(text):
                    raise SystemError("udev_conflict", "已有其他规则管理相机或追踪器 USB 权限；请维护人员确认。")
        helper = self.repo / "tools/capture_deploy/configure_realsense_usb.py"
        # The existing helper performs its own rule/backup preflight and preserves
        # unlimited or larger current usbfs values. It never opens a camera.
        _, check_output = self.run("/usr/bin/python3", "-I", str(helper), "check")
        if json.loads(check_output).get("rule_conflict") is not False:
            raise SystemError("udev_conflict", "已有不同的相机 USB 规则；不会覆盖现有配置。")
        for group in ("plugdev", "dialout"):
            status, _ = self.run("/usr/bin/getent", "group", group, check=False)
            if status:
                self.run("/usr/sbin/groupadd", "--system", group)
        for path, content in targets.items():
            self.managed_file(path, content, create=True)
        self.run("/usr/bin/python3", "-I", str(helper), "apply")
        self.run("/usr/sbin/usermod", "-a", "-G", "dialout,plugdev", self.account.pw_name)
        self.run("/usr/bin/systemctl", "daemon-reload")
        self.run("/usr/bin/systemctl", "enable", "visiontactile-usbfs-memory.service")


def conflicts_with_devices(text):
    """Notice exact device rules and vendor-wide grants, ignoring comments."""
    for line in text.lower().splitlines():
        line = line.split("#", 1)[0]
        for vendor, targets in (("8086", {"0b5b", "1156"}), ("0bb4", set(TRACKER_PRODUCTS))):
            if vendor not in line:
                continue
            product = re.search(r'\{(?:idproduct|id_model_id)\}\s*==\s*"([^"]+)"', line)
            if product is None or product[1] in targets or any(char in product[1] for char in "*?[]|"):
                return True
    return False


def serve(actions, source, emit):
    for line in source:
        action = None
        try:
            if len(line) > 4096:
                raise SystemError("request_invalid", "安装请求过长。")
            request = json.loads(line)
            if not isinstance(request, dict):
                raise SystemError("request_invalid", "安装请求格式不正确。")
            action = request.get("action")
            if not isinstance(action, str) or action not in {
                "base", "ros", "deps", "sdk_install", "permissions", "close",
            }:
                raise SystemError("request_invalid", "不支持此安装操作。")
            expected = {"action", "target"} if action == "deps" else {"action"}
            if set(request) != expected or (action == "deps" and request["target"] not in ("driver", "main")):
                raise SystemError("request_invalid", "安装请求包含不支持的参数。")
            if action == "close":
                emit({"kind": "result", "action": action, "ok": True, "errorcode": None})
                return
            if action == "deps":
                actions.deps(request["target"])
            else:
                getattr(actions, action)()
            emit({"kind": "result", "action": action, "ok": True, "errorcode": None})
        except (SystemError, OSError, ValueError, KeyError) as exc:
            code = exc.code if isinstance(exc, SystemError) else "system_error"
            message = str(exc) if isinstance(exc, SystemError) else "系统安装未完成；请检查网络、目录和安装日志后重试。"
            emit({"kind": "result", "action": action if isinstance(action, str) else None,
                  "ok": False, "errorcode": code, "message": message})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", required=True, type=Path)
    parser.add_argument("--uid", required=True, type=int)
    arguments = parser.parse_args(argv)
    interrupted = [False]

    def interrupt(_signum, _frame):
        interrupted[0] = True

    def emit(event):
        try:
            print(json.dumps(event, ensure_ascii=False), flush=True)
        except BrokenPipeError:
            # The current apt/dpkg transaction is still allowed to finish.
            interrupted[0] = True

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    try:
        account, root = validate_identity(arguments.uid, arguments.repo_root)
        check_platform()
        clean_environment = system_environment()
        os.environ.clear()
        os.environ.update(clean_environment)
        os.umask(0o022)
        runner = CommandRunner(emit, lambda: interrupted[0])
        descriptor = os.open("/run/visiontactile-oneclick.lock",
                             os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        with os.fdopen(descriptor, "w") as lock:
            info = os.fstat(lock.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o077:
                raise SystemError("lock_unsafe", "系统安装锁文件权限异常。")
            try:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise SystemError("install_busy", "另一安装任务正在进行，请等待完成。") from exc
            serve(SystemActions(account, root, runner), sys.stdin, emit)
        return 0
    except (SystemError, OSError, ValueError, KeyError) as exc:
        emit({"kind": "result", "action": None, "ok": False,
              "errorcode": exc.code if isinstance(exc, SystemError) else "system_error",
              "message": str(exc) if isinstance(exc, SystemError) else "无法启动系统安装服务。"})
        return 1
