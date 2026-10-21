"""Unprivileged, resumable installation of the supported Ubuntu workstation.

Only the system broker changes system packages and permissions. Subprocesses
never inherit a developer ROS/Conda environment, and launchers are registered
only after the installed runtime has passed its software checks.
"""

from __future__ import annotations

from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import platform
import pwd
import selectors
import shutil
import signal
import stat
import subprocess
import time
import uuid

from . import plan


class InstallationError(RuntimeError):
    """An actionable message that contains no private package contents."""


class InstallationCancelled(InstallationError):
    pass


def _cancelled(cancel):
    return bool(cancel and (cancel.is_set() if hasattr(cancel, "is_set") else cancel()))


def _check_cancel(cancel):
    if _cancelled(cancel):
        raise InstallationCancelled("安装已取消；已完成的系统依赖保留，原桌面入口未提前切换。")


def clean_environment(home):
    """Keep desktop authentication, networking and locale; discard build overrides."""
    allowed = {"DISPLAY", "WAYLAND_DISPLAY", "XAUTHORITY", "DBUS_SESSION_BUS_ADDRESS",
               "XDG_RUNTIME_DIR", "LANG", "LC_ALL", "LC_CTYPE", "TERM", "http_proxy",
               "https_proxy", "no_proxy", "HTTP_PROXY", "HTTPS_PROXY", "NO_PROXY"}
    env = {key: value for key, value in os.environ.items() if key in allowed}
    env.update(HOME=str(home), PATH="/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin",
               PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1",
               PIP_CONFIG_FILE=os.devnull, PIP_DISABLE_PIP_VERSION_CHECK="1",
               GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL=os.devnull,
               GIT_TERMINAL_PROMPT="0", GIT_ASKPASS="/bin/false", CMAKE_BUILD_PARALLEL_LEVEL="2")
    return env


def _private_directory(path):
    path = Path(path).absolute()
    for part in reversed((path, *path.parents)):
        try:
            info = part.lstat()
        except FileNotFoundError:
            part.mkdir(mode=0o700)
            info = part.lstat()
        if not stat.S_ISDIR(info.st_mode):
            raise InstallationError("安装目录包含符号链接或非目录项目，请检查当前用户的数据目录。")
    info = path.stat()
    if info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise InstallationError("安装目录必须由当前用户拥有且仅当前用户可访问。")
    return path


def _record(path, value):
    temporary = path.with_name("." + path.name + "." + uuid.uuid4().hex)
    try:
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def validate_host(home):
    if os.getuid() == 0 or os.geteuid() != os.getuid():
        raise InstallationError("请以普通桌面用户运行安装器；系统步骤会单独请求管理员认证。")
    try:
        release = platform.freedesktop_os_release()
    except OSError as exc:
        raise InstallationError("无法识别系统；本安装器仅支持 Ubuntu 24.04 x86_64。") from exc
    if (platform.system() != "Linux" or platform.machine() != "x86_64"
            or release.get("ID") != "ubuntu" or release.get("VERSION_ID") != "24.04"):
        raise InstallationError("本安装器仅支持原生 Ubuntu 24.04 x86_64；未修改当前系统。")
    account = pwd.getpwuid(os.getuid())
    if Path(account.pw_dir).absolute() != home:
        raise InstallationError("安装位置必须是当前登录用户的主目录。")
    if shutil.disk_usage(home).free < 12 * 1024**3:
        raise InstallationError("软件安装至少需要 12 GiB 可用空间；录制数据还需要额外空间。")
    if not Path("/usr/bin/pkexec").is_file():
        raise InstallationError("系统缺少管理员认证组件 pkexec，请使用标准 Ubuntu 24.04 桌面环境。")
    assert_workbench_closed()


def assert_workbench_closed(proc=Path("/proc")):
    """Inspect process metadata only; never stop another application's processes."""
    markers = ("vt_capture_console", "vt_capture_console.app", "vt_capture_console.tactile_app",
               "vt_capture_manager.node", "vt_capture_manager", "vt-capture-console", "vt-tactile-3d",
               "vt_vive_tracker.node", "vt_vive_tracker_node", "realsense2_camera_node",
               "live_windows_bootstrap.py")
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            args = (entry / "cmdline").read_bytes().split(b"\0")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        for argument in args:
            value = argument.decode(errors="replace")
            if value in markers or Path(value).name in markers:
                raise InstallationError("请先结束录制并关闭工作台、后台及相机进程，再重新运行安装器。")


@contextmanager
def installation_lock(home):
    directory = _private_directory(home / ".local/share/visiontactile/installation")
    fd = os.open(directory / ".lock", os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise InstallationError("安装锁不是当前用户的私有文件。")
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise InstallationError("另一个安装器正在运行，请等待其结束。") from exc
        yield directory
    finally:
        os.close(fd)


def _stop_owned(process):
    for number, timeout in ((signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, number)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        # A completed leader may have left children holding the output pipe.
        if number == signal.SIGTERM:
            continue
        break


class Runner:
    def __init__(self, home, notify, cancel):
        self.env = clean_environment(home)
        self.notify, self.cancel = notify, cancel

    def run(self, argv, *, cwd=None, timeout=3600, check=True, quiet=False):
        _check_cancel(self.cancel)
        try:
            process = subprocess.Popen(list(map(str, argv)), cwd=cwd, env=self.env,
                                       stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, start_new_session=True, umask=0o077)
        except OSError as exc:
            raise InstallationError("无法启动安装步骤，请检查系统命令是否可用。") from exc
        output, pending = bytearray(), bytearray()
        deadline = time.monotonic() + timeout
        try:
            with selectors.DefaultSelector() as selector:
                selector.register(process.stdout, selectors.EVENT_READ)
                eof = False
                while not eof or process.poll() is None:
                    _check_cancel(self.cancel)
                    if time.monotonic() > deadline:
                        raise InstallationError("当前安装步骤超时；已停止本次安装启动的进程，可以重新运行重试。")
                    for key, _ in selector.select(0.1):
                        block = os.read(key.fd, 8192)
                        if not block:
                            selector.unregister(key.fileobj)
                            eof = True
                            continue
                        output.extend(block)
                        del output[:-131072]
                        if not quiet:
                            pending.extend(block)
                            while b"\n" in pending or len(pending) > 4096:
                                length = pending.find(b"\n")
                                length = 4096 if length < 0 or length > 4096 else length + 1
                                line = bytes(pending[:length]).decode(errors="replace").rstrip()
                                del pending[:length]
                                self.notify({"kind": "log", "text": line})
            code = process.wait()
            if pending and not quiet:
                self.notify({"kind": "log", "text": pending.decode(errors="replace").rstrip()})
            if code and check:
                raise InstallationError("当前安装步骤失败，请查看安装日志后重试；原桌面入口未提前切换。")
            return subprocess.CompletedProcess(argv, code, output.decode(errors="replace"))
        finally:
            _stop_owned(process)
            process.stdout.close()


class SystemBroker:
    """One polkit authentication; cancellation waits for the active apt operation."""
    def __init__(self, source_root, home, notify, cancel):
        self.root, self.home, self.notify, self.cancel = source_root, home, notify, cancel
        self.process = None
        self.pending = bytearray()

    def _start(self):
        if self.process is not None:
            return
        self.process = subprocess.Popen(
            ["/usr/bin/pkexec", "/usr/bin/python3", "-I",
             str(self.root / "tools/capture_deploy/install_system.py"),
             "--repo-root", str(self.root), "--uid", str(os.getuid())],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=clean_environment(self.home), start_new_session=True, umask=0o077)

    def request(self, action, **options):
        _check_cancel(self.cancel)
        self._start()
        try:
            self.process.stdin.write((json.dumps({"action": action, **options}) + "\n").encode())
            self.process.stdin.flush()
        except (OSError, BrokenPipeError) as exc:
            raise InstallationError("管理员安装进程已退出，请重新运行安装器完成系统认证。") from exc
        deferred = False
        deadline = time.monotonic() + 3 * 3600
        timed_out = False
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdout, selectors.EVENT_READ)
            while True:
                if (_cancelled(self.cancel) or time.monotonic() > deadline) and not deferred:
                    timed_out = time.monotonic() > deadline
                    deferred = True
                    self.notify({"kind": "log", "text":
                                 "正在等待当前系统配置安全结束；结束后停止后续安装，不中断 apt/dpkg。"})
                while b"\n" in self.pending:
                    line, _, rest = self.pending.partition(b"\n")
                    self.pending = bytearray(rest)
                    try:
                        event = json.loads(line)
                    except (ValueError, UnicodeError):
                        continue  # Authentication diagnostics are not package payload logs.
                    if not isinstance(event, dict):
                        continue
                    if event.get("kind") == "log":
                        self.notify({"kind": "log", "text": str(event.get("text", ""))[:4096]})
                    elif event.get("kind") == "result":
                        if not event.get("ok"):
                            raise InstallationError(event.get("message") or "系统安装步骤失败，请检查日志。")
                        if event.get("action") != action:
                            raise InstallationError("系统安装响应顺序异常，请重新运行安装器。")
                        _check_cancel(self.cancel)
                        if timed_out:
                            raise InstallationError("系统配置耗时超出预期，已在当前事务完成后停止，请重试。")
                        return event
                for key, _ in selector.select(0.1):
                    block = os.read(key.fd, 8192)
                    if not block:
                        raise InstallationError("管理员认证被取消或系统安装进程退出，请重新运行安装器。")
                    self.pending.extend(block)
                    if len(self.pending) > 262144:
                        raise InstallationError("系统安装响应异常，已停止提交新的操作。")

    def close(self):
        if self.process is None:
            return
        # EOF lets the broker exit after its current transaction. Never kill apt.
        try:
            self.process.stdin.close()
        except OSError:
            pass
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.notify({"kind": "log", "text": "系统配置仍在安全收尾，请等待完成后再运行安装器。"})
        finally:
            self.process.stdout.close()


class Installer:
    def __init__(self, delivery_dir, *, home=None, notify=lambda event: None, cancel=None):
        self.delivery_dir = Path(delivery_dir).absolute()
        self.home = Path.home().absolute() if home is None else Path(home).absolute()
        self.notify, self.cancel = notify, cancel
        self.runner = Runner(self.home, notify, cancel)
        self.broker_factory = SystemBroker

    def phase(self, phase, message):
        _check_cancel(self.cancel)
        self.notify({"kind": "progress", "phase": phase, "message": message})

    def checkout(self, source, target):
        """Reuse only the exact trusted revision, never an arbitrary local checkout."""
        _private_directory(target.parent)
        if target.exists() or target.is_symlink():
            if target.is_symlink() or (target / ".git").is_symlink() or not (target / ".git").is_dir():
                raise InstallationError("依赖源码目录已被其他内容占用，请保留原内容并联系维护者。")
            revision = self.runner.run(["/usr/bin/git", "-C", target, "rev-parse", "HEAD"], quiet=True).stdout.strip()
            clean = self.runner.run(["/usr/bin/git", "-C", target, "diff", "--no-ext-diff", "--quiet", "HEAD", "--"],
                                    check=False, quiet=True).returncode == 0
            if revision != source.commit or not clean:
                raise InstallationError("已存在的依赖源码版本不一致或被修改；安装器不会覆盖这些内容。")
            return
        temporary = target.with_name("." + target.name + "." + uuid.uuid4().hex)
        try:
            self.runner.run(["/usr/bin/git", "clone", "--depth", "1", "--single-branch", "--branch",
                             source.tag, source.url, temporary], timeout=900)
            revision = self.runner.run(["/usr/bin/git", "-C", temporary, "rev-parse", "HEAD"], quiet=True).stdout.strip()
            if revision != source.commit:
                raise InstallationError("官方依赖标签与锁定提交不一致；未继续构建。")
            os.replace(temporary, target)
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)

    def run(self):
        from .delivery import inspect_delivery, stage_delivery

        self.phase("preflight", "正在检查系统、交付包和现有工作台状态……")
        validate_host(self.home)
        python_version = self.runner.run(["/usr/bin/python3", "-I", "-c",
                                          "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"],
                                         quiet=True, timeout=10)
        if python_version.stdout.strip() != "3.12":
            raise InstallationError("系统 Python 必须为 3.12；未修改当前系统。")
        delivery = inspect_delivery(self.delivery_dir)
        with installation_lock(self.home) as record_directory:
            self.phase("stage", "正在准备独立的应用版本……")
            root = Path(stage_delivery(delivery, self.home))
            paths = plan.Paths(self.home, root)
            _private_directory(paths.dependencies)
            broker = self.broker_factory(root, self.home, self.notify, self.cancel)
            try:
                self.phase("system", "请完成一次管理员认证；正在安装系统基础依赖……")
                broker.request("base")
                self.phase("ros", "正在安装 ROS 2 Jazzy 和录制依赖……")
                broker.request("ros")
                self.phase("sdk", "正在检查并构建 RealSense SDK 2.58.1……")
                installed = self.runner.run(["/usr/bin/pkg-config", "--modversion", "realsense2"],
                                            check=False, quiet=True)
                sdk_built = installed.returncode != 0
                if installed.returncode or installed.stdout.strip() != plan.SDK_VERSION:
                    if installed.returncode == 0 and installed.stdout.strip():
                        raise InstallationError("已有其他版本的 RealSense SDK，请由维护者处理版本冲突后重试。")
                    self.checkout(plan.SDK, paths.sdk)
                    self.runner.run(plan.sdk_configure(paths))
                    self.runner.run(["/usr/bin/cmake", "--build", paths.sdk / "build", "--parallel", "2"], timeout=7200)
                    broker.request("sdk_install")
                version = self.runner.run(["/usr/bin/pkg-config", "--modversion", "realsense2"], quiet=True)
                if version.stdout.strip() != plan.SDK_VERSION:
                    raise InstallationError("RealSense SDK 安装版本核验未通过。")
                self.phase("driver", "正在构建固定版本的 RealSense ROS 驱动……")
                self.checkout(plan.WRAPPER, paths.wrapper)
                broker.request("deps", target="driver")
                self.runner.run(plan.driver_build(), cwd=paths.driver, timeout=7200)
                self.phase("workspace", "正在构建采集、回放和 Tracker 组件……")
                broker.request("deps", target="main")
                self.runner.run(plan.main_build(paths), cwd=paths.workspace, timeout=7200)
                self.phase("permissions", "正在配置相机、Tracker 和串口访问权限……")
                broker.request("permissions")
                self.phase("verify", "正在验证 ROS、驱动版本和录制运行环境……")
                self.runner.run(plan.runtime_check(paths), timeout=120)
                self.phase("console", "正在安装工作台图形环境……")
                self.runner.run(["/bin/bash", root / "tools/capture_deploy/install.sh", "--no-desktop"], timeout=3600)
                self.runner.run([paths.python, "-I", "-c",
                                 "import PySide6.QtWidgets, pyqtgraph, OpenGL, serial; "
                                 "import vt_capture_console, vt_tracker_deploy; print('Qt runtime ready')"], timeout=120)
                assert_workbench_closed()
                self.phase("tracker", "正在验证并安装对应设备的 Tracker 依赖……")
                self.runner.run([paths.python, "-I", root / "tools/tracker_deploy/manage.py", "install",
                                 "--package", root.parent / "deployment/tracker.vttracker"], timeout=900, quiet=True)
                # Tracker's atomic activation has completed. Finish the short
                # launcher commit even if Stop arrives now, avoiding a half-
                # published application. Earlier phases remain cancellable.
                self.notify({"kind": "progress", "phase": "desktop", "message":
                             "环境检查通过，正在创建工作台桌面入口……"})
                previous_cancel = getattr(self.runner, "cancel", None)
                self.runner.cancel = None
                try:
                    self.runner.run([paths.python, "-I", "-m", "vt_capture_console.desktop_entry",
                                     "--repo-root", root, "--desktop"], timeout=30)
                finally:
                    self.runner.cancel = previous_cancel
                result = {"source_root": str(root), "python": str(paths.python),
                          "launcher": str(self.home / ".local/share/applications/visiontactile-capture.desktop"),
                          "installation_id": delivery.installation_id,
                          "sdk_version": plan.SDK_VERSION,
                          "sdk_commit": plan.SDK.commit if sdk_built else None,
                          "wrapper_version": plan.WRAPPER_VERSION, "wrapper_commit": plan.WRAPPER.commit,
                          "hardware_started": False}
                _record(record_directory / "last-success.json", result)
                self.notify({"kind": "progress", "phase": "complete", "message":
                             "安装完成。请从应用菜单打开工作台并确认连接的设备；安装过程未启动硬件。"})
                return result
            finally:
                broker.close()
