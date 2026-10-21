"""Manual GUI transport for the approved external bootstrap, never HID logic.

The only stdin value is ``yes\n``, in response to the current exact prompt.
An exit code alone is never evidence of a completed three-Tracker bootstrap.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import fcntl
import hashlib
import os
from pathlib import Path
import re
import selectors
import stat
import subprocess
import threading
import time

from .environment_discovery import discover_bootstrap_configuration


COMPLETE_LINE = "status=COMPLETE failure=none context=none last_sequence=51 feature_out=51 feature_get=21 close=1"
PROMPTS = (
    "confirm=all_trackers_off type=yes",
    "confirm=ready_to_power_on role=host type=yes",
    "confirm=ready_to_power_on roles=client0,client1 type=yes",
)
ACTIONS = ("action=power_on role=host", "action=power_on roles=client0,client1")
MAX_LINE_BYTES = 4096
MAX_OUTPUT_BYTES = 256 * 1024
_ACTIVE = {"RUNNING", "WAITING", "CANCELLING"}
_LOCAL_OWNER = threading.Lock()


def _bootstrap_inputs(values):
    """Validate file metadata once for both the GUI summary and process argv."""
    paths, errors = {}, {}
    for name in ("python", "script", "capture", "bundle"):
        value = values.get(name)
        if not isinstance(value, str) or not value or len(value) > 4096 or any(c in value for c in "\0\r\n"):
            errors[name] = f"{name} 需要明确的本地文件路径"
            continue
        try:
            path = Path(value).expanduser()
            valid = path.is_absolute() and path.is_file()
        except (OSError, RuntimeError, ValueError):
            valid = False
        if not valid:
            errors[name] = f"{name} 必须是现有的绝对文件路径"
        elif name == "python" and not os.access(path, os.X_OK):
            errors[name] = "Python 文件不可执行"
        elif name == "script" and path.name != "live_windows_bootstrap.py":
            errors[name] = "只接受经批准的 live_windows_bootstrap.py 脚本"
        else:
            paths[name] = str(path)
    digest = values.get("expected_sha256")
    if not isinstance(digest, str) or re.fullmatch(r"[a-fA-F0-9]{64}", digest) is None:
        errors["expected_sha256"] = "必须明确提供经批准 capture 的 64 位 SHA-256；不会自动计算替代批准值"
    return paths, errors


def bootstrap_input_issues(values) -> tuple[str, ...]:
    """Return invalid field keys in form order; never read, hash, or run inputs."""
    return tuple(_bootstrap_inputs(values)[1])


@dataclass(frozen=True)
class BootstrapConfig:
    python: str
    script: str
    capture: str
    bundle: str
    expected_sha256: str

    def argv(self):
        paths, errors = _bootstrap_inputs({name: getattr(self, name) for name in (
            "python", "script", "capture", "bundle", "expected_sha256")})
        if errors:
            raise ValueError(next(iter(errors.values())))
        return [paths["python"], "-u", paths["script"], "--capture", paths["capture"],
                "--expected-sha256", self.expected_sha256.lower(), "--bundle", paths["bundle"],
                "--startup-mode", "host-then-clients", "--execute-feature-writes"]


def detect_defaults(repo_root=None, *, include_environment=True):
    """Inspect path existence only; never source shell or read private contents."""
    root = next((p for p in Path(__file__).resolve().parents if (p / "docs/tracker-ros2-publisher.md").is_file()), None)
    root = Path(repo_root if repo_root is not None else os.environ.get("VT_REPO", root or Path.home())).expanduser().absolute()
    # Private inputs come only from an explicit saved/environment choice or a
    # coherent deployment manifest, never a guessed historical filename.
    values = dict(python="", script="", capture="", bundle="", expected_sha256="")
    values = discover_bootstrap_configuration(values, repo_root=root, environ={}).configuration
    if include_environment:
        values.update(bootstrap_environment_overrides())
    return values


def bootstrap_environment_overrides():
    """Explicit environment entries stay visible even if paths need repair."""
    result = {}
    for variable, name in (("VT_CAPTURE", "capture"), ("VT_BUNDLE", "bundle"),
                           ("VT_CAPTURE_SHA256", "expected_sha256")):
        if variable in os.environ:
            result[name] = os.environ[variable]
    for variable, name, suffix in (("VT_PYVUT_ROOT", "script", "scripts/live_windows_bootstrap.py"),
                                  ("VT_PYVUT_VENV", "python", "bin/python")):
        if variable in os.environ:
            value = os.environ[variable]
            result[name] = str(Path(value).expanduser() / suffix) if value else ""
    return result


def initialization_blocker(snapshot):
    """Fail closed if recording/publisher ownership is unavailable or unresolved."""
    if not isinstance(snapshot, dict) or snapshot.get("mode") != "live" or not snapshot.get("instance_id"):
        return "需要真实后台的新鲜快照；演示或失联状态不能初始化硬件。"
    if snapshot.get("recording", {}).get("state") not in {"UNAVAILABLE", "IDLE", "COMPLETE"}:
        return "录制正在进行或状态未确认；不能初始化 Tracker。"
    management = snapshot.get("management") or {}
    controller = management.get("controller")
    if controller is not None and (controller.get("recorder_alive") is not False or controller.get("state") not in (0, 5, 6)):
        return "需要先确认录制控制器空闲，且不存在未解决的录制进程。"
    if any(row.get("state") in {"STARTING", "RECORDING", "STOPPING", "UNKNOWN"} for row in management.get("trials", [])):
        return "仍有活动或待核实试次；先解决录制状态。"
    control = snapshot.get("process_control") or {}
    trackers = [g for g in control.get("groups", []) if g.get("kind") == "tracker"]
    if not control.get("enabled") or not snapshot.get("capabilities", {}).get("device_control") or not trackers:
        return "Tracker 进程归属不可确认；请先连接具有设备管理能力的后台。"
    if any(g.get("state") not in {"STOPPED", "FAILED"} or g.get("owned") or g.get("pid") is not None
           or not g.get("can_start") for g in trackers):
        return "Tracker 发布器仍在运行、由外部占用或归属未知；请先明确停止发布器。"
    if any(d.get("kind") == "tracker" and (d.get("status") == "online" or d.get("receive_hz", 0) > 0)
           for d in snapshot.get("devices", [])):
        return "仍观察到 Tracker 数据；不能与发布器同时初始化。"
    return ""


def capture_sha256(path, cancel=lambda: False):
    """An explicit local-file digest, not approval of a capture's provenance."""
    # O_NONBLOCK prevents selecting a FIFO from hanging a file-digest worker.
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_NONBLOCK", 0))
    with os.fdopen(descriptor, "rb") as stream:
        before = os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode) or before.st_size > 8 * 1024**3:
            raise ValueError("Capture 必须是最多 8 GiB 的常规文件")
        digest, started = hashlib.sha256(), time.monotonic()
        while True:
            if cancel():
                raise ValueError("指纹计算已取消")
            if time.monotonic() - started > 120:
                raise ValueError("指纹计算超时")
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
        after = os.fstat(stream.fileno())
        identity = lambda value: (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns)
        if identity(before) != identity(after) or identity(after) != identity(os.stat(path)):
            raise ValueError("Capture 在指纹计算期间变化，请重新确认文件")
        return digest.hexdigest()


class TrackerWorkflow:
    """A bounded background child with explicit start/answer/cancel, no retries."""
    def __init__(self, snapshot_provider, *, admission_path=None, fresh_snapshot_provider=None, process_factory=None,
                 timeout_s=900., terminate_grace_s=1.):
        if not 0 < timeout_s <= 3600 or not 0 < terminate_grace_s <= 5:
            raise ValueError("Invalid bootstrap timeout/grace bounds")
        self._provider = snapshot_provider
        self._fresh_provider = fresh_snapshot_provider or snapshot_provider
        self._factory = process_factory or subprocess.Popen
        self._admission_path = Path(admission_path).absolute() if admission_path is not None else None
        self._timeout, self._grace = timeout_s, terminate_grace_s
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._thread = self._process = None
        self._state, self._detail, self._prompt = "IDLE", "尚未启动；不会自动访问设备。", ""
        self._phase, self._instance = 0, ""
        self._logs = deque(maxlen=60)
        self._failure = ""
        self._complete = False

    def snapshot(self):
        with self._lock:
            return dict(state=self._state, detail=self._detail, prompt=self._prompt, logs=list(self._logs))

    def _guard(self, *, fresh=False):
        snapshot = (self._fresh_provider if fresh else self._provider)()
        problem = initialization_blocker(snapshot)
        if not problem and self._instance and snapshot["instance_id"] != self._instance:
            problem = "后台实例已变化；本次初始化停止，不能自动继续。"
        if problem:
            raise ValueError(problem)
        return snapshot

    def _acquire_admission(self):
        if self._admission_path is None:
            return None
        fd = os.open(self._admission_path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
                raise ValueError("拒绝非私有的桌面初始化门禁锁")
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise ValueError("后台正执行其他设备或配置操作；初始化未启动") from exc
            return fd
        except Exception:
            os.close(fd)
            raise

    def start(self, config):
        argv = config.argv()
        with self._lock:
            if self._state in _ACTIVE or self._thread is not None and self._thread.is_alive():
                raise ValueError("初始化已经运行；不会重复启动")
            self._instance = ""
            snapshot = self._guard()
            if not _LOCAL_OWNER.acquire(blocking=False):
                raise ValueError("本工作台已有 Tracker 初始化流程")
            admission = None
            try:
                admission = self._acquire_admission()
                # Cached UI state is checked again here. The actual fresh IPC
                # check occurs under this same lock in _run, off the Qt thread.
                snapshot = self._guard()
                self._instance = snapshot["instance_id"]
                self._state, self._detail, self._prompt = "RUNNING", "正在验证输入，等待脚本的明确提示。", ""
                self._phase, self._complete, self._failure = 0, False, ""
                self._logs.clear()
                self._cancel.clear()
                self._thread = threading.Thread(target=self._run, args=(argv, admission), name="tracker-manual-bootstrap", daemon=True)
                self._thread.start()
            except Exception:
                self._state = "FAILED"
                if admission is not None:
                    os.close(admission)
                _LOCAL_OWNER.release()
                raise

    def answer(self, prompt):
        with self._lock:
            if self._state != "WAITING" or not self._prompt or prompt != self._prompt or self._cancel.is_set():
                raise ValueError("只能确认当前明确提示；不能预先发送或重复确认")
            try:
                self._guard()
                if self._process is None or self._process.poll() is not None:
                    raise ValueError("初始化进程已退出")
                self._process.stdin.write(b"yes\n")
                self._process.stdin.flush()
                self._phase += 1
                self._state, self._prompt = "RUNNING", ""
                self._detail = "确认已发送；等待脚本提示，不要提前打开 Tracker。"
            except Exception as exc:
                self._failure = str(exc)
                self._cancel.set()
                raise ValueError(str(exc)) from exc

    def cancel(self):
        with self._lock:
            if self._state in _ACTIVE:
                self._cancel.set()
                self._state, self._prompt = "CANCELLING", ""
                self._detail = "仅停止本次初始化子进程；已发生的设备写入不会回滚。"

    def wait(self, timeout=2.):
        if self._thread:
            self._thread.join(timeout)
        return self._thread is None or not self._thread.is_alive()

    def _line(self, line):
        with self._lock:
            if line in PROMPTS:
                expected_phase = (0, 1, 3)[PROMPTS.index(line)]
                if self._phase != expected_phase or self._prompt:
                    raise ValueError("脚本提示顺序与已批准流程不一致；未发送任何额外回答")
                self._prompt, self._state = line, "WAITING"
                self._detail = ("请确认三个 Tracker 均已关闭。", "准备固定 Host；先确认准备就绪，等待开机指令。",
                                "准备固定 Client 0 与 Client 1；先确认准备就绪，等待同时开机指令。")[PROMPTS.index(line)]
            elif line in ACTIONS:
                expected_phase = (2, 4)[ACTIONS.index(line)]
                if self._phase != expected_phase or self._prompt:
                    raise ValueError("脚本开机提示顺序异常")
                self._phase += 1
                self._detail = ("现在打开固定 Host；保持两个 Client 关闭，等待下一提示。" if line == ACTIONS[0]
                                else "现在同时打开固定 Client 0 与 Client 1；等待完整握手结果。")
            elif line == COMPLETE_LINE:
                if self._phase != 5 or self._prompt or self._complete:
                    raise ValueError("未完成全部明确操作，不能接受 COMPLETE")
                self._complete = True
                self._detail = "脚本报告 COMPLETE；等待进程正常退出后再确认完成。"
            elif line.startswith(("confirm=", "action=", "status=")):
                raise ValueError("脚本未完成或提示契约不匹配：" + line[:500])
            else:
                # Do not relay arbitrary subprocess text: private inputs and
                # credentials must never become a generic terminal transcript.
                return
            self._logs.append(line)

    def _run(self, argv, admission):
        process = selector = None
        try:
            self._guard(fresh=True)
            if self._cancel.is_set():
                return
            process = self._factory(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    shell=False, start_new_session=True, bufsize=0,
                                    pass_fds=(() if admission is None else (admission,)))
            with self._lock:
                self._process = process
            selector = selectors.DefaultSelector()
            selector.register(process.stdout, selectors.EVENT_READ)
            started, last_guard, total, pending = time.monotonic(), 0., 0, bytearray()
            while selector.get_map() or process.poll() is None:
                if self._cancel.is_set():
                    break
                now = time.monotonic()
                if now - started >= self._timeout:
                    raise ValueError("初始化等待超时；不会自动重试")
                if now - last_guard >= .2:
                    self._guard()
                    last_guard = now
                for key, _ in selector.select(.05):
                    chunk = os.read(key.fileobj.fileno(), 4096)
                    if not chunk:
                        selector.unregister(key.fileobj)
                        continue
                    total += len(chunk)
                    pending.extend(chunk)
                    if total > MAX_OUTPUT_BYTES:
                        raise ValueError("初始化输出超过安全上限")
                    while b"\n" in pending:
                        line, _, tail = pending.partition(b"\n")
                        pending = bytearray(tail)
                        if len(line) > MAX_LINE_BYTES:
                            raise ValueError("初始化输出行超过安全上限")
                        self._line(line.decode("utf-8", errors="strict").rstrip("\r"))
                    if len(pending) > MAX_LINE_BYTES:
                        raise ValueError("初始化输出行超过安全上限")
            if not self._cancel.is_set() and (pending or process.returncode != 0 or not self._complete):
                raise ValueError("未取得精确 COMPLETE 和正常退出；初始化未完成，禁止自动重试。")
        except Exception as exc:
            self._failure = str(exc)[:1000]
        finally:
            if process is not None:
                try:
                    if process.poll() is None:
                        process.terminate()  # Only this exact owned Popen handle.
                        try:
                            process.wait(timeout=self._grace)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=self._grace)
                except (OSError, subprocess.TimeoutExpired) as exc:
                    self._failure = "子进程退出未确认，请人工核实：" + str(exc)[:400]
                finally:
                    process.stdin.close()
                    process.stdout.close()
            if selector:
                selector.close()
            # Publish COMPLETE only after the initialization child has exited
            # and its admission lock is released. The GUI can then request the
            # read-only publisher without racing the just-finished bootstrap.
            if admission is not None:
                os.close(admission)
            _LOCAL_OWNER.release()
            with self._lock:
                self._process, self._prompt = None, ""
                if self._failure:
                    self._state, self._detail = "FAILED", self._failure
                elif self._cancel.is_set():
                    self._state, self._detail = "CANCELLED", "初始化已取消；可能已部分写入，需人工核实，不会自动续跑。"
                elif self._complete:
                    self._state, self._detail = "COMPLETE", "初始化完整完成。随后可手动启动只读发布器；本操作没有启动发布器。"
                else:
                    self._state, self._detail = "FAILED", "初始化未完成。"
