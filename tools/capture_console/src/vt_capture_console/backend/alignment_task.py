"""One owned, cancellable offline child with bounded output; no ROS/Qt imports."""
from __future__ import annotations

from copy import deepcopy
import os
from pathlib import Path
import selectors
import subprocess
import threading
import time


MAX_LOG_BYTES = 128 * 1024


def recording_blocker(snapshot):
    if not isinstance(snapshot, dict):
        return ""
    management = snapshot.get("management") or {}
    controller = management.get("controller") or {}
    if (snapshot.get("recording", {}).get("state") == "RECORDING" or controller.get("recorder_alive")
            or any(t.get("state") in {"STARTING", "RECORDING", "STOPPING", "UNKNOWN"}
                   for t in management.get("trials", []))):
        return "录制正在进行或尚未确认结束；请先停止录制，再运行离线工具。"
    return ""


def _path(value, label):
    if not isinstance(value, str) or not value.strip() or any(c in value for c in "\0\r\n"):
        raise ValueError(f"请选择{label}")
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise ValueError(f"{label}必须是绝对路径")
    return path


def build_alignment_command(repo_root, action, *, bag="", config="", extrinsics="", output=""):
    if action not in {"inspect", "align", "validate"}:
        raise ValueError("仅支持检查、对齐与验证")
    wrapper = Path(repo_root).resolve() / "tools/capture_deploy/run_alignment.sh"
    if not wrapper.is_file():
        raise ValueError("离线工具入口缺失，请联系工作站维护人员")
    argv = ["/bin/bash", str(wrapper), action]
    bag_path = None
    if action in {"inspect", "align"}:
        bag_path = _path(bag, "bag 目录").resolve()
        config_path = _path(config, "对齐配置文件").resolve()
        if not bag_path.is_dir() or not (bag_path / "metadata.yaml").is_file():
            raise ValueError("bag 目录必须包含 metadata.yaml；不会访问正在录制的数据")
        if not config_path.is_file():
            raise ValueError("对齐配置文件不存在")
        argv += ["--bag", str(bag_path), "--config", str(config_path)]
    result_path = ""
    if action == "align":
        extrinsics_path = _path(extrinsics, "外参目录").resolve()
        if not extrinsics_path.is_dir():
            raise ValueError("外参目录不存在")
        target = _path(output, "新的结果目录")
        if target.exists() or target.is_symlink():
            raise ValueError("结果目录已存在；请选择新目录，不会覆盖原结果")
        target = target.resolve()
        if target.is_relative_to(bag_path):
            raise ValueError("结果不能写入源 bag 内；请选择独立目录")
        if not target.parent.is_dir():
            raise ValueError("结果目录的父目录不存在，请先选择已有父目录")
        argv += ["--extrinsics", str(extrinsics_path), "--output", str(target)]
        result_path = str(target)
    elif action == "validate":
        target = _path(output, "已有结果目录").resolve()
        if not target.is_dir():
            raise ValueError("待验证的结果目录不存在")
        argv += ["--output", str(target)]
        result_path = str(target)
    return argv, result_path


class AlignmentTask:
    def __init__(self, repo_root, *, snapshot_provider=None):
        self.repo_root = Path(repo_root)
        self.snapshot_provider = snapshot_provider
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._thread = None
        self._status = dict(state="IDLE", detail="请选择已结束的 bag 或已有结果。", exit_code=None,
                            output_path="", log="")
        self._log = bytearray()

    @property
    def status(self):
        with self._lock:
            return deepcopy(self._status)

    @property
    def running(self):
        return self.status["state"] in {"RUNNING", "CANCELLING"}

    def _update(self, **values):
        with self._lock:
            self._status.update(values)

    def _append(self, data):
        with self._lock:
            self._log.extend(data)
            truncated = len(self._log) > MAX_LOG_BYTES
            if truncated:
                del self._log[:-MAX_LOG_BYTES]
            prefix = "（日志过长，仅保留最近 128 KiB）\n" if truncated else ""
            self._status["log"] = prefix + self._log.decode("utf-8", errors="replace")

    def start(self, action, **paths):
        if self.running:
            raise RuntimeError("已有离线任务运行中")
        argv, result = build_alignment_command(self.repo_root, action, **paths)
        if self.snapshot_provider:
            problem = recording_blocker(self.snapshot_provider())
            if problem:
                raise ValueError(problem)
        self._cancel = threading.Event()
        self._log = bytearray()
        self._update(state="RUNNING", detail="离线任务运行中；不修改源 bag。", exit_code=None,
                     output_path=result, log="")
        # Non-daemon: after an explicit close/cancel, interpreter shutdown also
        # allows the owned child its bounded termination grace period.
        self._thread = threading.Thread(target=self._run, args=(argv,), name="offline-alignment", daemon=False)
        self._thread.start()

    def cancel(self):
        with self._lock:
            if self._status["state"] in {"RUNNING", "CANCELLING"}:
                self._cancel.set()
                self._status.update(state="CANCELLING", detail="正在取消本窗口的离线子进程；不会停止采集设备。")

    def _run(self, argv):
        process = None
        selector = selectors.DefaultSelector()
        try:
            if self.snapshot_provider:
                problem = recording_blocker(self.snapshot_provider())
                if problem:
                    raise ValueError(problem)
            if self._cancel.is_set():
                self._update(state="CANCELLED", detail="任务已取消，未启动子进程。")
                return
            process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            os.set_blocking(process.stdout.fileno(), False)
            selector.register(process.stdout, selectors.EVENT_READ)
            cancel_at = None
            finished_at = None
            while True:
                if self._cancel.is_set() and process.poll() is None:
                    if cancel_at is None:
                        cancel_at = time.monotonic()
                        process.terminate()  # Only this Popen child, never a discovered PID.
                    elif time.monotonic() - cancel_at >= 1.5:
                        process.kill()
                for key, _ in selector.select(0.05):
                    chunk = os.read(key.fd, 65536)
                    if chunk:
                        self._append(chunk)
                    else:
                        selector.unregister(key.fileobj)
                code = process.poll()
                if code is not None:
                    finished_at = finished_at or time.monotonic()
                    if not selector.get_map() or time.monotonic() - finished_at > 0.25:
                        break
            if self._cancel.is_set():
                self._update(state="CANCELLED", exit_code=code,
                             detail="已取消。未修改源 bag；如工具留下部分结果，请选择新目录重试。")
            elif code == 0:
                self._update(state="COMPLETE", exit_code=code, detail="任务完成。检查项与质量结论请查看日志和结果文件。")
            elif code == 2:
                self._update(state="FAILED", exit_code=code, detail="工具已结束，但结果未通过判定；请查看日志，不应视为质量通过。")
            else:
                self._update(state="FAILED", exit_code=code, detail=f"工具执行失败（退出码 {code}）；请查看日志。")
        except Exception as exc:
            self._append((str(exc) + "\n").encode("utf-8", errors="replace"))
            self._update(state="FAILED", detail=f"离线任务未完成：{str(exc)[:500]}")
        finally:
            selector.close()
            if process is not None:
                if process.poll() is None:
                    process.kill()
                process.wait()  # Worker only; the Qt thread never waits.
                if process.stdout:
                    process.stdout.close()
