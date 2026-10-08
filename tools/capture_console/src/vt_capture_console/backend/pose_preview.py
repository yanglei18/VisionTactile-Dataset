"""Asynchronous newest-only MANO display reconstruction, never pose solving.

One persistent child per hand owns CPU model inference. GUI tiles and enlarged
views share it; they never import torch or acquire hardware. A request identity
and generation fence prevent late replies from replacing cleared/current data.
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
from PySide6.QtCore import QCoreApplication, QObject, QProcess, QTimer, Signal
from shiboken6 import isValid

from ..ui.widgets.tactile_model_loader import model_loader, validate_bundle

MAX_OUTPUT_BYTES = 512 * 1024
MAX_INPUT_BYTES = 32 * 1024
MAX_ERROR_BYTES = 8192


def key_for(mano):
    if not isinstance(mano, dict) or mano.get("hand") not in ("right", "left"):
        raise ValueError("MANO preview requires an explicit hand")
    strings = ("session_id", "model_rest_sha256", "config_sha256")
    for name in strings:
        if not isinstance(mano.get(name), str) or len(mano[name]) > 256:
            raise ValueError(f"MANO preview {name} must be a bounded string")
    for name in ("calibration_version", "frame_index", "source_realtime_ns"):
        if type(mano.get(name)) is not int or not 0 <= mano[name] <= 2**64 - 1:
            raise ValueError(f"MANO preview {name} must be a nonnegative integer")
    if mano.get("source_mode") not in ("live", "replay"):
        raise ValueError("MANO preview source must be live or replay")
    return tuple(mano[name] for name in ("hand", "session_id", "calibration_version", "frame_index",
        "source_realtime_ns", "model_rest_sha256", "config_sha256", "source_mode"))


def _finite_values(value, count, name):
    if (not isinstance(value, (list, tuple)) or len(value) != count
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
        raise ValueError(f"MANO preview {name} must contain {count} finite numbers")
    return [float(v) for v in value]


def _request(mano, joints, hand):
    key = key_for(mano)
    if mano["hand"] != hand:
        raise ValueError("MANO preview hand differs from renderer")
    clean = dict(zip(("hand", "session_id", "calibration_version", "frame_index", "source_realtime_ns",
                     "model_rest_sha256", "config_sha256", "source_mode"), key))
    clean.update(pose=_finite_values(mano.get("pose"), 48, "pose"),
                 shape=_finite_values(mano.get("shape"), 10, "shape"))
    if joints is not None:
        if not isinstance(joints, (list, tuple)) or len(joints) != 21:
            raise ValueError("MANO preview joints must contain 21 positions")
        joints = [_finite_values(point, 3, "joint") for point in joints]
    return key, clean, joints


def _worker_arguments(hand, assets):
    return ["-I", "-m", "vt_fsglove12.mano_pose_preview", "--mano-assets", assets, "--hand", hand]


def _geometry(value):
    if not isinstance(value, dict):
        raise ValueError("MANO geometry is missing")
    result = {}
    for name, shape in (("vertices_m", (778, 3)), ("joints_m", (21, 3))):
        array = np.asarray(value.get(name))
        if (array.shape != shape or array.dtype.kind not in "fiu" or not np.isfinite(array).all()
                or np.any(np.abs(array) > 2.)):
            raise ValueError("MANO geometry must contain finite metre-scale vertices and joints")
        array = np.array(array, dtype=float, copy=True)
        array.setflags(write=False)
        result[name] = array
    return result


class PosePreview(QObject):
    initialized = Signal(object)
    ready = Signal(object)
    failed = Signal(str)

    def __init__(self, hand, configuration_provider=None, parent=None, *, init_timeout_ms=60_000,
                 frame_timeout_ms=5000):
        super().__init__(parent)
        if hand not in ("right", "left"):
            raise ValueError("preview hand must be right or left")
        self.hand = hand
        self._settings = model_loader(configuration_provider)
        self._init_timeout_ms, self._frame_timeout_ms = init_timeout_ms, frame_timeout_ms
        self._process = None
        self._retiring = set()
        self._config_key = self._fatal_key = None
        self._bundle = self._geometry = None
        self._pending = self._inflight = self._desired_key = None
        self._generation = self._serial = 0
        self._output, self._errors = bytearray(), bytearray()
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(lambda: self._fail("MANO 显示进程超时；请重新加载模型"))
        self._closed = False
        app = QCoreApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.shutdown)

    @property
    def bundle(self):
        return self._bundle

    @property
    def geometry(self):
        """Latest accepted {'key', 'geometry'} result, or None after clear."""
        return self._geometry

    def clear(self):
        self._generation += 1
        self._pending = self._desired_key = self._geometry = None

    def retry(self):
        self.clear()
        self._fatal_key = None
        self._retire()

    def submit(self, mano, joints=None):
        if self._closed:
            return None
        try:
            key, clean, points = _request(mano, joints, self.hand)
            configuration = self._settings._configuration(self.hand)
        except (ValueError, TypeError, OSError, OverflowError, AttributeError) as exc:
            self.clear()
            self.failed.emit(str(exc))
            return None
        if configuration != self._config_key:
            self.clear()
            self._retire()
            self._config_key = configuration
            self._fatal_key = None
        if self._fatal_key == configuration:
            return key
        if key == self._desired_key:
            return key
        self._desired_key = key
        self._pending = (key, clean, points, self._generation)
        if self._process is None:
            self._start(configuration)
        else:
            self._dispatch()
        return key

    def _start(self, configuration):
        self._bundle = None
        self._output.clear()
        self._errors.clear()
        process = self._process = QProcess(self)
        process.setProcessChannelMode(QProcess.ProcessChannelMode.SeparateChannels)
        process.readyReadStandardOutput.connect(lambda: self._read(process))
        process.readyReadStandardError.connect(lambda: self._read_errors(process))
        process.finished.connect(lambda code, status: self._finished(process, code, status))
        process.errorOccurred.connect(lambda error: self._process_error(process))
        process.start(configuration[0], _worker_arguments(self.hand, configuration[1]))
        self._timer.start(self._init_timeout_ms)

    def _dispatch(self):
        if self._process is None or self._bundle is None or self._inflight or not self._pending:
            return
        key, clean, points, generation = self._pending
        self._pending = None
        self._serial += 1
        payload = json.dumps(dict(id=self._serial, mano=clean, joints=points),
                             allow_nan=False, separators=(",", ":")).encode() + b"\n"
        if len(payload) > MAX_INPUT_BYTES:
            self._fail("MANO 位姿请求过大")
            return
        self._inflight = (self._serial, key, generation)
        if self._process.write(payload) != len(payload):
            self._fail("MANO 位姿显示进程无法接收数据")
            return
        self._timer.start(self._frame_timeout_ms)

    def _read_errors(self, process):
        if not isValid(self) or not isValid(process):
            return
        data = bytes(process.readAllStandardError())
        if process is self._process:
            self._errors.extend(data[-MAX_ERROR_BYTES:])
            del self._errors[:-MAX_ERROR_BYTES]

    def _read(self, process):
        if not isValid(self) or not isValid(process):
            return
        data = bytes(process.readAllStandardOutput())
        if process is not self._process:
            return
        self._output.extend(data)
        if len(self._output) > MAX_OUTPUT_BYTES:
            self._fail("MANO 显示进程输出过大")
            return
        while b"\n" in self._output and process is self._process:
            line, _, remainder = self._output.partition(b"\n")
            self._output = bytearray(remainder)
            try:
                self._message(json.loads(line))
            except (ValueError, TypeError, KeyError, OverflowError, RecursionError, OSError, AttributeError) as exc:
                self._fail("MANO 显示结果无效：" + str(exc))

    def _message(self, value):
        if not isinstance(value, dict):
            raise ValueError("invalid message type")
        if self._settings._configuration(self.hand) != self._config_key:
            self._fail("MANO 配置已变化；请重新加载模型")
            return
        if value.get("type") == "ready":
            if self._bundle is not None:
                raise ValueError("duplicate model initialization")
            self._bundle = validate_bundle(value.get("bundle"), self.hand)
            self._timer.stop()
            self.initialized.emit(self._bundle)
            self._dispatch()
            return
        inflight = self._inflight
        if not inflight or type(value.get("id")) is not int or value["id"] != inflight[0]:
            raise ValueError("unexpected pose response identity")
        self._timer.stop()
        self._inflight = None
        current = inflight[1] == self._desired_key and inflight[2] == self._generation
        if value.get("type") == "geometry":
            geometry = _geometry(value.get("geometry"))
            if current:
                self._geometry = {"key": inflight[1], "geometry": geometry}
                self.ready.emit(self._geometry)
        elif value.get("type") == "error":
            if current:
                self._geometry = None
                self.failed.emit(str(value.get("message", "MANO 位姿显示失败"))[:1024])
        else:
            raise ValueError("unknown pose response")
        self._dispatch()

    def _process_error(self, process):
        if isValid(self) and isValid(process) and not self._closed and process is self._process:
            self._fail("MANO 显示进程无法运行：" + process.errorString())

    def _finished(self, process, code, status):
        # Parent destruction can invalidate a wrapper before an already queued
        # finished callback runs. Never invoke Qt methods on that stale wrapper.
        if not isValid(self):
            return
        if not self._closed and isValid(process) and process is self._process:
            self._read_errors(process)
            detail = self._errors.decode("utf-8", errors="replace").strip()
            self._fail("MANO 显示进程已退出；请重新加载模型" + ("\n" + detail if detail else ""))
        self._dispose_process(process)

    def _dispose_process(self, process):
        self._retiring.discard(process)
        if isValid(process):
            process.deleteLater()

    def _retire(self):
        self._timer.stop()
        process, self._process = self._process, None
        self._inflight = None
        self._bundle = None
        if process is not None and isValid(process):
            self._retiring.add(process)
            if process.state() != QProcess.ProcessState.NotRunning:
                process.kill()
            else:
                self._dispose_process(process)

    def _fail(self, message):
        if self._closed or not isValid(self):
            return
        self._fatal_key = self._config_key
        self.clear()
        self._retire()
        self.failed.emit(message)

    def shutdown(self):
        """Reap model-only children before QApplication destroys their owners.

        Normal rendering/reconfiguration stays fully asynchronous. Explicit
        shutdown gives stdin EOF 150 ms to finish normally, then kills and
        waits up to another 600 ms *total* for any owned/retiring children. This
        bounded wait also works after the application's event loop has stopped.
        It never signals a sensor process or waits for an external process.
        """
        if not isValid(self):
            return
        self._closed = True
        self.clear()
        self._timer.stop()
        process, self._process = self._process, None
        self._inflight = self._bundle = None
        if process is not None and isValid(process):
            self._retiring.add(process)
            if process.state() != QProcess.ProcessState.NotRunning:
                process.closeWriteChannel()
        processes = tuple(self._retiring)
        self._wait_for_children(processes, time.monotonic() + .15)
        for child in processes:
            if isValid(child) and child.state() != QProcess.ProcessState.NotRunning:
                child.kill()
        self._wait_for_children(processes, time.monotonic() + .6)
        for child in processes:
            if not isValid(child) or child.state() == QProcess.ProcessState.NotRunning:
                self._dispose_process(child)

    @staticmethod
    def _wait_for_children(processes, deadline):
        for child in processes:
            remaining_ms = max(0, int((deadline - time.monotonic()) * 1000))
            if remaining_ms and isValid(child) and child.state() != QProcess.ProcessState.NotRunning:
                child.waitForFinished(remaining_ms)


def pose_preview(hand, configuration_provider=None):
    app = QCoreApplication.instance()
    if app is None:
        raise RuntimeError("MANO preview requires a Qt application")
    previews = getattr(app, "_vt_pose_previews", None)
    if previews is None:
        previews = app._vt_pose_previews = {}
    if configuration_provider is not None:
        model_loader(configuration_provider)
    if hand not in previews or previews[hand]._closed:
        previews[hand] = PosePreview(hand, configuration_provider, app)
    return previews[hand]
