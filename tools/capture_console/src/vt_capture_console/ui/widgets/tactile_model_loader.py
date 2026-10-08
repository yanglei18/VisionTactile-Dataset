"""Asynchronous, app-shared static MANO geometry loading; no device ownership.

PyTorch/manotorch stay in the configured glove Python subprocess. Geometry is
cached only in this application's memory and never written to a mesh asset.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from pathlib import Path

import numpy as np
from PySide6.QtCore import QCoreApplication, QObject, QProcess, QTimer, Signal


MAX_OUTPUT_BYTES = 512 * 1024
MAX_ERROR_BYTES = 8192
LOAD_TIMEOUT_MS = 60_000
_AXES = ("pitch_axis", "roll_axis", "palm_up_axis")


def _saved_configuration():
    from ...backend.workstation_settings import WorkstationSettings

    return WorkstationSettings().configuration


def _worker_arguments(hand, assets):
    return ["-I", "-m", "vt_fsglove12.tactile_preview_export", "--mano-assets", assets, "--hand", hand]


def validate_bundle(value, hand):
    """Bound and sanitize the worker's JSON before exposing it to OpenGL."""
    if hand not in ("right", "left"):
        raise ValueError("3D 模型手别必须为 right 或 left")
    if (not isinstance(value, dict) or type(value.get("schema_version")) is not int
            or value["schema_version"] != 1):
        raise ValueError("不支持的 3D 模型数据版本")
    geometry, axes = value.get("neutral_geometry"), value.get("model_axes")
    if not isinstance(geometry, dict) or not isinstance(axes, dict):
        raise ValueError("3D 模型缺少中立几何或坐标轴")
    provenance = geometry.get("provenance")
    if not isinstance(provenance, dict) or provenance.get("hand") != hand or axes.get("hand") != hand:
        raise ValueError("3D 模型手别与请求不一致")

    def finite_array(data, shape, label):
        array = np.asarray(data)
        if array.shape != shape or array.dtype.kind not in "fiu" or not np.isfinite(array).all():
            raise ValueError(f"3D 模型 {label} 的形状或数值无效")
        result = np.array(array, dtype=np.float64, copy=True)
        if np.any(np.abs(result) > 2.):
            raise ValueError(f"3D 模型 {label} 超出米制手部范围")
        result.setflags(write=False)
        return result

    vertices = finite_array(geometry.get("vertices_m"), (778, 3), "vertices_m")
    joints = finite_array(geometry.get("joints_m"), (21, 3), "joints_m")
    faces = np.asarray(geometry.get("faces"))
    if (faces.ndim != 2 or faces.shape[1:] != (3,) or not 1 <= len(faces) <= 4096
            or faces.dtype.kind not in "iu" or np.any(faces < 0) or np.any(faces >= len(vertices))):
        raise ValueError("3D 模型三角面索引无效")
    faces = np.array(faces, dtype=np.int64, copy=True)
    triangles = vertices[faces]
    if np.any(np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                     triangles[:, 2] - triangles[:, 0]), axis=1) <= 1e-14):
        raise ValueError("3D 模型包含退化三角面")
    faces.setflags(write=False)
    basis = np.stack([finite_array(axes.get(name), (3,), name) for name in _AXES])
    if not np.allclose(basis @ basis.T, np.eye(3), atol=1e-4):
        raise ValueError("3D 模型坐标轴必须正交且归一化")
    expected_up = (1 if hand == "right" else -1) * np.cross(basis[1], basis[0])
    if not np.allclose(expected_up, basis[2], atol=1e-4):
        raise ValueError("3D 模型掌面方向与手别不一致")
    return {"schema_version": 1,
            "neutral_geometry": {"vertices_m": vertices, "faces": faces, "joints_m": joints,
                                 "provenance": {"hand": hand}},
            "model_axes": {**{name: basis[index].tolist() for index, name in enumerate(_AXES)},
                           "hand": hand, "source": f"{hand}_neutral_mano_21_joints"}}


@dataclass
class _Job:
    hand: str
    key: tuple
    process: QProcess
    timer: QTimer
    output: bytearray = field(default_factory=bytearray)
    errors: bytearray = field(default_factory=bytearray)


class TactileModelLoader(QObject):
    ready = Signal(str, object)
    failed = Signal(str, str)

    def __init__(self, configuration_provider=None, parent=None, *, timeout_ms=LOAD_TIMEOUT_MS):
        super().__init__(parent)
        self._configuration_provider = configuration_provider or _saved_configuration
        self._timeout_ms = timeout_ms
        self._cache = {}
        self._errors = {}
        self._jobs = {}
        self._closed = False
        app = QCoreApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.shutdown)

    def set_configuration_provider(self, provider):
        self._configuration_provider = provider or _saved_configuration

    def _configuration(self, hand):
        if hand not in ("right", "left"):
            raise ValueError("3D 模型手别必须为 right 或 left")
        configuration = self._configuration_provider()
        profile = next((item for item in configuration.get("process_groups", ())
                        if item.get("id") == hand + "_glove"), None)
        parameters = profile.get("parameters", {}) if profile else {}
        label = "右手" if hand == "right" else "左手"
        if parameters.get("hand", hand) != hand:
            raise ValueError(f"请在“配置设备 → {label}姿态”中修正手别设置")
        python, assets = parameters.get("glove_python"), parameters.get("mano_assets")
        if not isinstance(python, str) or not python:
            raise ValueError(f"请在“配置设备 → {label}姿态”中设置手套 Python 环境")
        interpreter = Path(python).expanduser()
        if not interpreter.is_absolute() or not interpreter.is_file() or not os.access(interpreter, os.X_OK):
            raise ValueError(f"{label}手套 Python 不存在或不可执行；请检查“配置设备”中的手套 Python 环境")
        if not isinstance(assets, str) or not assets:
            raise ValueError(f"请在“配置设备 → {label}姿态”中设置可信 MANO 资源目录")
        root = Path(assets).expanduser()
        model = root / "models" / f"MANO_{hand.upper()}.pkl"
        if not root.is_absolute() or not model.is_file():
            raise ValueError(f"{label} MANO 资源缺少 models/MANO_{hand.upper()}.pkl；请检查“配置设备”中的资源目录")
        # Preserve the venv interpreter symlink: resolving it would bypass that
        # environment. File identity also invalidates cache after replacement.
        stamp = lambda path: (path.stat().st_dev, path.stat().st_ino, path.stat().st_size, path.stat().st_mtime_ns)
        key = (str(interpreter), str(root), stamp(interpreter), stamp(model))
        return key

    def _key(self, hand):
        try:
            return self._configuration(hand)
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            self._errors[hand] = (None, str(exc))
            return None

    def available(self, hand):
        """Configuration files exist; this is not a successful model load."""
        return not self._closed and self._key(hand) is not None

    def get(self, hand):
        key = self._key(hand)
        cached = self._cache.get(hand)
        return cached[1] if key is not None and cached and cached[0] == key else None

    def error(self, hand):
        key = self._key(hand)
        error = self._errors.get(hand)
        return error[1] if error and error[0] == key else ""

    def request(self, hand):
        """Load once per hand/configuration, reusing any in-flight request."""
        if self._closed:
            return
        key = self._key(hand)
        if key is None:
            self.failed.emit(hand, self.error(hand))
            return
        cached = self._cache.get(hand)
        if cached and cached[0] == key:
            self.ready.emit(hand, cached[1])
            return
        old = self._jobs.get(hand)
        if old and old.key == key:
            return
        if old:
            self._retire(old)
        self._errors.pop(hand, None)
        process, timer = QProcess(self), QTimer(self)
        timer.setSingleShot(True)
        job = _Job(hand, key, process, timer)
        self._jobs[hand] = job
        process.setProcessChannelMode(QProcess.ProcessChannelMode.SeparateChannels)
        process.readyReadStandardOutput.connect(lambda: self._read_output(job))
        process.readyReadStandardError.connect(lambda: self._read_errors(job))
        process.finished.connect(lambda code, status: self._finished(job, code, status))
        process.errorOccurred.connect(lambda error: self._process_error(job, error))
        timer.timeout.connect(lambda: self._fail(job, "3D 模型加载超过 60 秒；请检查手套 Python 依赖和 MANO 资源，后重试"))
        process.start(key[0], _worker_arguments(hand, key[1]))
        timer.start(self._timeout_ms)

    def _read_output(self, job):
        data = bytes(job.process.readAllStandardOutput())
        if self._jobs.get(job.hand) is not job:
            return
        if len(job.output) + len(data) > MAX_OUTPUT_BYTES:
            self._fail(job, "3D 模型输出超出安全大小；请检查手套环境中的预览导出模块")
        else:
            job.output.extend(data)

    def _read_errors(self, job):
        data = bytes(job.process.readAllStandardError())
        job.errors.extend(data[-MAX_ERROR_BYTES:])
        del job.errors[:-MAX_ERROR_BYTES]

    def _retire(self, job):
        if self._jobs.get(job.hand) is job:
            self._jobs.pop(job.hand)
        job.timer.stop()
        job.timer.deleteLater()
        if job.process.state() != QProcess.ProcessState.NotRunning:
            job.process.kill()  # Only this model-export child, never a sensor.
        else:
            job.process.deleteLater()

    def _fail(self, job, message):
        if self._jobs.get(job.hand) is not job:
            return
        self._retire(job)
        self._errors[job.hand] = (job.key, message)
        self.failed.emit(job.hand, message)

    def _process_error(self, job, error):
        if self._jobs.get(job.hand) is job:
            self._read_errors(job)
            self._fail(job, "3D 模型进程无法运行；请检查手套 Python 环境：" + job.process.errorString())
        elif job.process.state() == QProcess.ProcessState.NotRunning:
            job.process.deleteLater()

    def _finished(self, job, code, status):
        if self._jobs.get(job.hand) is not job:
            job.process.deleteLater()
            return
        self._read_output(job)
        self._read_errors(job)
        if self._jobs.get(job.hand) is not job:
            return
        if code != 0 or status != QProcess.ExitStatus.NormalExit:
            detail = job.errors.decode("utf-8", errors="replace").strip()
            self._fail(job, "3D 模型加载失败；请确认手套 Python 已安装 vt-fsglove12、torch、manotorch，且 MANO 模型可信可读。"
                       + ("\n" + detail if detail else ""))
            return
        try:
            bundle = validate_bundle(json.loads(job.output), job.hand)
        except (UnicodeError, ValueError, TypeError, OverflowError, RecursionError) as exc:
            self._fail(job, "3D 模型数据无效：" + str(exc))
            return
        # A settings edit while loading must never publish the previous model.
        if self._key(job.hand) != job.key:
            self._retire(job)
            # The requester must leave its loading state and expose retry.
            # Do not attach this diagnostic to the old configuration key: a
            # new request should inspect the newly selected paths itself.
            self.failed.emit(job.hand, "3D 模型配置已在加载期间变化；请点击“重新加载模型”使用当前配置")
            return
        self._retire(job)
        self._cache[job.hand] = (job.key, bundle)
        self._errors.pop(job.hand, None)
        self.ready.emit(job.hand, bundle)

    def shutdown(self):
        self._closed = True
        for job in list(self._jobs.values()):
            self._retire(job)
        self._cache.clear()


def model_loader(configuration_provider=None):
    """One loader per application; tiles/details share cached geometry and jobs."""
    app = QCoreApplication.instance()
    if app is None:
        raise RuntimeError("创建 3D 模型加载器前需要 Qt application")
    loader = getattr(app, "_vt_tactile_model_loader", None)
    if loader is None:
        loader = TactileModelLoader(configuration_provider, app)
        app._vt_tactile_model_loader = loader
    elif configuration_provider is not None:
        loader.set_configuration_provider(configuration_provider)
    return loader
