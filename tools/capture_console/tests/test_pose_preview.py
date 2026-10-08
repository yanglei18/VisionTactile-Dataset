"""Newest-only async display worker, identity fences and model-only ownership."""
import json
import os
import subprocess
import sys
import time

import pytest
from PySide6.QtCore import QProcess
from PySide6.QtTest import QSignalSpy, QTest
from shiboken6 import delete, isValid

from vt_capture_console.backend import pose_preview as module
from test_tactile_model_loader import bundle, configuration, wait_for


def mano(index=1, hand="right"):
    return dict(hand=hand, session_id="test-session", calibration_version=2, frame_index=index,
                source_realtime_ns=1_000_000_000 + index, model_rest_sha256="a" * 64,
                config_sha256="b" * 64, source_mode="live", pose=[0.] * 48, shape=[0.] * 10)


def command(monkeypatch, *, delay=.08, response="geometry"):
    document = json.dumps({"type": "ready", "bundle": bundle()})
    source = f"""
import json, sys, time
print({document!r}, flush=True)
for line in sys.stdin:
    value = json.loads(line)
    time.sleep({delay!r})
    if {response!r} == 'error':
        result = dict(type='error', id=value['id'], message='model mismatch')
    else:
        geometry = dict(vertices_m=[[value['mano']['frame_index']*.0001,0,0]]*778,
                        joints_m=[[0,0,0]]*21)
        result = dict(type='geometry', id=value['id'], geometry=geometry)
    print(json.dumps(result), flush=True)
"""
    monkeypatch.setattr(module, "_worker_arguments", lambda hand, assets: ["-I", "-c", source])


@pytest.fixture
def preview(qapp, configuration):
    value = module.PosePreview("right", lambda: configuration)
    yield value
    value.shutdown()
    QTest.qWait(30)
    value.deleteLater()
    qapp.processEvents()


def test_keys_include_session_calibration_hashes_and_source_mode():
    base = mano()
    key = module.key_for(base)
    for name, value in (("session_id", "other"), ("calibration_version", 3),
            ("frame_index", 99), ("source_realtime_ns", 99), ("model_rest_sha256", "c" * 64),
            ("config_sha256", "c" * 64), ("source_mode", "replay"), ("hand", "left")):
        assert module.key_for(dict(base, **{name: value})) != key


def test_fixed_worker_only_loads_configured_model():
    assert module._worker_arguments("left", "/trusted models") == ["-I", "-m",
        "vt_fsglove12.mano_pose_preview", "--mano-assets", "/trusted models", "--hand", "left"]


def test_async_newest_only_skips_intermediate_frames(preview, monkeypatch):
    command(monkeypatch, delay=.15)
    initialized, ready = QSignalSpy(preview.initialized), QSignalSpy(preview.ready)
    preview.submit(mano(1))
    wait_for(initialized)
    assert preview._inflight is not None
    process = preview._process
    preview.submit(mano(2))
    preview.submit(mano(3))
    preview.submit(mano(3))
    assert preview._pending[0] == module.key_for(mano(3))
    wait_for(ready)
    assert ready.count() == 1 and ready.at(0)[0]["key"] == module.key_for(mano(3))
    assert preview._serial == 2 and preview._process is process
    assert not preview.geometry["geometry"]["vertices_m"].flags.writeable
    assert preview.bundle is initialized.at(0)[0]


def test_clear_drops_inflight_and_can_resubmit_same_key(preview, monkeypatch):
    command(monkeypatch, delay=.1)
    initialized, ready = QSignalSpy(preview.initialized), QSignalSpy(preview.ready)
    preview.submit(mano())
    wait_for(initialized)
    process = preview._process
    preview.clear()
    QTest.qWait(150)
    assert ready.count() == 0 and preview.geometry is None
    preview.submit(mano())
    wait_for(ready)
    assert ready.count() == 1 and preview._process is process and preview._serial == 2


def test_clear_then_new_request_before_old_reply_uses_generation(preview, monkeypatch):
    command(monkeypatch, delay=.1)
    initialized, ready = QSignalSpy(preview.initialized), QSignalSpy(preview.ready)
    preview.submit(mano())
    wait_for(initialized)
    preview.clear()
    preview.submit(mano())
    wait_for(ready)
    assert ready.count() == 1 and preview._serial == 2


@pytest.mark.parametrize("change", [lambda v: v.update(hand="left"),
    lambda v: v.update(pose=[0.] * 47), lambda v: v.update(shape=[float("nan")] * 10),
    lambda v: v.update(frame_index=True), lambda v: v.update(source_mode="demo")])
def test_bad_input_does_not_start_any_process(preview, change):
    value = mano()
    change(value)
    failed = QSignalSpy(preview.failed)
    assert preview.submit(value) is None
    assert failed.count() == 1 and not preview.findChildren(QProcess)


def test_per_frame_model_mismatch_does_not_kill_worker(preview, monkeypatch):
    command(monkeypatch, response="error")
    failed, ready = QSignalSpy(preview.failed), QSignalSpy(preview.ready)
    preview.submit(mano())
    wait_for(failed)
    process = preview._process
    assert ready.count() == 0 and process.state() == QProcess.ProcessState.Running
    assert preview.geometry is None
    preview.submit(mano())
    QTest.qWait(50)
    assert preview._serial == 1
    preview.submit(mano(2))
    assert failed.wait(3000)
    assert failed.count() == 2 and preview._process is process


@pytest.mark.parametrize("source", ["print('not JSON',flush=True)",
    "print('x'*600000,flush=True)", "import sys;sys.exit(2)"])
def test_bad_worker_is_latched_until_explicit_retry(preview, monkeypatch, source):
    monkeypatch.setattr(module, "_worker_arguments", lambda hand, assets: ["-I", "-c", source])
    failed = QSignalSpy(preview.failed)
    preview.submit(mano())
    wait_for(failed)
    assert preview._process is None
    preview.submit(mano(2))
    QTest.qWait(40)
    assert failed.count() == 1 and preview._process is None
    command(monkeypatch)
    preview.retry()
    ready = QSignalSpy(preview.ready)
    preview.submit(mano(2))
    wait_for(ready)
    assert ready.count() == 1


def test_frame_timeout_stops_only_owned_worker(preview, monkeypatch):
    preview._frame_timeout_ms = 40
    command(monkeypatch, delay=.5)
    failed = QSignalSpy(preview.failed)
    preview.submit(mano())
    wait_for(failed)
    assert "超时" in failed.at(0)[0] and preview._process is None


def test_configuration_change_drops_old_model_response(preview, monkeypatch, configuration):
    command(monkeypatch, delay=.1)
    initialized, ready, failed = (QSignalSpy(preview.initialized), QSignalSpy(preview.ready),
                                 QSignalSpy(preview.failed))
    preview.submit(mano())
    wait_for(initialized)
    configuration["process_groups"][0]["parameters"]["mano_assets"] = "/missing/model"
    wait_for(failed)
    assert ready.count() == 0 and preview.geometry is None and preview._process is None


def test_shutdown_waits_for_idle_worker_eof_and_is_idempotent(preview, monkeypatch):
    command(monkeypatch, delay=0.)
    ready = QSignalSpy(preview.ready)
    preview.submit(mano())
    wait_for(ready)
    child = preview._process
    started = time.monotonic()
    preview.shutdown()
    assert time.monotonic() - started < 1.
    assert not isValid(child) or child.state() == QProcess.ProcessState.NotRunning
    assert not preview._retiring and preview._process is None
    preview.shutdown()


def test_shutdown_kills_blocked_owned_worker_after_bounded_grace(preview, monkeypatch):
    command(monkeypatch, delay=30.)
    initialized = QSignalSpy(preview.initialized)
    preview.submit(mano())
    wait_for(initialized)
    child = preview._process
    started = time.monotonic()
    preview.shutdown()
    assert time.monotonic() - started < 1.
    assert not isValid(child) or child.state() == QProcess.ProcessState.NotRunning
    assert not preview._retiring


def test_already_deleted_qprocess_callbacks_are_safe(preview):
    child = QProcess(preview)
    preview._retiring.add(child)
    delete(child)
    preview._read(child)
    preview._read_errors(child)
    preview._process_error(child)
    preview._finished(child, 0, QProcess.ExitStatus.NormalExit)
    assert not preview._retiring


@pytest.mark.parametrize("stage", ["ready", "startup", "inflight", "retired"])
def test_application_exit_reaps_workers_without_qt_destruction_errors(configuration, stage):
    # The real QApplication destruction boundary cannot be tested by merely
    # processEvents() in the suite's session-scoped app. Use a clean interpreter.
    document = json.dumps({"type": "ready", "bundle": bundle()})
    worker_source = f"""
import json,sys,time
if {stage!r} == 'startup':
    time.sleep(30)
print({document!r},flush=True)
for line in sys.stdin:
    request=json.loads(line)
    if {stage!r} == 'inflight':
        time.sleep(30)
    print(json.dumps(dict(type='geometry',id=request['id'],geometry=dict(
        vertices_m=[[0.,0.,0.]]*778,joints_m=[[0.,0.,0.]]*21))),flush=True)
"""
    script = f"""
import time
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer,QProcess
from shiboken6 import delete,isValid
from vt_capture_console.backend import pose_preview as module
app=QApplication([])
module._worker_arguments=lambda hand,assets: ['-I','-c',{worker_source!r}]
preview=module.pose_preview('right',lambda: {configuration!r})
if {stage!r} == 'ready':
    preview.ready.connect(lambda result: app.quit())
elif {stage!r} == 'inflight':
    preview.initialized.connect(lambda result: QTimer.singleShot(30,app.quit))
elif {stage!r} == 'retired':
    def retire_and_quit(result):
        preview.retry()
        app.quit()
    preview.initialized.connect(retire_and_quit)
else:
    QTimer.singleShot(30,app.quit)
preview.submit({mano()!r})
child=preview._process
QTimer.singleShot(5000,app.quit)
started=time.monotonic()
app.exec()
assert time.monotonic()-started < 2.
assert preview._process is None and not preview._retiring
assert not isValid(child) or child.state() == QProcess.ProcessState.NotRunning
delete(app)
print('APPLICATION_EXIT_CLEAN')
"""
    result = subprocess.run([sys.executable, "-I", "-c", script],
        env=dict(os.environ, QT_QPA_PLATFORM="offscreen"), capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "APPLICATION_EXIT_CLEAN" in result.stdout
    for marker in ("Destroyed while process", "C++ object", "RuntimeError", "Traceback"):
        assert marker not in result.stdout + result.stderr
