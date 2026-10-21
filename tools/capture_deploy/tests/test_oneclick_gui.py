"""The installer view is exercised with inert workers and no system changes."""
from __future__ import annotations

from pathlib import Path
import gc
import threading
import time
from unittest.mock import Mock

import pytest

pytest.importorskip("tkinter")
import tkinter as tk

from oneclick import gui


@pytest.fixture
def root():
    try:
        window = tk.Tk()
    except tk.TclError as exc:
        pytest.skip(f"Tk display unavailable: {exc}")
    window.withdraw()
    yield window
    try:
        window.destroy()
    except tk.TclError:
        pass
    # Tk widgets keep Python cycles via registered callbacks. Reclaim them on
    # the owning thread before a later test starts any background workers.
    window = None
    gc.collect()


def pump_until(root, predicate):
    deadline = time.monotonic() + 3
    while not predicate():
        assert time.monotonic() < deadline, "Installer worker did not report its result"
        root.update()
        time.sleep(0.01)
    root.update()


def test_open_workbench_uses_installed_paths_and_sanitizes_frozen_environment(monkeypatch, tmp_path):
    calls = []
    sentinel = object()

    def popen(argv, **kwargs):
        calls.append((argv, kwargs))
        return sentinel

    monkeypatch.setattr(gui.subprocess, "Popen", popen)
    for name, value in {"PYTHONHOME": "/old/python", "PYTHONPATH": "/old/modules",
                        "_MEIPASS2": "/tmp/bundle", "LD_LIBRARY_PATH": "/tmp/frozen",
                        "LD_LIBRARY_PATH_ORIG": "/system/libs"}.items():
        monkeypatch.setenv(name, value)
    source = tmp_path / "installed $(not-a-command) /source"
    python = source / ".venv-capture-console/bin/python"
    result = gui.launch_workbench({"source_root": str(source), "python": str(python)})
    assert result is sentinel
    argv, options = calls[0]
    assert argv == ["/bin/bash", str(source / "tools/capture_deploy/start_workbench.sh")]
    assert options["env"]["VT_CONSOLE_PYTHON"] == str(python)
    assert options["env"]["LD_LIBRARY_PATH"] == "/system/libs"
    for name in ("PYTHONHOME", "PYTHONPATH", "_MEIPASS2", "LD_LIBRARY_PATH_ORIG"):
        assert name not in options["env"]
    assert options["start_new_session"] is True
    assert options["stdin"] == gui.subprocess.DEVNULL
    assert not options.get("shell")


def test_installation_worker_reports_completion_on_main_thread(root, tmp_path):
    worker_threads = []
    result = {"source_root": str(tmp_path / "installed"), "python": str(tmp_path / "python")}

    class InertInstaller:
        def __init__(self, directory, *, notify, cancel):
            assert directory == tmp_path
            self.notify = notify
            assert not cancel()

        def run(self):
            worker_threads.append(threading.get_ident())
            self.notify({"kind": "progress", "phase": "stage", "message": "synthetic stage"})
            return result

    view = gui.InstallerWindow(root, tmp_path, installer_factory=InertInstaller)
    assert not view.running
    view._primary()
    pump_until(root, lambda: view.result is not None)
    view.thread.join(timeout=1)
    assert worker_threads and worker_threads[0] != threading.get_ident()
    assert view.result == result
    assert view.state == "complete"
    assert not view.running
    assert view.primary.cget("state") == "normal"
    assert view.primary.cget("text") == view.text("open")
    assert "synthetic stage" in view.log.get("1.0", "end")


def test_close_requests_cancellation_then_waits_for_worker(root, tmp_path, monkeypatch):
    began, release = threading.Event(), threading.Event()
    cancellation_seen = []

    class ControlledInstaller:
        def __init__(self, directory, *, notify, cancel):
            self.cancel = cancel

        def run(self):
            began.set()
            assert release.wait(2)
            cancellation_seen.append(self.cancel())
            raise RuntimeError("Synthetic cancellation")

    view = gui.InstallerWindow(root, tmp_path, installer_factory=ControlledInstaller)
    destroyed = Mock()
    real_destroy = root.destroy
    monkeypatch.setattr(root, "destroy", destroyed)
    view._primary()
    try:
        assert began.wait(1)
        view._close()
        assert view.cancel.is_set()
        assert view.running
        assert view.status_key == "stopping"
        destroyed.assert_not_called()
        release.set()
        pump_until(root, lambda: destroyed.called)
        assert cancellation_seen == [True]
        assert not view.running
        destroyed.assert_called_once()
    finally:
        release.set()
        view.thread.join(timeout=2)
        monkeypatch.setattr(root, "destroy", real_destroy)


def test_failed_installation_can_retry_without_spawning_parallel_workers(root, tmp_path):
    attempts = []
    release = threading.Event()

    class RetryInstaller:
        def __init__(self, directory, *, notify, cancel):
            pass

        def run(self):
            attempts.append(threading.get_ident())
            if len(attempts) == 1:
                assert release.wait(2)
                raise RuntimeError("Synthetic failure without private data")
            return {"source_root": str(tmp_path), "python": "/synthetic/python"}

    view = gui.InstallerWindow(root, tmp_path, installer_factory=RetryInstaller)
    view._primary()
    try:
        pump_until(root, lambda: len(attempts) == 1)
        first_worker = view.thread
        view._primary()
        assert view.thread is first_worker
        release.set()
        pump_until(root, lambda: view.state == "failed")
        first_worker.join(timeout=1)
        assert view.primary.cget("text") == view.text("retry")
        assert "Synthetic failure" in view.log.get("1.0", "end")
        view._primary()
        pump_until(root, lambda: view.result is not None)
        view.thread.join(timeout=1)
        assert len(attempts) == 2
        assert view.state == "complete"
    finally:
        release.set()
        view.thread.join(timeout=2)


def test_language_switch_translates_completion_controls(root, tmp_path):
    view = gui.InstallerWindow(root, tmp_path)
    view.result = {"source_root": str(tmp_path), "python": "/synthetic/python"}
    view.state = view.status_key = "complete"
    view.english = True
    view._translate()
    assert view.primary.cget("text") == "Open workbench"
    assert view.status.cget("text") == "Installation complete"
    view._language()
    assert view.primary.cget("text") == "打开工作台"
    assert view.status.cget("text") == "安装完成"
    assert "下载目录" in view.intro.cget("text")


def test_workbench_launch_failure_keeps_successful_installation_available(root, tmp_path, monkeypatch):
    view = gui.InstallerWindow(root, tmp_path)
    result = {"source_root": str(tmp_path), "python": "/synthetic/python"}
    view.result = result
    monkeypatch.setattr(gui, "launch_workbench", Mock(side_effect=OSError("Synthetic launch error")))
    destroyed = Mock()
    real_destroy = root.destroy
    monkeypatch.setattr(root, "destroy", destroyed)
    try:
        view._primary()
        destroyed.assert_not_called()
        assert view.result == result
        assert "Synthetic launch error" in view.log.get("1.0", "end")
        assert view.log.winfo_manager()
    finally:
        monkeypatch.setattr(root, "destroy", real_destroy)


@pytest.mark.parametrize("phase", ["sdk", "driver", "permissions"])
def test_language_switch_preserves_current_installation_phase(root, tmp_path, phase):
    view = gui.InstallerWindow(root, tmp_path)
    view.running = True
    view.status_key = "working"
    view.english = True
    view.events.put({"kind": "progress", "phase": phase, "message": "raw Chinese detail"})
    pump_until(root, lambda: view.phase == phase)
    assert view.status.cget("text") == gui.PHASES[phase][1]
    assert view.status.cget("text") != view.text("working")
    view._language()
    assert view.phase == phase
    assert view.status.cget("text") == gui.PHASES[phase][0]
    view._language()
    assert view.status.cget("text") == gui.PHASES[phase][1]


def test_pending_phase_event_does_not_replace_cancellation_status(root, tmp_path):
    view = gui.InstallerWindow(root, tmp_path)
    view.running = True
    view._close()
    view.events.put({"kind": "progress", "phase": "sdk", "message": "stale stage"})
    pump_until(root, lambda: view.phase == "sdk")
    assert view.status.cget("text") == view.text("stopping")
    view._language()
    assert view.status.cget("text") == view.text("stopping")


def test_worker_does_not_keep_destroyed_tk_view_alive(root, tmp_path):
    """A finishing worker must never become the last owner of Tk widgets."""
    import weakref

    started, release = threading.Event(), threading.Event()

    class HeldInstaller:
        def __init__(self, directory, *, notify, cancel):
            self.notify, self.cancel = notify, cancel

        def run(self):
            started.set()
            assert release.wait(2)
            return {"source_root": str(tmp_path), "python": "/synthetic/python"}

    view = gui.InstallerWindow(root, tmp_path, installer_factory=HeldInstaller)
    view._primary()
    worker = view.thread
    view_reference = weakref.ref(view)
    try:
        assert started.wait(1)
        root.destroy()
        del view
        gc.collect()
        assert view_reference() is None
        assert worker.is_alive()
    finally:
        release.set()
        worker.join(timeout=2)
