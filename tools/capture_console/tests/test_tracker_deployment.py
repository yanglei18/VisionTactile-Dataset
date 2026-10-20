"""Deployment tasks use inert installers; never install dependencies or hardware."""
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

from vt_capture_console.backend.tracker_deployment import TrackerDeploymentTask


def completed(task):
    deadline = time.monotonic() + 3
    while task.running and time.monotonic() < deadline:
        time.sleep(.005)
    assert not task.running
    return task.status


def test_installer_runs_off_caller_thread_and_returns_only_structured_result():
    caller = threading.get_ident()
    result = SimpleNamespace(installation_id="fixture")
    calls = []

    def install(package, **options):
        assert threading.get_ident() != caller
        calls.append((package, options))
        options["progress"]("environment")
        return result

    task = TrackerDeploymentTask(installer=install, home=Path("/private/home"))
    assert task.status == dict(state="IDLE", stage="", error="", installation=None)
    task.start("/local/fixture.vttracker")
    status = completed(task)
    assert status["state"] == "COMPLETE" and status["installation"] == result
    assert calls[0][0] == Path("/local/fixture.vttracker")
    assert calls[0][1]["home"] == Path("/private/home")
    assert calls[0][1]["python"] == "/usr/bin/python3"
    assert not calls[0][1]["cancel"]()


def test_cancellation_is_cooperative_and_prevents_duplicate_start():
    entered = threading.Event()

    def install(_package, **options):
        entered.set()
        while not options["cancel"]():
            time.sleep(.005)
        raise RuntimeError("private bundle body must not escape")

    task = TrackerDeploymentTask(installer=install)
    task.start("/local/fixture.vttracker")
    assert entered.wait(1)
    with pytest.raises(RuntimeError, match="already_running"):
        task.start("/other.vttracker")
    task.cancel()
    status = completed(task)
    assert status["state"] == "CANCELLED" and status["installation"] is None
    assert "private bundle" not in str(status)


@pytest.mark.parametrize("exception,code", [
    (ImportError, "dependency"), (ValueError, "package"),
    (OSError, "filesystem"), (RuntimeError, "installation"),
])
def test_errors_do_not_expose_private_package_contents(exception, code):
    def install(*args, **kwargs):
        raise exception("secret_capture_token_abcdef")

    task = TrackerDeploymentTask(installer=install)
    task.start("/local/package.vttracker")
    status = completed(task)
    assert status["state"] == "FAILED" and status["error"] == code
    assert "secret_capture" not in str(status)


@pytest.mark.parametrize("code,expected", [
    ("runtime_dependency", "runtime_dependency"),
    ("runtime_validation", "runtime_validation"),
    ("unsupported_python", "unsupported_python"),
    ("untrusted_secret_capture_token", "installation"),
])
def test_structured_installer_errors_use_only_allowlisted_codes(code, expected):
    class InstallError(RuntimeError):
        pass

    def install(*args, **kwargs):
        error = InstallError("secret_capture_token_abcdef")
        error.code = code
        raise error

    task = TrackerDeploymentTask(installer=install)
    task.start("/local/package.vttracker")
    status = completed(task)
    assert status["state"] == "FAILED" and status["error"] == expected
    assert "secret_capture" not in str(status)


def test_late_cancel_does_not_misreport_committed_installation():
    entered, release = threading.Event(), threading.Event()

    def install(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        return SimpleNamespace(installation_id="already-committed")

    task = TrackerDeploymentTask(installer=install)
    task.start("/local/package.vttracker")
    assert entered.wait(1)
    task.cancel()
    release.set()
    status = completed(task)
    assert status["state"] == "COMPLETE"
    assert status["installation"].installation_id == "already-committed"
