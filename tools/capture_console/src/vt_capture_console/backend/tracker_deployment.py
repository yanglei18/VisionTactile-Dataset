"""One cancellable local deployment task; no Qt, ROS, or hardware access."""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import threading


STAGES = frozenset(("verify", "environment", "validate", "activate"))
ACTIVE_STATES = frozenset(("RUNNING", "CANCELLING"))
ERROR_CODES = frozenset((
    "unsupported_platform", "unsupported_python", "python_unavailable",
    "unsafe_directory", "process_timeout", "runtime_dependency",
    "runtime_validation", "existing_installation_changed",
    "incomplete_installation", "venv_failed", "dependencies_failed", "cancelled",
))


def _install(package, **options):
    # Optional deployment tooling is imported in the worker, never on GUI open.
    from vt_tracker_deploy.installer import install_package

    return install_package(package, **options)


class TrackerDeploymentTask:
    def __init__(self, *, installer=None, home=None):
        self._installer = installer or _install
        self._home = home
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._thread = None
        self._status = dict(state="IDLE", stage="", error="", installation=None)

    @property
    def status(self):
        with self._lock:
            return deepcopy(self._status)

    @property
    def running(self):
        return self.status["state"] in ACTIVE_STATES

    def start(self, package):
        with self._lock:
            if self._status["state"] in ACTIVE_STATES:
                raise RuntimeError("deployment_already_running")
            self._cancel = threading.Event()
            self._status = dict(state="RUNNING", stage="verify", error="", installation=None)
            self._thread = threading.Thread(
                target=self._run, args=(Path(package).expanduser(),),
                name="tracker-package-installation", daemon=False)
            self._thread.start()

    def cancel(self):
        with self._lock:
            if self._status["state"] in ACTIVE_STATES:
                self._cancel.set()
                self._status["state"] = "CANCELLING"

    def _progress(self, stage):
        if stage in STAGES:
            with self._lock:
                self._status["stage"] = stage

    def _run(self, package):
        try:
            installation = self._installer(
                package, home=self._home, python="/usr/bin/python3",
                progress=self._progress, cancel=self._cancel.is_set)
        except Exception as exc:
            # Do not forward installer output or exception text: they may
            # contain private bundle/capture contents or subprocess output.
            error = ("dependency" if isinstance(exc, ImportError) else
                     "package" if isinstance(exc, ValueError) else
                     "filesystem" if isinstance(exc, OSError) else "installation")
            code = getattr(exc, "code", "")
            if isinstance(code, str) and code in ERROR_CODES:
                error = code
            with self._lock:
                self._status.update(state="CANCELLED" if self._cancel.is_set() or error == "cancelled" else "FAILED",
                                    error=error, installation=None)
        else:
            # A returned installation is committed. A late cancel request must
            # not misreport a successfully installed version as rolled back.
            with self._lock:
                self._status.update(state="COMPLETE", stage="activate",
                                    error="", installation=installation)
