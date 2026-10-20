"""Install verified packages into immutable versions and activate only on success."""
from __future__ import annotations

from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import selectors
import shutil
import signal
import stat
import subprocess
import time
import uuid

from .archive import read_package
from .manifest import Installation, deployment_directory, load_installation


class DeploymentError(RuntimeError):
    """A public error code; never includes subprocess output or private inputs."""

    def __init__(self, code: str):
        self.code = code
        super().__init__(code)


class DeploymentCancelled(DeploymentError):
    def __init__(self):
        super().__init__("cancelled")


def _check_cancel(cancel):
    if cancel():
        raise DeploymentCancelled()


def _directory(path: Path, *, private=False):
    """Create missing directory components, refusing symlinks along the chain."""
    path = path.absolute()
    current = Path(path.anchor)
    for part in path.parts[1:]:
        current /= part
        try:
            current.mkdir(mode=0o700)
        except FileExistsError:
            pass
        info = current.lstat()
        if not stat.S_ISDIR(info.st_mode):
            raise DeploymentError("unsafe_directory")
    info = path.lstat()
    if private and (info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o700):
        raise DeploymentError("unsafe_directory")


def _write_file(path: Path, data: bytes):
    with os.fdopen(os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600), "wb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())


def _atomic_json(path: Path, value, *, on_replace=lambda: None):
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        _write_file(temporary, (json.dumps(value, sort_keys=True) + "\n").encode())
        os.replace(temporary, path)
        # The new pointer is already visible even if the directory fsync fails.
        on_replace()
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        temporary.unlink(missing_ok=True)


@contextmanager
def _installation_lock(base, cancel):
    descriptor = os.open(base / ".install.lock", os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o600:
            raise DeploymentError("unsafe_directory")
        while True:
            _check_cancel(cancel)
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                time.sleep(0.05)
        yield
    finally:
        os.close(descriptor)


def _stop(process):
    # A finished leader may still have children holding its output pipe open.
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()
    else:
        # A leader can exit on SIGTERM while one of its children ignores it.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def _run(command, *, cancel, timeout=300):
    """Bound output in memory and cancel the entire subprocess group we own."""
    _check_cancel(cancel)
    env = {key: value for key, value in os.environ.items() if not key.startswith("PIP_")}
    for key in (
        "PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV", "LD_PRELOAD", "LD_LIBRARY_PATH",
        "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH",
    ):
        env.pop(key, None)
    env.update(PIP_NO_INDEX="1", PIP_DISABLE_PIP_VERSION_CHECK="1",
               PIP_CONFIG_FILE=os.devnull, PYTHONDONTWRITEBYTECODE="1")
    output = bytearray()
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               stdin=subprocess.DEVNULL, start_new_session=True, env=env)
    assert process.stdout is not None
    started = time.monotonic()
    try:
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            eof = False
            while not eof or process.poll() is None:
                _check_cancel(cancel)
                if time.monotonic() - started > timeout:
                    raise DeploymentError("process_timeout")
                for key, _ in selector.select(timeout=0.05):
                    block = os.read(key.fd, 8192)
                    if block:
                        output.extend(block)
                        del output[:-65536]
                    else:
                        selector.unregister(key.fileobj)
                        eof = True
            return process.wait(), bytes(output)
    finally:
        _stop(process)
        process.stdout.close()


def _validate_runtime(version, digest, *, cancel):
    verifier = Path(__file__).with_name("verify_runtime.py").resolve()
    code, output = _run([str(version / "venv/bin/python"), "-I", "-B", str(verifier),
                         str(version), digest], cancel=cancel)
    try:
        result = json.loads(output)
    except (ValueError, UnicodeError):
        result = {}
    if code or result != {"ok": True}:
        error = result.get("code") if isinstance(result, dict) else None
        raise DeploymentError(error if error in {"runtime_dependency", "runtime_validation"} else "runtime_validation")


def _existing_payload(version, files):
    for name, expected in files.items():
        path = version / name
        for parent in (path, *path.parents):
            if parent == version.parent:
                break
            if parent.is_symlink():
                raise DeploymentError("existing_installation_changed")
        try:
            descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
            with os.fdopen(descriptor, "rb") as stream:
                info = os.fstat(stream.fileno())
                if not stat.S_ISREG(info.st_mode) or info.st_size != len(expected):
                    raise DeploymentError("existing_installation_changed")
                if hashlib.sha256(stream.read(len(expected) + 1)).digest() != hashlib.sha256(expected).digest():
                    raise DeploymentError("existing_installation_changed")
        except OSError as exc:
            raise DeploymentError("existing_installation_changed") from exc


def install_package(
    package: Path, *, home=None, python="/usr/bin/python3",
    progress=lambda stage: None, cancel=lambda: False,
) -> Installation:
    """Install a trusted package offline. No settings save, backend restart, or HID I/O."""
    _check_cancel(cancel)
    progress("verify")
    verified = read_package(Path(package))
    target = verified.manifest["target"]
    if platform.system() != target["system"] or platform.machine() != target["machine"]:
        raise DeploymentError("unsupported_platform")
    base = deployment_directory(home)
    if any((parent / ".git").exists() or (parent / ".git").is_symlink()
           for parent in (base, *base.parents)):
        # The private bundle loader forbids credentials in a Git checkout.
        raise DeploymentError("unsafe_directory")
    python = Path(python)
    if not python.is_absolute() or not python.is_file() or not os.access(python, os.X_OK):
        raise DeploymentError("python_unavailable")
    code, output = _run([str(python), "-I", "-c",
                         "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"],
                        cancel=cancel, timeout=30)
    if code or output.strip().decode(errors="replace") != target["python"]:
        raise DeploymentError("unsupported_python")
    _directory(base, private=True)
    _directory(base / "versions", private=True)
    with _installation_lock(base, cancel):
        version = base / "versions" / verified.package_id
        created = False
        activated = False
        try:
            if version.exists() or version.is_symlink():
                try:
                    existing = load_installation(home=home, installation_id=verified.package_id)
                except ValueError as exc:
                    raise DeploymentError("existing_installation_changed") from exc
                if existing is None:
                    # A killed installer can leave an incomplete version. Preserve
                    # it for inspection; never guess whether deleting it is safe.
                    raise DeploymentError("incomplete_installation")
                _existing_payload(version, verified.files)
            else:
                version.mkdir(mode=0o700)
                created = True
                for name, data in verified.files.items():
                    _check_cancel(cancel)
                    destination = version / name
                    _directory(destination.parent)
                    _write_file(destination, data)
                progress("environment")
                code, _ = _run([str(python), "-I", "-m", "venv", str(version / "venv")], cancel=cancel)
                if code:
                    raise DeploymentError("venv_failed")
                code, _ = _run([
                    str(version / "venv/bin/python"), "-I", "-m", "pip", "install",
                    "--no-index", "--no-deps", "--no-compile", "--no-cache-dir",
                    "--find-links", str(version / "wheels"),
                    *verified.manifest["requirements"],
                ], cancel=cancel)
                if code:
                    raise DeploymentError("dependencies_failed")
            progress("validate")
            _validate_runtime(version, verified.manifest["capture_sha256"], cancel=cancel)
            _check_cancel(cancel)
            if created:
                _atomic_json(version / "installation.json", {
                    "schema_version": 1, "installation_id": verified.package_id,
                    "capture_sha256": verified.manifest["capture_sha256"],
                    "paths": {
                        "source": "source", "python": "venv/bin/python",
                        "site_packages": f"venv/lib/python{target['python']}/site-packages",
                        "bundle": "inputs/bundle.json", "roles": "inputs/roles.yaml",
                        "capture": "inputs/capture.pcapng",
                    },
                })
            installation = load_installation(home=home, installation_id=verified.package_id)
            if installation is None:
                raise DeploymentError("runtime_validation")
            progress("activate")
            _check_cancel(cancel)

            def mark_activated():
                nonlocal activated
                activated = True

            _atomic_json(base / "current.json", {"schema_version": 1, "installation_id": verified.package_id},
                         on_replace=mark_activated)
            return installation
        finally:
            if created and not activated:
                shutil.rmtree(version)
