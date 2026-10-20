"""Installation lifecycle, with synthetic processes and real private manifests."""
from concurrent.futures import ThreadPoolExecutor
import fcntl
from hashlib import sha256
import json
import os
from pathlib import Path
import signal
import stat
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from vt_tracker_deploy import installer
from vt_tracker_deploy.archive import REQUIREMENTS, TARGET
from vt_tracker_deploy.manifest import deployment_directory, load_installation


@pytest.fixture
def deployment(tmp_path, monkeypatch):
    context = SimpleNamespace(home=tmp_path / "home", calls=[], fail=None, sequence=1)
    context.home.mkdir()

    def package():
        identity = sha256(str(context.sequence).encode()).hexdigest()
        return SimpleNamespace(
            package_id=identity,
            manifest={"target": dict(TARGET), "requirements": list(REQUIREMENTS), "capture_sha256": "a" * 64},
            files={"source/pyvut/live_probe.py": b"# synthetic source\n",
                   "inputs/bundle.json": b"{}", "inputs/roles.yaml": b"roles: {}",
                   "inputs/capture.pcapng": b"capture", "wheels/fixture.whl": b"synthetic wheel"},
        )

    context.package = package

    def run(command, *, cancel, **kwargs):
        installer._check_cancel(cancel)
        context.calls.append(tuple(command))
        if "-c" in command:
            return 0, b"3.12\n"
        if "venv" in command:
            root = Path(command[-1])
            (root / "bin").mkdir(parents=True)
            python = root / "bin/python"
            python.write_text("# synthetic interpreter")
            python.chmod(0o700)
            (root / "lib/python3.12/site-packages").mkdir(parents=True)
            return (1 if context.fail == "venv" else 0), b""
        if "pip" in command:
            return (1 if context.fail == "pip" else 0), b"private process details must not escape"
        return (1, b'{"ok":false,"code":"runtime_validation"}') if context.fail == "validate" else (0, b'{"ok":true}')

    monkeypatch.setattr(installer, "read_package", lambda _: package())
    monkeypatch.setattr(installer, "_run", run)
    monkeypatch.setattr(installer.platform, "system", lambda: "Linux")
    monkeypatch.setattr(installer.platform, "machine", lambda: "x86_64")
    context.install = lambda **kwargs: installer.install_package(
        tmp_path / "test.vttracker", home=context.home, python=sys.executable, **kwargs)
    return context


def test_install_is_private_and_same_package_reuses_verified_version(deployment):
    stages = []
    first = deployment.install(progress=stages.append)
    assert stages == ["verify", "environment", "validate", "activate"]
    assert first == load_installation(home=deployment.home)
    assert stat.S_IMODE(first.bundle.stat().st_mode) == 0o600
    assert stat.S_IMODE(first.source.parent.stat().st_mode) == 0o700
    first_call_count = sum("venv" in call for call in deployment.calls)
    second = deployment.install()
    assert first == second
    assert sum("venv" in call for call in deployment.calls) == first_call_count


@pytest.mark.parametrize("failure,code", [("venv", "venv_failed"), ("pip", "dependencies_failed"),
                                         ("validate", "runtime_validation")])
def test_failed_new_version_preserves_active_installation_and_removes_partial_version(deployment, failure, code):
    previous = deployment.install()
    before = (deployment_directory(deployment.home) / "current.json").read_bytes()
    deployment.sequence += 1
    failed_id = deployment.package().package_id
    deployment.fail = failure
    with pytest.raises(installer.DeploymentError) as error:
        deployment.install()
    assert error.value.code == code
    assert "private process" not in str(error.value)
    assert load_installation(home=deployment.home) == previous
    assert (deployment_directory(deployment.home) / "current.json").read_bytes() == before
    assert not (deployment_directory(deployment.home) / "versions" / failed_id).exists()


@pytest.mark.parametrize("stage", ["environment", "validate", "activate"])
def test_cancellation_keeps_previous_activation(deployment, stage):
    previous = deployment.install()
    deployment.sequence += 1
    cancelled = False

    def progress(value):
        nonlocal cancelled
        cancelled = value == stage

    with pytest.raises(installer.DeploymentCancelled):
        deployment.install(progress=progress, cancel=lambda: cancelled)
    assert load_installation(home=deployment.home) == previous
    assert not (deployment_directory(deployment.home) / "versions" / deployment.package().package_id).exists()


def test_existing_payload_change_is_not_overwritten_or_reactivated(deployment):
    installed = deployment.install()
    path = installed.source / "pyvut/live_probe.py"
    path.write_bytes(b"modified")
    with pytest.raises(installer.DeploymentError) as error:
        deployment.install()
    assert error.value.code == "existing_installation_changed"
    assert path.read_bytes() == b"modified"


def test_existing_invalid_manifest_is_reported_as_changed_and_preserved(deployment):
    installed = deployment.install()
    path = installed.source.parent / "installation.json"
    path.write_text("{}")
    with pytest.raises(installer.DeploymentError) as error:
        deployment.install()
    assert error.value.code == "existing_installation_changed"
    assert path.read_text() == "{}"


def test_incomplete_version_is_preserved_for_inspection(deployment):
    base = deployment_directory(deployment.home)
    root = base / "versions" / deployment.package().package_id
    root.mkdir(mode=0o700, parents=True)
    base.chmod(0o700)
    root.parent.chmod(0o700)
    note = root / "do-not-delete"
    note.write_text("interrupted installation")
    with pytest.raises(installer.DeploymentError) as error:
        deployment.install()
    assert error.value.code == "incomplete_installation"
    assert note.exists()


def test_activation_replace_failure_keeps_old_pointer(deployment, monkeypatch):
    previous = deployment.install()
    deployment.sequence += 1
    current = deployment_directory(deployment.home) / "current.json"
    original = installer.os.replace

    def fail(source, destination):
        if destination == current:
            raise OSError("synthetic replacement failure")
        return original(source, destination)

    monkeypatch.setattr(installer.os, "replace", fail)
    with pytest.raises(OSError):
        deployment.install()
    assert load_installation(home=deployment.home) == previous
    assert not (current.parent / "versions" / deployment.package().package_id).exists()


def test_directory_fsync_failure_after_activation_does_not_delete_referenced_version(deployment, monkeypatch):
    base = deployment_directory(deployment.home)
    original = installer.os.fsync

    def fail(descriptor):
        target = Path(os.readlink(f"/proc/self/fd/{descriptor}"))
        if target == base and (base / "current.json").exists():
            raise OSError("synthetic durability failure after replace")
        return original(descriptor)

    monkeypatch.setattr(installer.os, "fsync", fail)
    with pytest.raises(OSError):
        deployment.install()
    assert load_installation(home=deployment.home) is not None
    identity = json.loads((base / "current.json").read_text())["installation_id"]
    assert (base / "versions" / identity / "installation.json").is_file()


def test_parallel_installers_only_build_one_version(deployment):
    with ThreadPoolExecutor(max_workers=2) as pool:
        first = pool.submit(deployment.install)
        second = pool.submit(deployment.install)
        assert first.result(timeout=5) == second.result(timeout=5)
    assert sum("venv" in call for call in deployment.calls) == 1


def test_waiting_installer_can_be_cancelled_without_touching_active_version(deployment):
    previous = deployment.install()
    base = deployment_directory(deployment.home)
    descriptor = os.open(base / ".install.lock", os.O_RDWR)
    fcntl.flock(descriptor, fcntl.LOCK_EX)
    cancelled = threading.Event()
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            result = pool.submit(deployment.install, cancel=cancelled.is_set)
            time.sleep(0.1)
            cancelled.set()
            with pytest.raises(installer.DeploymentCancelled):
                result.result(timeout=3)
    finally:
        os.close(descriptor)
    assert load_installation(home=deployment.home) == previous


def test_existing_payload_fifo_is_opened_nonblocking_before_type_check(tmp_path, monkeypatch):
    version = tmp_path / "version"
    version.mkdir()
    fifo = version / "source.py"
    os.mkfifo(fifo, 0o600)
    original = installer.os.open
    original_path_open = Path.open
    seen = []

    def checked(path, flags, *args, **kwargs):
        if Path(path) == fifo:
            seen.append(flags)
            assert flags & os.O_NONBLOCK
            assert flags & os.O_NOFOLLOW
        return original(path, flags, *args, **kwargs)

    def reject_blocking(path, *args, **kwargs):
        if path == fifo:
            raise AssertionError("A FIFO must not be opened with blocking Path.open")
        return original_path_open(path, *args, **kwargs)

    monkeypatch.setattr(installer.os, "open", checked)
    monkeypatch.setattr(Path, "open", reject_blocking)
    with pytest.raises(installer.DeploymentError) as error:
        installer._existing_payload(version, {"source.py": b"expected"})
    assert error.value.code == "existing_installation_changed"
    assert seen


def test_process_environment_cannot_redirect_pip_installation(monkeypatch):
    monkeypatch.setenv("PIP_TARGET", "/unrelated/target")
    monkeypatch.setenv("PIP_PREFIX", "/unrelated/prefix")
    monkeypatch.setenv("PIP_USER", "1")
    code, output = installer._run(
        [sys.executable, "-I", "-c",
         "import json,os; print(json.dumps({k:v for k,v in os.environ.items() if k.startswith('PIP_')}))"],
        cancel=lambda: False, timeout=5)
    assert code == 0
    environment = json.loads(output)
    assert set(environment) == {"PIP_NO_INDEX", "PIP_DISABLE_PIP_VERSION_CHECK", "PIP_CONFIG_FILE"}
    assert environment["PIP_NO_INDEX"] == "1"


def test_installation_inside_git_checkout_is_rejected_before_processes_or_files(deployment):
    (deployment.home / ".git").write_text("gitdir: fixture")
    with pytest.raises(installer.DeploymentError) as error:
        deployment.install()
    assert error.value.code == "unsafe_directory"
    assert deployment.calls == []
    assert not deployment_directory(deployment.home).exists()


def test_process_cancellation_stops_owned_group_even_after_leader_exits(monkeypatch):
    sent = []
    monkeypatch.setattr(installer.os, "killpg", lambda pid, sig: sent.append((pid, sig)))
    process = SimpleNamespace(pid=12345, poll=lambda: 0, wait=lambda **kwargs: 0)
    installer._stop(process)
    assert sent == [(12345, signal.SIGTERM), (12345, signal.SIGKILL)]


def test_process_timeout_is_bounded_and_does_not_expose_output():
    started = time.monotonic()
    with pytest.raises(installer.DeploymentError) as error:
        installer._run([sys.executable, "-I", "-u", "-c",
                        "import time; print('PRIVATE_PROCESS_OUTPUT'); time.sleep(10)"],
                       cancel=lambda: False, timeout=0.05)
    assert error.value.code == "process_timeout"
    assert "PRIVATE_PROCESS_OUTPUT" not in str(error.value)
    assert time.monotonic() - started < 3


def test_cancellation_kills_child_ignoring_term_after_its_leader_exits(tmp_path):
    ready = tmp_path / "child-ready"
    child = (
        "import os,signal,sys,time; from pathlib import Path; "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        "Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(10)"
    )
    leader = (
        "import subprocess,sys; "
        f"subprocess.Popen([sys.executable, '-I', '-c', {child!r}, {str(ready)!r}])"
    )
    with pytest.raises(installer.DeploymentCancelled):
        installer._run([sys.executable, "-I", "-c", leader],
                       cancel=ready.exists, timeout=5)
    child_pid = int(ready.read_text())
    status = Path(f"/proc/{child_pid}/stat")
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        try:
            if status.read_text().split()[2] in {"Z", "X"}:
                break
        except FileNotFoundError:
            break
        time.sleep(0.01)
    else:
        os.kill(child_pid, signal.SIGKILL)
        pytest.fail("Cancellation left its owned child running")
