from __future__ import annotations

import os
import re
import signal
import subprocess
import tempfile
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import IO, Callable, Protocol

import yaml

from .bag_contract import expected_topic_type, expected_topics

_CAMERA_NAME_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_ABSOLUTE_TOPIC_PATTERN = re.compile(
    r"(?:/[A-Za-z_][A-Za-z0-9_]*)+"
)
_BEST_EFFORT_PROFILE = {
    "history": "keep_last",
    "depth": 30,
    "reliability": "best_effort",
    "durability": "volatile",
}
MAX_BAG_DURATION_SECONDS = 300
MAX_BAG_SIZE_BYTES = 137_438_953_472
MAX_CACHE_SIZE_BYTES = 1_073_741_824


class _Process(Protocol):
    pid: int

    def poll(self) -> int | None: ...

    def wait(self, timeout: float) -> int: ...


@dataclass(frozen=True)
class RecorderStopResult:
    valid: bool
    returncode: int | None
    reason: str
    termination_confirmed: bool
    possibly_alive: bool

    @property
    def clean(self) -> bool:
        return self.valid


def required_topics(camera_names: Sequence[str]) -> tuple[str, ...]:
    """Return the explicit recorder allowlist for *camera_names*."""

    if isinstance(camera_names, (str, bytes)):
        raise ValueError("camera names must be a non-empty sequence")
    names = tuple(camera_names)
    if not names:
        raise ValueError("camera names must not be empty")
    if any(
        not isinstance(name, str) or _CAMERA_NAME_PATTERN.fullmatch(name) is None
        for name in names
    ):
        raise ValueError("camera names must be safe ROS namespace tokens")
    if len(names) != len(set(names)):
        raise ValueError("camera names must be unique")

    return tuple(expected_topics(names))


def _qos_profile(topic: str) -> dict[str, object]:
    expected_topic_type(topic)
    return dict(_BEST_EFFORT_PROFILE)


def _validated_topics(topics: Sequence[str]) -> tuple[str, ...]:
    if isinstance(topics, (str, bytes)):
        raise ValueError("topics must be a non-empty explicit sequence")
    values = tuple(topics)
    if not values:
        raise ValueError("topic list must not be empty")
    if any(
        not isinstance(topic, str)
        or _ABSOLUTE_TOPIC_PATTERN.fullmatch(topic) is None
        for topic in values
    ):
        raise ValueError("topics must be absolute, wildcard-free ROS names")
    if len(values) != len(set(values)):
        raise ValueError("topic list must not contain duplicates")

    for topic in values:
        try:
            expected_topic_type(topic)
        except ValueError as exc:
            raise ValueError(f"unsupported recorder topic: {topic}") from exc
    return values


def _validated_record_topics(topics: Sequence[str]) -> tuple[str, ...]:
    values = _validated_topics(topics)
    camera_names = {
        topic.split("/", 2)[1]
        for topic in values
    }
    if (
        len(camera_names) != 3
        or values != tuple(expected_topics(tuple(camera_names)))
    ):
        raise ValueError(
            "record topics must match the exact three-camera "
            "recorder-only contract"
        )
    return values


def _new_qos_path(path: Path | str) -> Path:
    target = Path(path)
    if not target.is_absolute():
        raise ValueError("QoS override path must be absolute")
    if target.is_symlink():
        raise ValueError("QoS override path must not be a symlink")
    if target.suffix not in {".yaml", ".yml"}:
        raise ValueError("QoS override path must name a YAML file")
    if target.resolve(strict=False) != target:
        raise ValueError("QoS override path must be canonical")
    if not target.parent.is_dir():
        raise ValueError("QoS override parent must be an existing directory")
    if target.exists():
        raise FileExistsError(f"refusing to overwrite unrelated file: {target}")
    return target


def write_qos_overrides(path: Path | str, topics: Sequence[str]) -> Path:
    """Write rosbag2 QoS overrides for the explicit *topics* list."""

    values = _validated_topics(topics)
    target = _new_qos_path(path)
    document = {topic: _qos_profile(topic) for topic in values}
    payload = yaml.safe_dump(document, sort_keys=False)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=target.parent,
        prefix=f".{target.name}.",
        suffix=".tmp",
    )
    temporary = Path(temporary_name)
    try:
        stream = os.fdopen(descriptor, "w", encoding="utf-8")
        descriptor = -1
        with stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, target, follow_symlinks=False)
        temporary.unlink()
        _fsync_directory(target.parent)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)
    return target


def _fsync_directory(directory: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    descriptor = os.open(directory, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def build_record_command(
    *,
    output_path: Path | str,
    storage_config_path: Path | str,
    qos_overrides_path: Path | str,
    topics: Sequence[str],
) -> list[str]:
    """Build the system ``ros2 bag record`` argv for an MCAP session."""

    output = _validated_output_path(output_path)
    storage_config = _validated_config_path(
        storage_config_path, "storage config"
    )
    qos_overrides = _validated_config_path(
        qos_overrides_path, "QoS config"
    )
    values = _validated_record_topics(topics)
    return [
        "ros2",
        "bag",
        "record",
        "--storage",
        "mcap",
        "--output",
        str(output),
        "--storage-config-file",
        str(storage_config),
        "--qos-profile-overrides-path",
        str(qos_overrides),
        "--max-bag-duration",
        str(MAX_BAG_DURATION_SECONDS),
        "--max-bag-size",
        str(MAX_BAG_SIZE_BYTES),
        "--max-cache-size",
        str(MAX_CACHE_SIZE_BYTES),
        "--disable-keyboard-controls",
        "--include-unpublished-topics",
        "--node-name",
        "vt_rosbag_recorder",
        "--topics",
        *values,
    ]


def _absolute_canonical_path(path: Path | str, context: str) -> Path:
    value = Path(path)
    if not value.is_absolute():
        raise ValueError(f"{context} path must be absolute")
    if value.is_symlink():
        raise ValueError(f"{context} path must not be a symlink")
    if value.resolve(strict=False) != value:
        raise ValueError(f"{context} path must be canonical")
    return value


def _validated_output_path(path: Path | str) -> Path:
    output = _absolute_canonical_path(path, "output")
    if not output.parent.is_dir():
        raise ValueError("output parent must be an existing directory")
    if output.exists():
        raise FileExistsError(f"output path already exists: {output}")
    return output


def _validated_config_path(path: Path | str, context: str) -> Path:
    config = _absolute_canonical_path(path, context)
    if not config.is_file():
        raise FileNotFoundError(f"{context} file does not exist: {config}")
    return config


class RecorderProcess:
    """Own one rosbag2 process and its recorder log."""

    def __init__(
        self,
        command: Sequence[str],
        *,
        session_parent: Path | str,
        popen_factory: Callable[..., _Process] | None = None,
        killpg: Callable[[int, int], None] | None = None,
    ) -> None:
        self._command = _validated_command(command)
        self._session_parent = _validated_session_parent(session_parent)
        self._popen_factory = popen_factory or subprocess.Popen
        self._killpg = killpg or os.killpg
        self._process: _Process | None = None
        self._log_stream: IO[str] | None = None
        self._started = False
        self._stop_result: RecorderStopResult | None = None
        self._stop_fault_reason: str | None = None

    @property
    def pid(self) -> int | None:
        return None if self._process is None else self._process.pid

    @property
    def alive(self) -> bool:
        if self._process is None or self._stop_result is not None:
            return False
        try:
            return self.poll() is None
        except Exception:
            return True

    def poll(self) -> int | None:
        if self._process is None:
            raise RuntimeError("recorder has not started")
        return self._process.poll()

    def start(self) -> int:
        if self._started:
            raise RuntimeError("RecorderProcess may be started only once")
        log_path = self._session_parent / "recorder.log"
        self._log_stream = log_path.open("x", encoding="utf-8")
        try:
            self._process = self._popen_factory(
                list(self._command),
                start_new_session=True,
                stdout=self._log_stream,
                stderr=subprocess.STDOUT,
            )
        except BaseException:
            self._log_stream.close()
            self._log_stream = None
            log_path.unlink(missing_ok=True)
            raise
        self._started = True
        return self._process.pid

    def stop(self) -> RecorderStopResult:
        if self._stop_result is not None:
            return self._stop_result
        if self._process is None:
            raise RuntimeError("recorder has not started")

        try:
            returncode = self._process.poll()
        except Exception:
            return self._recover_with_termination("poll_failed")
        if returncode is not None:
            return self._finish(
                valid=False,
                returncode=returncode,
                reason=self._stop_fault_reason or "unexpected_exit",
            )

        try:
            self._killpg(self._process.pid, signal.SIGINT)
        except Exception:
            return self._control_failure("sigint_signal_failed")
        try:
            returncode = self._process.wait(timeout=60.0)
        except subprocess.TimeoutExpired:
            return self._terminate_after_sigint_timeout()
        except Exception:
            return self._control_failure("sigint_wait_failed")
        if self._stop_fault_reason is not None:
            return self._finish(
                valid=False,
                returncode=returncode,
                reason=self._stop_fault_reason,
            )
        return self._finish(
            valid=returncode == 0,
            returncode=returncode,
            reason="clean_exit" if returncode == 0 else "nonzero_exit",
        )

    def _terminate_after_sigint_timeout(self) -> RecorderStopResult:
        assert self._process is not None
        try:
            self._killpg(self._process.pid, signal.SIGTERM)
        except Exception:
            return self._control_failure("sigterm_signal_failed")
        try:
            returncode = self._process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            return self._kill_and_reap("forced_kill")
        except Exception:
            return self._control_failure("sigterm_wait_failed")
        return self._finish(
            valid=False,
            returncode=returncode,
            reason="sigint_timeout",
        )

    def _recover_with_termination(self, reason: str) -> RecorderStopResult:
        assert self._process is not None
        self._remember_stop_fault(reason)
        try:
            self._killpg(self._process.pid, signal.SIGTERM)
        except Exception:
            return self._control_failure(
                f"{reason}_sigterm_signal_failed"
            )
        try:
            returncode = self._process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            return self._kill_and_reap(f"{reason}_forced_kill")
        except Exception:
            return self._control_failure(f"{reason}_sigterm_wait_failed")
        return self._finish(
            valid=False,
            returncode=returncode,
            reason=reason,
        )

    def _kill_and_reap(self, reason: str) -> RecorderStopResult:
        assert self._process is not None
        try:
            self._killpg(self._process.pid, signal.SIGKILL)
        except Exception:
            return self._control_failure("sigkill_signal_failed")
        try:
            returncode = self._process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            return self._control_failure("sigkill_wait_timeout")
        except Exception:
            return self._control_failure("sigkill_wait_failed")
        return self._finish(
            valid=False,
            returncode=returncode,
            reason=reason,
        )

    def _remember_stop_fault(self, reason: str) -> str:
        if self._stop_fault_reason is None:
            self._stop_fault_reason = reason
        return self._stop_fault_reason

    def _control_failure(self, reason: str) -> RecorderStopResult:
        assert self._process is not None
        fault_reason = self._remember_stop_fault(reason)
        try:
            returncode = self._process.poll()
        except Exception:
            return self._uncertain(fault_reason)
        if returncode is None:
            return self._uncertain(fault_reason)
        return self._finish(
            valid=False,
            returncode=returncode,
            reason=fault_reason,
        )

    def _uncertain(self, reason: str) -> RecorderStopResult:
        return RecorderStopResult(
            valid=False,
            returncode=None,
            reason=reason,
            termination_confirmed=False,
            possibly_alive=True,
        )

    def _finish(
        self, *, valid: bool, returncode: int | None, reason: str
    ) -> RecorderStopResult:
        self._stop_result = RecorderStopResult(
            valid=valid,
            returncode=returncode,
            reason=reason,
            termination_confirmed=True,
            possibly_alive=False,
        )
        if self._log_stream is not None:
            self._log_stream.close()
        return self._stop_result


def _validated_command(command: Sequence[str]) -> list[str]:
    if isinstance(command, (str, bytes)):
        raise ValueError("command must be an argv sequence, never a shell string")
    values = list(command)
    if not values:
        raise ValueError("command must not be empty")
    if any(
        not isinstance(value, str) or not value or "\x00" in value
        for value in values
    ):
        raise ValueError("command argv must contain non-empty strings without NUL")
    return values


def _validated_session_parent(path: Path | str) -> Path:
    parent = _absolute_canonical_path(path, "session parent")
    if not parent.is_dir():
        raise ValueError("session parent must be an existing directory")
    return parent
