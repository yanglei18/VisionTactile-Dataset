"""Offline configuration and owned subprocesses for the calibration GUI.

No device is opened on import or while constructing commands.  The caller must
explicitly start a ManagedJob, and wait for recording to finish before solving.
"""

from __future__ import annotations

from datetime import datetime, timezone
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile

import yaml

from .charuco import create_board
from .config import BoardConfig, CalibrationConfig, load_config
from .config_writer import build_config_document, write_config


def _external_path(path: Path) -> Path:
    """Keep generated data outside Git repositories, including linked worktrees."""
    expanded = Path(path).expanduser()
    if not expanded.is_absolute():
        raise ValueError("output path must be absolute and outside the repository")
    target = expanded.resolve()
    for parent in (target, *target.parents):
        marker = parent / ".git"
        if marker.exists() or marker.is_symlink():
            raise ValueError(f"output must be outside Git repositories: {target}")
    return target


def _new_output(path: Path) -> Path:
    expanded = Path(path).expanduser()
    target = _external_path(expanded)
    if expanded.exists() or expanded.is_symlink() or target.exists():
        raise FileExistsError(f"refusing to overwrite output: {expanded}")
    if not target.parent.is_dir():
        raise FileNotFoundError(f"output parent does not exist: {target.parent}")
    return target


def _topics(config: CalibrationConfig) -> tuple[str, str, str, str]:
    return (
        config.image_topic,
        config.camera_info_topic,
        config.timing_topic,
        config.tracker_sample_topic,
    )


def _qos_document(config: CalibrationConfig) -> dict[str, object]:
    # Best-effort readers accept both reliable and best-effort publishers.
    # Volatile readers are compatible with both common camera durability modes.
    return {
        topic: {
            "history": "keep_last",
            "depth": 10,
            "reliability": "best_effort",
            "durability": "volatile",
        }
        for topic in _topics(config)
    }


def _write_qos(path: Path, config: CalibrationConfig) -> Path:
    target = _new_output(path)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(target, flags, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        yaml.safe_dump(_qos_document(config), stream, sort_keys=False)
        stream.flush()
        os.fsync(stream.fileno())
    return target


def _create_session(output_root: Path, document: dict[str, object]) -> Path:
    root = _external_path(output_root)
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session = Path(tempfile.mkdtemp(prefix=f"calibration_{stamp}_", dir=root))
    config_path = session / "config.yaml"
    qos_path = session / "recorder_qos.yaml"
    try:
        write_config(config_path, document)
        config = load_config(config_path)
        create_board(config.board)
        _write_qos(qos_path, config)
    except BaseException:
        # Only these two files in our newly allocated private directory are ours.
        config_path.unlink(missing_ok=True)
        qos_path.unlink(missing_ok=True)
        session.rmdir()
        raise
    return config_path


def prepare_config(
    output_root: Path,
    camera_name: str,
    tracker_role: str,
    square_length_mm: float,
    marker_length_mm: float,
    squares_x: int = 9,
    squares_y: int = 6,
    dictionary: str = "DICT_5X5_1000",
) -> Path:
    """Snapshot reference hardware configuration into a unique external session.

    The existing solver's corner-count and quality thresholds are preserved.
    recorder_qos.yaml is created beside the returned config.yaml.
    """
    for value in (square_length_mm, marker_length_mm):
        if type(value) not in {int, float} or not math.isfinite(value):
            raise ValueError("measured board lengths must be finite numbers")
    if type(squares_x) is not int or type(squares_y) is not int:
        raise ValueError("board square counts must be integers")
    if not isinstance(dictionary, str) or not dictionary:
        raise ValueError("board dictionary must be a non-empty string")
    document = build_config_document(
        camera_name=camera_name,
        tracker_role=tracker_role,
        square_length_mm=square_length_mm,
        marker_length_mm=marker_length_mm,
    )
    board = document["board"]
    board.update(squares_x=squares_x, squares_y=squares_y, dictionary=dictionary)
    create_board(BoardConfig(**board))
    return _create_session(output_root, document)


def import_config(output_root: Path, source: Path) -> Path:
    """Copy any supported config into a session; never refer to its mutable source."""
    source = Path(source).expanduser().resolve()
    config = load_config(source)
    create_board(config.board)
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    # write_config validates the copied document again, including a changed source.
    return _create_session(output_root, document)


def record_command(
    config: CalibrationConfig,
    bag_path: Path,
    qos_path: Path | None = None,
) -> list[str]:
    """Build the four-topic recorder command, without starting ROS or a device.

    Pass the session's recorder_qos.yaml.  If omitted, an exclusive sibling
    <bag-name>.qos.yaml is written for standalone callers (never overwritten).
    """
    bag = _new_output(bag_path)
    if qos_path is None:
        qos = _write_qos(bag.with_name(f"{bag.name}.qos.yaml"), config)
    else:
        qos = Path(qos_path).expanduser().resolve()
        if not qos.is_file():
            raise FileNotFoundError(f"recorder QoS file does not exist: {qos}")
        if yaml.safe_load(qos.read_text(encoding="utf-8")) != _qos_document(config):
            raise ValueError("recorder QoS must match the four configured topics")
    return [
        "ros2", "bag", "record",
        "--storage", "mcap",
        "--output", str(bag),
        "--max-cache-size", "268435456",
        "--disable-keyboard-controls",
        "--qos-profile-overrides-path", str(qos),
        "--topics", *_topics(config),
    ]


def solver_command(config_path: Path, bag_path: Path, output_path: Path) -> list[str]:
    """Build the unchanged offline solver command; caller must finish recording."""
    config = Path(config_path).expanduser().resolve()
    load_config(config)
    bag = Path(bag_path).expanduser().resolve()
    if not bag.is_dir():
        raise FileNotFoundError(f"bag directory does not exist: {bag}")
    output = _new_output(output_path)
    return [
        sys.executable, "-m", "vt_tracker_camera_calib.cli", "calibrate",
        "--config", str(config), "--bag", str(bag), "--output", str(output),
    ]


def compare_command(inputs: list[Path], output_path: Path) -> list[str]:
    """Build a repeatability command; identity and quality checks stay in the CLI."""
    if len(inputs) < 3:
        raise ValueError("repeatability comparison requires at least three results")
    paths = [Path(path).expanduser().resolve() for path in inputs]
    if len(set(paths)) != len(paths):
        raise ValueError("repeatability inputs must be distinct")
    for path in paths:
        if not path.is_file():
            raise FileNotFoundError(f"extrinsics file does not exist: {path}")
    output = _new_output(output_path)
    if output.suffix.lower() != ".json":
        raise ValueError("repeatability output must be a JSON file")
    return [
        sys.executable, "-m", "vt_tracker_camera_calib.cli", "compare",
        "--inputs", *(str(path) for path in paths), "--output", str(output),
    ]


class ManagedJob:
    """One owned process group with file logging and nonblocking graceful stop.

    Invoke from one GUI thread.  stop() sends SIGINT once; it neither waits nor
    escalates to terminate/kill, so a recorder can finalize its MCAP and metadata.
    No cleanup/destructor signals unrelated or previously completed processes.
    """

    def __init__(self) -> None:
        self._process: subprocess.Popen[bytes] | None = None
        self._stop_requested = False
        self.log_path: Path | None = None

    @property
    def running(self) -> bool:
        return self._process is not None and self.poll() is None

    @property
    def stop_requested(self) -> bool:
        return self._stop_requested

    @property
    def pid(self) -> int | None:
        return None if self._process is None else self._process.pid

    def start(self, argv: list[str], log_path: Path, cwd: Path | None = None) -> None:
        if self.running:
            raise RuntimeError("a managed job is already running")
        if not isinstance(argv, list) or not argv or any(
            not isinstance(arg, str) or not arg or "\0" in arg for arg in argv
        ):
            raise ValueError("argv must contain non-empty strings without NUL bytes")
        directory = None if cwd is None else Path(cwd).expanduser().resolve()
        if directory is not None and not directory.is_dir():
            raise FileNotFoundError(f"working directory does not exist: {directory}")
        target = _new_output(log_path)
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC
        flags |= getattr(os, "O_NOFOLLOW", 0)
        descriptor = os.open(target, flags, 0o600)
        # Child inherits the file descriptor; the parent's copy closes immediately.
        # No unread PIPE can fill and block a long recording or calibration run.
        with os.fdopen(descriptor, "wb") as log:
            process = subprocess.Popen(
                list(argv), stdin=subprocess.DEVNULL, stdout=log,
                stderr=subprocess.STDOUT, cwd=directory, shell=False,
                start_new_session=True,
                env={**os.environ, "PYTHONUNBUFFERED": "1"},
            )
        self._process = process
        self._stop_requested = False
        self.log_path = target

    def stop(self) -> None:
        process = self._process
        if process is None or self._stop_requested or process.poll() is not None:
            return
        try:
            # start_new_session makes our child the leader of this owned group.
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            # Child exited between poll and killpg; nothing else should be signalled.
            return
        self._stop_requested = True

    def poll(self) -> int | None:
        return None if self._process is None else self._process.poll()
