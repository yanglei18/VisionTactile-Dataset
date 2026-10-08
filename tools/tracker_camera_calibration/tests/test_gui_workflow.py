from pathlib import Path
import signal
import stat
import subprocess
import sys
import time
from unittest.mock import Mock

import pytest
import yaml

from vt_tracker_camera_calib.config import load_config
from vt_tracker_camera_calib.config_writer import build_config_document, write_config
from vt_tracker_camera_calib import gui_workflow as workflow


@pytest.fixture
def config_path(tmp_path: Path) -> Path:
    return workflow.prepare_config(tmp_path / "sessions", "d436", "right_wrist", 39.92, 29.94)


def test_prepare_unique_sessions_preserves_solver_thresholds(tmp_path: Path) -> None:
    first = workflow.prepare_config(tmp_path, "d436", "right_wrist", 39.92, 29.94)
    second = workflow.prepare_config(tmp_path, "d436", "right_wrist", 39.92, 29.94)
    assert first.parent != second.parent
    assert first.name == second.name == "config.yaml"
    config = load_config(first)
    assert config.camera_serial == "408322071716"
    assert config.tracker_sample_topic == "/vive/right_wrist/sample"
    assert config.board.square_length_m == pytest.approx(0.03992)
    assert config.board.min_corners == 12
    assert config.acceptance.min_pairs == 40
    assert stat.S_IMODE(first.stat().st_mode) == 0o600
    assert stat.S_IMODE(first.parent.stat().st_mode) == 0o700
    qos = first.with_name("recorder_qos.yaml")
    assert stat.S_IMODE(qos.stat().st_mode) == 0o600
    document = yaml.safe_load(qos.read_text())
    assert set(document) == {
        config.image_topic, config.camera_info_topic, config.timing_topic,
        config.tracker_sample_topic,
    }
    for policy in document.values():
        assert policy == {
            "history": "keep_last", "depth": 10,
            "reliability": "best_effort", "durability": "volatile",
        }


def test_prepare_custom_board(tmp_path: Path) -> None:
    path = workflow.prepare_config(
        tmp_path, "d405_1", "torso", 40, 30,
        squares_x=7, squares_y=5, dictionary="DICT_4X4_100",
    )
    board = load_config(path).board
    assert (board.squares_x, board.squares_y) == (7, 5)
    assert board.dictionary == "DICT_4X4_100"
    assert board.min_corners == 12


@pytest.mark.parametrize("overrides", [
    {"square_length_mm": True}, {"square_length_mm": float("nan")},
    {"square_length_mm": 0}, {"marker_length_mm": 40},
    {"squares_x": True}, {"squares_y": 6.0},
    {"squares_x": 3, "squares_y": 3}, {"dictionary": "NOT_A_DICTIONARY"},
    {"camera_name": "unknown"}, {"tracker_role": "unknown"},
])
def test_bad_settings_do_not_allocate_session(tmp_path: Path, overrides: dict) -> None:
    arguments = dict(
        output_root=tmp_path / "sessions", camera_name="d436", tracker_role="torso",
        square_length_mm=40, marker_length_mm=30,
    )
    arguments.update(overrides)
    with pytest.raises(ValueError):
        workflow.prepare_config(**arguments)
    assert not (tmp_path / "sessions").exists()


def test_import_snapshots_custom_camera_without_reference_restriction(tmp_path: Path) -> None:
    document = build_config_document(
        camera_name="d436", tracker_role="left_wrist",
        square_length_mm=40, marker_length_mm=30,
    )
    document["camera"].update(
        name="custom_head", serial="custom-serial", model="custom-rgb",
        frame_id="custom_head_optical_frame", image_topic="/custom_head/image",
        camera_info_topic="/custom_head/camera_info", timing_topic="/custom_head/timing",
    )
    source = write_config(tmp_path / "source.yaml", document)
    copied = workflow.import_config(tmp_path / "sessions", source)
    config = load_config(copied)
    assert config.camera_name == "custom_head"
    assert config.camera_serial == "custom-serial"
    before = copied.read_bytes()
    source.write_text("edited after import\n")
    assert copied.read_bytes() == before
    assert copied.with_name("recorder_qos.yaml").is_file()
    assert not copied.is_symlink()


def test_import_rejects_invalid_before_creating_output(tmp_path: Path) -> None:
    source = tmp_path / "bad.yaml"
    source.write_text("schema_version: 200\n")
    with pytest.raises(ValueError):
        workflow.import_config(tmp_path / "sessions", source)
    assert not (tmp_path / "sessions").exists()


@pytest.mark.parametrize("git_file", [False, True])
def test_reject_repository_and_linked_worktree_output(tmp_path: Path, git_file: bool) -> None:
    repository = tmp_path / "repository"
    repository.mkdir()
    marker = repository / ".git"
    if git_file:
        marker.write_text("gitdir: elsewhere\n")
    else:
        marker.mkdir()
    alias = tmp_path / "alias"
    alias.symlink_to(repository, target_is_directory=True)
    for root in (repository / "captures", alias / "captures"):
        with pytest.raises(ValueError, match="outside Git"):
            workflow.prepare_config(root, "d436", "torso", 40, 30)
    assert not (repository / "captures").exists()


def test_reject_relative_output() -> None:
    with pytest.raises(ValueError, match="absolute"):
        workflow.prepare_config(Path("sessions"), "d436", "torso", 40, 30)


def test_record_command_exact_topics_qos_and_no_capture(config_path: Path) -> None:
    config = load_config(config_path)
    bag = config_path.parent / "capture"
    qos = config_path.with_name("recorder_qos.yaml")
    original = qos.read_bytes()
    command = workflow.record_command(config, bag, qos_path=qos)
    assert command == [
        "ros2", "bag", "record", "--storage", "mcap", "--output", str(bag),
        "--max-cache-size", "268435456", "--disable-keyboard-controls",
        "--qos-profile-overrides-path", str(qos), "--topics",
        config.image_topic, config.camera_info_topic, config.timing_topic,
        config.tracker_sample_topic,
    ]
    assert not bag.exists()
    assert qos.read_bytes() == original
    # Preparing a command with an explicit config snapshot has no write side effect.
    assert workflow.record_command(config, bag, qos_path=qos) == command


def test_record_default_qos_is_exclusive(config_path: Path) -> None:
    config = load_config(config_path)
    bag = config_path.parent / "capture"
    command = workflow.record_command(config, bag)
    qos = bag.with_name("capture.qos.yaml")
    assert command[command.index("--qos-profile-overrides-path") + 1] == str(qos)
    assert qos.is_file()
    with pytest.raises(FileExistsError):
        workflow.record_command(config, bag)


def test_record_rejects_existing_bag_and_mismatched_qos(config_path: Path) -> None:
    config = load_config(config_path)
    bag = config_path.parent / "capture"
    bag.mkdir()
    with pytest.raises(FileExistsError):
        workflow.record_command(config, bag)
    assert not bag.with_name("capture.qos.yaml").exists()
    qos = config_path.parent / "wrong_qos.yaml"
    qos.write_text("{}\n")
    with pytest.raises(ValueError, match="four configured topics"):
        workflow.record_command(config, bag.with_name("new_capture"), qos_path=qos)


def test_record_rejects_dangling_output_symlink(config_path: Path) -> None:
    bag = config_path.parent / "capture"
    bag.symlink_to(config_path.parent / "nonexistent")
    with pytest.raises(FileExistsError):
        workflow.record_command(load_config(config_path), bag)


def test_solver_uses_same_python_and_unchanged_cli(config_path: Path) -> None:
    bag = config_path.parent / "capture"
    bag.mkdir()
    output = config_path.parent / "result"
    assert workflow.solver_command(config_path, bag, output) == [
        sys.executable, "-m", "vt_tracker_camera_calib.cli", "calibrate",
        "--config", str(config_path), "--bag", str(bag), "--output", str(output),
    ]
    assert not output.exists()
    output.mkdir()
    with pytest.raises(FileExistsError):
        workflow.solver_command(config_path, bag, output)
    with pytest.raises(FileNotFoundError):
        workflow.solver_command(config_path, bag / "missing", output.with_name("other"))


def test_compare_builds_command_without_loading_or_changing_results(tmp_path: Path) -> None:
    inputs = [tmp_path / f"result_{index}.yaml" for index in range(3)]
    for path in inputs:
        path.write_text("solver validates these files\n")
    output = tmp_path / "repeatability.json"
    assert workflow.compare_command(inputs, output) == [
        sys.executable, "-m", "vt_tracker_camera_calib.cli", "compare",
        "--inputs", *(str(path) for path in inputs), "--output", str(output),
    ]
    assert not output.exists()
    with pytest.raises(ValueError, match="three"):
        workflow.compare_command(inputs[:2], output)
    with pytest.raises(ValueError, match="distinct"):
        workflow.compare_command([inputs[0]] * 3, output)
    with pytest.raises(ValueError, match="JSON"):
        workflow.compare_command(inputs, output.with_suffix(".yaml"))
    output.write_text("keep")
    with pytest.raises(FileExistsError):
        workflow.compare_command(inputs, output)
    assert output.read_text() == "keep"


@pytest.fixture
def process_mocks(monkeypatch):
    process = Mock(pid=43210)
    process.poll.return_value = None
    popen = Mock(return_value=process)
    killpg = Mock()
    monkeypatch.setattr(workflow.subprocess, "Popen", popen)
    monkeypatch.setattr(workflow.os, "killpg", killpg)
    return process, popen, killpg


def test_job_owns_session_uses_file_not_pipe_and_stops_once(tmp_path: Path, process_mocks) -> None:
    process, popen, killpg = process_mocks
    job = workflow.ManagedJob()
    assert not job.running and job.poll() is None and job.pid is None
    job.stop()
    killpg.assert_not_called()
    log = tmp_path / "job.log"
    job.start(["ros2", "bag", "record"], log, cwd=tmp_path)
    assert job.running and job.pid == process.pid and job.log_path == log
    arguments = popen.call_args.kwargs
    assert arguments["start_new_session"] is True
    assert arguments["shell"] is False
    assert arguments["stdin"] == subprocess.DEVNULL
    assert arguments["stderr"] == subprocess.STDOUT
    assert arguments["stdout"] != subprocess.PIPE
    assert arguments["stdout"].closed
    assert arguments["cwd"] == tmp_path
    assert arguments["env"]["PYTHONUNBUFFERED"] == "1"
    assert stat.S_IMODE(log.stat().st_mode) == 0o600
    with pytest.raises(RuntimeError, match="already running"):
        job.start(["other"], tmp_path / "other.log")
    assert not (tmp_path / "other.log").exists()
    job.stop()
    job.stop()
    killpg.assert_called_once_with(process.pid, signal.SIGINT)
    assert job.stop_requested and job.running
    process.wait.assert_not_called()
    process.terminate.assert_not_called()
    process.kill.assert_not_called()
    process.poll.return_value = 0
    assert not job.running and job.poll() == 0
    job.stop()
    killpg.assert_called_once()


def test_job_completed_pid_is_never_signalled_and_restart_resets_stop(tmp_path: Path, process_mocks) -> None:
    process, popen, killpg = process_mocks
    job = workflow.ManagedJob()
    job.start(["first"], tmp_path / "first.log")
    process.poll.return_value = 0
    job.stop()
    killpg.assert_not_called()
    second = Mock(pid=43211)
    second.poll.return_value = None
    popen.return_value = second
    job.start(["second"], tmp_path / "second.log")
    assert job.pid == second.pid and not job.stop_requested
    job.stop()
    killpg.assert_called_once_with(second.pid, signal.SIGINT)


def test_job_handles_exit_race_but_does_not_hide_permission_error(tmp_path: Path, process_mocks) -> None:
    _, _, killpg = process_mocks
    job = workflow.ManagedJob()
    job.start(["test"], tmp_path / "job.log")
    killpg.side_effect = ProcessLookupError
    job.stop()
    assert not job.stop_requested
    killpg.side_effect = PermissionError("not ours")
    with pytest.raises(PermissionError):
        job.stop()
    assert not job.stop_requested


@pytest.mark.parametrize("argv", [[], "not-an-argv", [""], ["bad\0argument"], [1]])
def test_job_rejects_bad_argv_without_creating_log(tmp_path: Path, process_mocks, argv) -> None:
    _, popen, _ = process_mocks
    log = tmp_path / "job.log"
    with pytest.raises(ValueError, match="argv"):
        workflow.ManagedJob().start(argv, log)
    assert not log.exists()
    popen.assert_not_called()


def test_job_refuses_log_overwrite_and_cleans_parent_fd_on_launch_failure(tmp_path: Path, process_mocks) -> None:
    _, popen, killpg = process_mocks
    log = tmp_path / "job.log"
    log.write_text("keep")
    job = workflow.ManagedJob()
    with pytest.raises(FileExistsError):
        job.start(["example"], log)
    assert log.read_text() == "keep"
    popen.assert_not_called()
    popen.side_effect = FileNotFoundError("executable missing")
    with pytest.raises(FileNotFoundError):
        job.start(["missing"], tmp_path / "failed.log")
    assert popen.call_args.kwargs["stdout"].closed
    assert not job.running
    job.stop()
    killpg.assert_not_called()


def test_real_benign_job_large_file_log_does_not_block(tmp_path: Path) -> None:
    # A short Python-only child; no ROS, device, recorder or solver is started.
    job = workflow.ManagedJob()
    log = tmp_path / "large.log"
    job.start([sys.executable, "-c", "print('x' * 262144)"], log)
    deadline = time.monotonic() + 5
    try:
        while job.running and time.monotonic() < deadline:
            time.sleep(0.01)
        assert job.poll() == 0
        assert log.stat().st_size == 262145
    finally:
        if job.running:
            job.stop()
