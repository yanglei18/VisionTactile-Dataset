import importlib.util
import json
import os
import signal
import stat
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace

import pytest
import yaml

from vt_realsense_capture.bag_contract import (
    CAMERA_TOPIC_SUFFIXES,
    SCHEMA_VERSION,
    SYSTEM_TOPICS,
    expected_topic_type,
    expected_topic_types,
    expected_topics,
)
from vt_realsense_capture.recorder import (
    RecorderProcess,
    build_record_command,
    required_topics,
    write_qos_overrides,
)


CAMERA_NAMES = ("d405_1", "d405_2", "d436")
EXPECTED_TOPICS = (
    "/d405_1/color/image_raw",
    "/d405_1/depth/image_rect_raw",
    "/d405_1/color/camera_info",
    "/d405_1/frame_timing",
    "/d405_2/color/image_raw",
    "/d405_2/depth/image_rect_raw",
    "/d405_2/color/camera_info",
    "/d405_2/frame_timing",
    "/d436/color/image_raw",
    "/d436/depth/image_rect_raw",
    "/d436/color/camera_info",
    "/d436/frame_timing",
    "/vive/left_wrist/sample",
    "/vive/right_wrist/sample",
    "/vive/torso/sample",
)
PACKAGE_ROOT = Path(__file__).parents[1]
MCAP_WRITER_OPTIONS = PACKAGE_ROOT / "config" / "mcap_writer_options.yaml"
STORAGE_BENCH_SCRIPT = PACKAGE_ROOT / "scripts" / "storage_bench.py"
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]


def test_bag_contract_constants_are_exact_unified_dataset_schema() -> None:
    assert CAMERA_TOPIC_SUFFIXES == (
        "color/image_raw",
        "depth/image_rect_raw",
        "color/camera_info",
        "frame_timing",
    )
    assert SYSTEM_TOPICS == ()
    assert SCHEMA_VERSION == "unified-dataset-v1"


def test_required_topics_are_exact_unified_dataset_contract() -> None:
    assert required_topics(CAMERA_NAMES) == EXPECTED_TOPICS
    assert tuple(expected_topic_types(CAMERA_NAMES)) == EXPECTED_TOPICS
    assert len(set(EXPECTED_TOPICS)) == 15


@pytest.mark.parametrize(
    ("topic", "topic_type"),
    [
        pytest.param(
            "/camera/color/image_raw",
            "sensor_msgs/msg/Image",
            id="color-image",
        ),
        pytest.param(
            "/camera/depth/image_rect_raw",
            "sensor_msgs/msg/Image",
            id="depth-image",
        ),
        pytest.param(
            "/camera/frame_timing",
            "vt_camera_msgs/msg/CameraFrameTiming",
            id="frame-timing",
        ),
        pytest.param(
            "/camera/color/camera_info",
            "sensor_msgs/msg/CameraInfo",
            id="camera-info",
        ),
        pytest.param(
            "/vive/left_wrist/sample",
            "vt_tracker_msgs/msg/TrackerSample",
            id="tracker-sample",
        ),
    ],
)
def test_expected_topic_type_accepts_exact_structural_contract(
    topic: str, topic_type: str
) -> None:
    assert expected_topic_type(topic) == topic_type


@pytest.mark.parametrize(
    "topic",
    [
        pytest.param(
            "/d405_1/color/frame_timing", id="legacy-color-timing"
        ),
        pytest.param(
            "/d405_1/depth/frame_timing", id="legacy-depth-timing"
        ),
        pytest.param("/camera/color/metadata", id="metadata"),
        pytest.param("/camera/extrinsics/depth_to_color", id="extrinsics"),
        pytest.param("/tf_static", id="system-topic"),
        pytest.param(
            "/foo/bar/color/image_raw", id="nested-camera-namespace"
        ),
        pytest.param("camera/frame_timing", id="relative"),
        pytest.param("//camera/frame_timing", id="empty-namespace"),
        pytest.param("/1camera/frame_timing", id="unsafe-namespace"),
        pytest.param("/camera/frame_clock", id="unknown-suffix"),
        pytest.param("", id="empty"),
        pytest.param(1, id="non-string"),
    ],
)
def test_expected_topic_type_rejects_topics_outside_unified_dataset_structure(
    topic: object,
) -> None:
    with pytest.raises(ValueError, match="unsupported unified-dataset-v1 topic"):
        expected_topic_type(topic)  # type: ignore[arg-type]


def test_required_topics_are_deterministic_across_camera_order() -> None:
    assert required_topics(tuple(reversed(CAMERA_NAMES))) == required_topics(
        CAMERA_NAMES
    )


@pytest.mark.parametrize(
    "camera_names",
    [
        pytest.param((), id="empty-inventory"),
        pytest.param(("",), id="empty-name"),
        pytest.param(("d405_1", "d405_1"), id="duplicate"),
        pytest.param((1,), id="non-string"),
        pytest.param(("/d405_1",), id="absolute-name"),
        pytest.param(("d405_1/color",), id="namespace-injection"),
        pytest.param(("d405*",), id="wildcard-star"),
        pytest.param(("d405?",), id="wildcard-question"),
        pytest.param(("d405 {name}",), id="whitespace-and-substitution"),
        pytest.param(("~d405",), id="private-name"),
        pytest.param(("1camera",), id="leading-digit"),
    ],
)
def test_required_topics_reject_invalid_camera_names(
    camera_names: tuple[object, ...],
) -> None:
    with pytest.raises(ValueError, match="camera"):
        required_topics(camera_names)  # type: ignore[arg-type]


def test_mcap_writer_options_lock_uncompressed_chunked_storage() -> None:
    document = yaml.safe_load(MCAP_WRITER_OPTIONS.read_text())
    assert document["noChunkCRC"] is False
    assert document["noChunking"] is False
    assert document["chunkSize"] == 16777216
    assert document["compression"] == "None"


def test_qos_overrides_are_best_effort_depth_30(tmp_path: Path) -> None:
    path = write_qos_overrides(tmp_path / "qos.yaml", EXPECTED_TOPICS)
    document = yaml.safe_load(path.read_text())
    assert tuple(document) == EXPECTED_TOPICS
    assert set(tuple(profile.items()) for profile in document.values()) == {
        (
            ("history", "keep_last"),
            ("depth", 30),
            ("reliability", "best_effort"),
            ("durability", "volatile"),
        )
    }


@pytest.mark.parametrize(
    "topics",
    [
        pytest.param((), id="empty"),
        pytest.param(("/camera/frame_timing", "/camera/frame_timing"), id="duplicate"),
        pytest.param(("camera/frame_timing",), id="relative"),
        pytest.param(("/capture/*",), id="wildcard"),
        pytest.param(("/capture/{name}",), id="substitution"),
        pytest.param((1,), id="non-string"),
    ],
)
def test_qos_overrides_reject_invalid_topic_lists(
    tmp_path: Path, topics: tuple[object, ...]
) -> None:
    path = tmp_path / "qos_overrides.yaml"

    with pytest.raises(ValueError, match="topic"):
        write_qos_overrides(path, topics)  # type: ignore[arg-type]

    assert not path.exists()


def test_qos_overrides_reject_relative_target(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)

    with pytest.raises(ValueError, match="absolute"):
        write_qos_overrides(Path("qos_overrides.yaml"), ("/camera/frame_timing",))

    assert not (tmp_path / "qos_overrides.yaml").exists()


def test_qos_overrides_refuse_unrelated_existing_file(tmp_path: Path) -> None:
    path = tmp_path / "qos_overrides.yaml"
    path.write_text("belongs to someone else\n")

    with pytest.raises(FileExistsError, match="unrelated"):
        write_qos_overrides(path, ("/camera/frame_timing",))

    assert path.read_text() == "belongs to someone else\n"


def test_qos_overrides_refuse_symlink_target(tmp_path: Path) -> None:
    victim = tmp_path / "victim.yaml"
    victim.write_text("must survive\n")
    path = tmp_path / "qos_overrides.yaml"
    path.symlink_to(victim)

    with pytest.raises(ValueError, match="symlink"):
        write_qos_overrides(path, ("/camera/frame_timing",))

    assert victim.read_text() == "must survive\n"


def test_qos_overrides_publish_by_no_overwrite_link_and_fsync_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "qos_overrides.yaml"
    link_calls: list[tuple[Path, Path, bool]] = []
    fsync_directory_facts: list[bool] = []
    real_link = os.link
    real_fsync = os.fsync

    def tracked_link(
        source: object, target: object, *, follow_symlinks: bool = True
    ) -> None:
        link_calls.append((Path(source), Path(target), follow_symlinks))
        real_link(source, target, follow_symlinks=follow_symlinks)

    def tracked_fsync(descriptor: int) -> None:
        fsync_directory_facts.append(
            stat.S_ISDIR(os.fstat(descriptor).st_mode)
        )
        real_fsync(descriptor)

    monkeypatch.setattr(os, "link", tracked_link)
    monkeypatch.setattr(os, "fsync", tracked_fsync)

    write_qos_overrides(path, ("/camera/frame_timing",))

    assert len(link_calls) == 1
    temporary, destination, followed = link_calls[0]
    assert temporary.parent == destination.parent == tmp_path
    assert temporary != destination == path
    assert followed is False
    assert fsync_directory_facts == [False, True]
    assert path.read_text() == (
        "/camera/frame_timing:\n"
        "  history: keep_last\n"
        "  depth: 30\n"
        "  reliability: best_effort\n"
        "  durability: volatile\n"
    )
    assert tuple(tmp_path.iterdir()) == (path,)


@pytest.mark.parametrize("racer_kind", ["file", "symlink"])
def test_qos_overrides_never_clobber_racer_created_destination(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    racer_kind: str,
) -> None:
    path = tmp_path / "qos_overrides.yaml"
    victim = tmp_path / "victim.yaml"
    victim.write_text("victim survives\n")
    real_link = os.link

    def racing_link(
        source: object, target: object, *, follow_symlinks: bool = True
    ) -> None:
        destination = Path(target)
        if racer_kind == "file":
            destination.write_text("racer owns this\n")
        else:
            destination.symlink_to(victim)
        real_link(source, target, follow_symlinks=follow_symlinks)

    monkeypatch.setattr(os, "link", racing_link)

    with pytest.raises(FileExistsError):
        write_qos_overrides(path, ("/camera/frame_timing",))

    assert victim.read_text() == "victim survives\n"
    if racer_kind == "file":
        assert path.read_text() == "racer owns this\n"
    else:
        assert path.is_symlink()
        assert path.resolve() == victim
    assert tuple(tmp_path.glob(".qos_overrides.yaml.*.tmp")) == ()


def test_qos_overrides_link_failure_cleans_owned_temporary(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "qos_overrides.yaml"

    def failed_link(
        source: object, target: object, *, follow_symlinks: bool = True
    ) -> None:
        raise OSError("link failed")

    monkeypatch.setattr(os, "link", failed_link)

    with pytest.raises(OSError, match="link failed"):
        write_qos_overrides(path, ("/camera/frame_timing",))

    assert not path.exists()
    assert tuple(tmp_path.iterdir()) == ()


def test_record_command_is_exact_explicit_jazzy_argv(tmp_path: Path) -> None:
    output_path = tmp_path / "session-001"
    storage_config_path = tmp_path / "mcap_writer_options.yaml"
    storage_config_path.write_text(MCAP_WRITER_OPTIONS.read_text())
    topics = required_topics(CAMERA_NAMES)
    qos_path = tmp_path / "qos_overrides.yaml"
    write_qos_overrides(qos_path, topics)

    command = build_record_command(
        output_path=output_path,
        storage_config_path=storage_config_path,
        qos_overrides_path=qos_path,
        topics=topics,
    )

    assert command == [
        "ros2",
        "bag",
        "record",
        "--storage",
        "mcap",
        "--output",
        str(output_path),
        "--storage-config-file",
        str(storage_config_path),
        "--qos-profile-overrides-path",
        str(qos_path),
        "--max-bag-duration",
        "300",
        "--max-bag-size",
        "137438953472",
        "--max-cache-size",
        "1073741824",
        "--disable-keyboard-controls",
        "--include-unpublished-topics",
        "--node-name",
        "vt_rosbag_recorder",
        "--topics",
        *EXPECTED_TOPICS,
    ]
    forbidden = {
        "--all",
        "--regex",
        "--start-paused",
        "--compression-mode",
        "--compression-format",
        "--compression-threads",
        "--compression-queue-size",
    }
    assert forbidden.isdisjoint(command)


def _valid_record_command_arguments(tmp_path: Path) -> dict[str, object]:
    storage = tmp_path / "mcap_writer_options.yaml"
    storage.write_text(MCAP_WRITER_OPTIONS.read_text())
    topics = required_topics(CAMERA_NAMES)
    qos = tmp_path / "qos_overrides.yaml"
    write_qos_overrides(qos, topics)
    return {
        "output_path": tmp_path / "session",
        "storage_config_path": storage,
        "qos_overrides_path": qos,
        "topics": topics,
    }


@pytest.mark.parametrize(
    ("field", "relative"),
    [
        pytest.param("output_path", "session", id="output"),
        pytest.param(
            "storage_config_path", "mcap_writer_options.yaml", id="storage"
        ),
        pytest.param("qos_overrides_path", "qos_overrides.yaml", id="qos"),
    ],
)
def test_record_command_rejects_relative_paths(
    tmp_path: Path, field: str, relative: str
) -> None:
    arguments = _valid_record_command_arguments(tmp_path)
    arguments[field] = Path(relative)

    with pytest.raises(ValueError, match="absolute"):
        build_record_command(**arguments)  # type: ignore[arg-type]


@pytest.mark.parametrize("field", ["storage_config_path", "qos_overrides_path"])
def test_record_command_requires_existing_regular_config_files(
    tmp_path: Path, field: str
) -> None:
    arguments = _valid_record_command_arguments(tmp_path)
    arguments[field] = tmp_path / f"missing-{field}.yaml"

    with pytest.raises(FileNotFoundError, match="config"):
        build_record_command(**arguments)  # type: ignore[arg-type]


@pytest.mark.parametrize("field", ["storage_config_path", "qos_overrides_path"])
def test_record_command_rejects_symlinked_config_files(
    tmp_path: Path, field: str
) -> None:
    arguments = _valid_record_command_arguments(tmp_path)
    target = arguments[field]
    symlink = tmp_path / f"linked-{field}.yaml"
    symlink.symlink_to(target)
    arguments[field] = symlink

    with pytest.raises(ValueError, match="symlink"):
        build_record_command(**arguments)  # type: ignore[arg-type]


def test_record_command_refuses_existing_output(tmp_path: Path) -> None:
    arguments = _valid_record_command_arguments(tmp_path)
    output = arguments["output_path"]
    assert isinstance(output, Path)
    output.mkdir()

    with pytest.raises(FileExistsError, match="output"):
        build_record_command(**arguments)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "topics",
    [
        pytest.param((), id="empty"),
        pytest.param(("/camera/frame_timing", "/camera/frame_timing"), id="duplicate"),
        pytest.param(("/capture/*",), id="wildcard"),
        pytest.param(("/cam/extra/color/image_raw",), id="nested-injection"),
        pytest.param("/camera/frame_timing", id="single-string"),
    ],
)
def test_record_command_requires_explicit_valid_topics(
    tmp_path: Path, topics: object
) -> None:
    arguments = _valid_record_command_arguments(tmp_path)
    arguments["topics"] = topics

    with pytest.raises(ValueError, match="topic"):
        build_record_command(**arguments)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "topics",
    [
        pytest.param(("/d405_1/frame_timing",), id="camera-subset"),
        pytest.param(
            tuple(reversed(required_topics(CAMERA_NAMES))),
            id="reversed-canonical-order",
        ),
        pytest.param(
            required_topics(CAMERA_NAMES)[:-1], id="missing-camera-topic"
        ),
        pytest.param(
            (
                "/other/color/image_raw",
                *required_topics(CAMERA_NAMES)[1:],
            ),
            id="mixed-fourth-camera-topic",
        ),
        pytest.param(
            tuple(expected_topics(("d405_1",))),
            id="complete-one-camera-contract",
        ),
        pytest.param(
            tuple(expected_topics(("d405_1", "d405_2"))),
            id="complete-two-camera-contract",
        ),
    ],
)
def test_record_command_rejects_noncanonical_contract_selections(
    tmp_path: Path, topics: tuple[str, ...]
) -> None:
    arguments = _valid_record_command_arguments(tmp_path)
    arguments["topics"] = topics

    with pytest.raises(
        ValueError, match="three-camera unified dataset core|canonical unified dataset core"
    ):
        build_record_command(**arguments)  # type: ignore[arg-type]


def test_additional_glove_topics_extend_contract_deterministically(
    tmp_path: Path,
) -> None:
    topics = required_topics(
        CAMERA_NAMES,
        ("/gloves/right/state", "/gloves/left/state"),
    )
    assert topics[-2:] == ("/gloves/left/state", "/gloves/right/state")
    storage = tmp_path / "mcap_writer_options.yaml"
    storage.write_text(MCAP_WRITER_OPTIONS.read_text())
    qos = tmp_path / "qos.yaml"
    write_qos_overrides(qos, topics)
    command = build_record_command(
        output_path=tmp_path / "bag",
        storage_config_path=storage,
        qos_overrides_path=qos,
        topics=topics,
    )
    assert command[-2:] == ["/gloves/left/state", "/gloves/right/state"]


def test_additional_topic_may_share_a_core_suffix_without_becoming_a_camera(
    tmp_path: Path,
) -> None:
    topics = required_topics(CAMERA_NAMES, ("/glove/color/image_raw",))
    storage = tmp_path / "mcap_writer_options.yaml"
    storage.write_text(MCAP_WRITER_OPTIONS.read_text())
    qos = tmp_path / "qos.yaml"
    write_qos_overrides(qos, topics)

    command = build_record_command(
        output_path=tmp_path / "bag",
        storage_config_path=storage,
        qos_overrides_path=qos,
        topics=topics,
    )

    assert command[-1] == "/glove/color/image_raw"


class FakeProcess:
    def __init__(
        self,
        *,
        pid: int = 4321,
        poll_result: int | None = None,
        wait_actions: tuple[object, ...] = (0,),
    ) -> None:
        self.pid = pid
        self.poll_result = poll_result
        self.wait_actions = list(wait_actions)
        self.wait_timeouts: list[float] = []

    def poll(self) -> int | None:
        return self.poll_result

    def wait(self, timeout: float) -> int:
        self.wait_timeouts.append(timeout)
        action = self.wait_actions.pop(0)
        if isinstance(action, BaseException):
            raise action
        self.poll_result = int(action)
        return self.poll_result


class PollFailureProcess(FakeProcess):
    def poll(self) -> int | None:
        raise OSError("poll failed")


def test_recorder_process_owns_clean_start_and_sigint_stop(tmp_path: Path) -> None:
    process = FakeProcess()
    popen_calls: list[tuple[list[str], dict[str, object]]] = []
    kill_calls: list[tuple[int, signal.Signals]] = []

    def fake_popen(command: list[str], **kwargs: object) -> FakeProcess:
        popen_calls.append((command, kwargs))
        return process

    def fake_killpg(pid: int, sig: signal.Signals) -> None:
        kill_calls.append((pid, sig))

    command = ["ros2", "bag", "record", "--topics", "/capture/event"]
    recorder = RecorderProcess(
        command,
        session_parent=tmp_path,
        popen_factory=fake_popen,
        killpg=fake_killpg,
    )

    assert recorder.start() == process.pid
    assert recorder.pid == process.pid
    assert recorder.poll() is None
    assert recorder.alive is True
    assert len(popen_calls) == 1
    started_command, kwargs = popen_calls[0]
    assert started_command == command
    assert started_command is not command
    assert kwargs["start_new_session"] is True
    assert kwargs["stderr"] is subprocess.STDOUT
    assert "shell" not in kwargs
    log_stream = kwargs["stdout"]
    assert getattr(log_stream, "name") == str(tmp_path / "recorder.log")
    assert getattr(log_stream, "closed") is False

    with pytest.raises(RuntimeError, match="once"):
        recorder.start()

    result = recorder.stop()

    assert result.valid is True
    assert result.clean is True
    assert result.returncode == 0
    assert result.reason == "clean_exit"
    assert result.termination_confirmed is True
    assert result.possibly_alive is False
    assert kill_calls == [(process.pid, signal.SIGINT)]
    assert process.wait_timeouts == [60.0]
    assert getattr(log_stream, "closed") is True
    assert recorder.alive is False
    assert recorder.stop() is result


def _start_fake_recorder(
    tmp_path: Path,
    process: FakeProcess,
) -> tuple[RecorderProcess, list[tuple[int, signal.Signals]]]:
    kill_calls: list[tuple[int, signal.Signals]] = []

    def fake_popen(command: list[str], **kwargs: object) -> FakeProcess:
        return process

    def fake_killpg(pid: int, sig: signal.Signals) -> None:
        kill_calls.append((pid, sig))

    recorder = RecorderProcess(
        ["ros2", "bag", "record", "--topics", "/capture/event"],
        session_parent=tmp_path,
        popen_factory=fake_popen,
        killpg=fake_killpg,
    )
    recorder.start()
    return recorder, kill_calls


def test_recorder_process_marks_nonzero_sigint_exit_invalid(tmp_path: Path) -> None:
    process = FakeProcess(wait_actions=(17,))
    recorder, kill_calls = _start_fake_recorder(tmp_path, process)

    result = recorder.stop()

    assert result.valid is False
    assert result.clean is False
    assert result.returncode == 17
    assert result.reason == "nonzero_exit"
    assert result.termination_confirmed is True
    assert result.possibly_alive is False
    assert kill_calls == [(process.pid, signal.SIGINT)]
    assert process.wait_timeouts == [60.0]
    assert recorder.alive is False


@pytest.mark.parametrize(
    "returncode",
    [
        pytest.param(0, id="zero"),
        pytest.param(9, id="nonzero"),
    ],
)
def test_recorder_process_marks_pre_stop_exit_unexpected_without_signal(
    tmp_path: Path, returncode: int
) -> None:
    process = FakeProcess(poll_result=returncode, wait_actions=())
    recorder, kill_calls = _start_fake_recorder(tmp_path, process)

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode == returncode
    assert result.reason == "unexpected_exit"
    assert result.termination_confirmed is True
    assert result.possibly_alive is False
    assert kill_calls == []
    assert process.wait_timeouts == []
    assert recorder.alive is False


def _wait_timeout(seconds: float) -> subprocess.TimeoutExpired:
    return subprocess.TimeoutExpired(
        cmd=["ros2", "bag", "record"], timeout=seconds
    )


def test_recorder_process_sigint_timeout_terminates_and_is_invalid(
    tmp_path: Path,
) -> None:
    process = FakeProcess(wait_actions=(_wait_timeout(60.0), 0))
    recorder, kill_calls = _start_fake_recorder(tmp_path, process)

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode == 0
    assert result.reason == "sigint_timeout"
    assert kill_calls == [
        (process.pid, signal.SIGINT),
        (process.pid, signal.SIGTERM),
    ]
    assert process.wait_timeouts == [60.0, 10.0]
    assert recorder.alive is False


def test_recorder_process_second_timeout_kills_and_reaps_bounded(
    tmp_path: Path,
) -> None:
    process = FakeProcess(
        wait_actions=(
            _wait_timeout(60.0),
            _wait_timeout(10.0),
            -signal.SIGKILL,
        )
    )
    recorder, kill_calls = _start_fake_recorder(tmp_path, process)

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode == -signal.SIGKILL
    assert result.reason == "forced_kill"
    assert kill_calls == [
        (process.pid, signal.SIGINT),
        (process.pid, signal.SIGTERM),
        (process.pid, signal.SIGKILL),
    ]
    assert process.wait_timeouts == [60.0, 10.0, 10.0]
    assert recorder.alive is False


@pytest.mark.parametrize(
    "command",
    [
        pytest.param("ros2 bag record", id="shell-string"),
        pytest.param([], id="empty"),
        pytest.param(["ros2", 1], id="non-string"),
        pytest.param(["ros2", "bag\x00record"], id="nul-byte"),
    ],
)
def test_recorder_process_rejects_non_argv_commands(
    tmp_path: Path, command: object
) -> None:
    with pytest.raises(ValueError, match="command"):
        RecorderProcess(
            command,  # type: ignore[arg-type]
            session_parent=tmp_path,
        )


def test_recorder_process_requires_absolute_canonical_session_parent(
    tmp_path: Path,
) -> None:
    linked_parent = tmp_path / "linked"
    linked_parent.symlink_to(tmp_path, target_is_directory=True)

    with pytest.raises(ValueError, match="absolute"):
        RecorderProcess(["ros2"], session_parent=Path("relative"))
    with pytest.raises(ValueError, match="symlink"):
        RecorderProcess(["ros2"], session_parent=linked_parent)
    with pytest.raises(ValueError, match="directory"):
        RecorderProcess(
            ["ros2"], session_parent=tmp_path / "missing-directory"
        )


def test_recorder_process_refuses_existing_unowned_log(tmp_path: Path) -> None:
    log_path = tmp_path / "recorder.log"
    log_path.write_text("someone else's log\n")
    popen_called = False

    def fake_popen(command: list[str], **kwargs: object) -> FakeProcess:
        nonlocal popen_called
        popen_called = True
        return FakeProcess()

    recorder = RecorderProcess(
        ["ros2"], session_parent=tmp_path, popen_factory=fake_popen
    )

    with pytest.raises(FileExistsError):
        recorder.start()

    assert popen_called is False
    assert log_path.read_text() == "someone else's log\n"


def test_recorder_process_popen_error_closes_and_removes_owned_log(
    tmp_path: Path,
) -> None:
    captured_streams: list[object] = []

    def failed_popen(command: list[str], **kwargs: object) -> FakeProcess:
        captured_streams.append(kwargs["stdout"])
        raise OSError("popen failed")

    recorder = RecorderProcess(
        ["ros2"], session_parent=tmp_path, popen_factory=failed_popen
    )

    with pytest.raises(OSError, match="popen failed"):
        recorder.start()

    assert len(captured_streams) == 1
    assert getattr(captured_streams[0], "closed") is True
    assert not (tmp_path / "recorder.log").exists()
    assert recorder.pid is None
    assert recorder.alive is False


def _start_recorder_with_controls(
    tmp_path: Path,
    process: FakeProcess,
    killpg: object,
) -> tuple[RecorderProcess, list[object]]:
    streams: list[object] = []

    def fake_popen(command: list[str], **kwargs: object) -> FakeProcess:
        streams.append(kwargs["stdout"])
        return process

    recorder = RecorderProcess(
        ["ros2"],
        session_parent=tmp_path,
        popen_factory=fake_popen,
        killpg=killpg,  # type: ignore[arg-type]
    )
    recorder.start()
    return recorder, streams


def test_recorder_process_sigint_signal_exception_remains_uncertain(
    tmp_path: Path,
) -> None:
    process = FakeProcess(wait_actions=(0,))
    kill_calls: list[tuple[int, signal.Signals]] = []

    def flaky_killpg(pid: int, sig: signal.Signals) -> None:
        kill_calls.append((pid, sig))
        if sig is signal.SIGINT:
            raise OSError("SIGINT failed")

    recorder, streams = _start_recorder_with_controls(
        tmp_path, process, flaky_killpg
    )

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode is None
    assert result.reason == "sigint_signal_failed"
    assert result.termination_confirmed is False
    assert result.possibly_alive is True
    assert kill_calls == [(process.pid, signal.SIGINT)]
    assert process.wait_timeouts == []
    assert recorder.alive is True
    assert getattr(streams[0], "closed") is False
    process.poll_result = -signal.SIGKILL
    assert recorder.stop().termination_confirmed is True
    assert getattr(streams[0], "closed") is True


def test_recorder_process_poll_exception_recovers_invalid(tmp_path: Path) -> None:
    process = PollFailureProcess(wait_actions=(0,))
    kill_calls: list[tuple[int, signal.Signals]] = []

    def fake_killpg(pid: int, sig: signal.Signals) -> None:
        kill_calls.append((pid, sig))

    recorder, streams = _start_recorder_with_controls(
        tmp_path, process, fake_killpg
    )

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode == 0
    assert result.reason == "poll_failed"
    assert kill_calls == [(process.pid, signal.SIGTERM)]
    assert process.wait_timeouts == [10.0]
    assert getattr(streams[0], "closed") is True


def test_recorder_process_sigint_wait_exception_remains_uncertain(
    tmp_path: Path,
) -> None:
    process = FakeProcess(wait_actions=(OSError("wait failed"), 0))
    kill_calls: list[tuple[int, signal.Signals]] = []

    def fake_killpg(pid: int, sig: signal.Signals) -> None:
        kill_calls.append((pid, sig))

    recorder, streams = _start_recorder_with_controls(
        tmp_path, process, fake_killpg
    )

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode is None
    assert result.reason == "sigint_wait_failed"
    assert result.termination_confirmed is False
    assert result.possibly_alive is True
    assert kill_calls == [(process.pid, signal.SIGINT)]
    assert process.wait_timeouts == [60.0]
    assert recorder.alive is True
    assert getattr(streams[0], "closed") is False
    process.poll_result = -signal.SIGKILL
    assert recorder.stop().termination_confirmed is True
    assert getattr(streams[0], "closed") is True


def test_recorder_process_sigterm_signal_exception_remains_uncertain(
    tmp_path: Path,
) -> None:
    process = FakeProcess(wait_actions=(_wait_timeout(60.0), -signal.SIGKILL))
    kill_calls: list[tuple[int, signal.Signals]] = []

    def flaky_killpg(pid: int, sig: signal.Signals) -> None:
        kill_calls.append((pid, sig))
        if sig is signal.SIGTERM:
            raise OSError("SIGTERM failed")

    recorder, streams = _start_recorder_with_controls(
        tmp_path, process, flaky_killpg
    )

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode is None
    assert result.reason == "sigterm_signal_failed"
    assert result.termination_confirmed is False
    assert result.possibly_alive is True
    assert kill_calls == [
        (process.pid, signal.SIGINT),
        (process.pid, signal.SIGTERM),
    ]
    assert process.wait_timeouts == [60.0]
    assert recorder.alive is True
    assert getattr(streams[0], "closed") is False
    process.poll_result = -signal.SIGKILL
    assert recorder.stop().termination_confirmed is True
    assert getattr(streams[0], "closed") is True


def test_recorder_process_sigterm_wait_exception_remains_uncertain(
    tmp_path: Path,
) -> None:
    process = FakeProcess(
        wait_actions=(
            _wait_timeout(60.0),
            OSError("SIGTERM wait failed"),
            -signal.SIGKILL,
        )
    )
    recorder, streams = _start_recorder_with_controls(
        tmp_path, process, lambda pid, sig: None
    )

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode is None
    assert result.reason == "sigterm_wait_failed"
    assert result.termination_confirmed is False
    assert result.possibly_alive is True
    assert process.wait_timeouts == [60.0, 10.0]
    assert recorder.alive is True
    assert getattr(streams[0], "closed") is False
    process.poll_result = -signal.SIGKILL
    assert recorder.stop().termination_confirmed is True
    assert getattr(streams[0], "closed") is True


def test_recorder_process_sigkill_signal_exception_is_invalid(
    tmp_path: Path,
) -> None:
    process = FakeProcess(
        wait_actions=(_wait_timeout(60.0), _wait_timeout(10.0))
    )

    def flaky_killpg(pid: int, sig: signal.Signals) -> None:
        if sig is signal.SIGKILL:
            raise OSError("SIGKILL failed")

    recorder, streams = _start_recorder_with_controls(
        tmp_path, process, flaky_killpg
    )

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode is None
    assert result.reason == "sigkill_signal_failed"
    assert result.termination_confirmed is False
    assert result.possibly_alive is True
    assert process.wait_timeouts == [60.0, 10.0]
    assert recorder.alive is True
    assert getattr(streams[0], "closed") is False
    process.poll_result = -signal.SIGKILL
    assert recorder.stop().termination_confirmed is True
    assert getattr(streams[0], "closed") is True


def test_recorder_process_sigkill_wait_exception_is_invalid(
    tmp_path: Path,
) -> None:
    process = FakeProcess(
        wait_actions=(
            _wait_timeout(60.0),
            _wait_timeout(10.0),
            OSError("SIGKILL wait failed"),
        )
    )
    recorder, streams = _start_recorder_with_controls(
        tmp_path, process, lambda pid, sig: None
    )

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode is None
    assert result.reason == "sigkill_wait_failed"
    assert result.termination_confirmed is False
    assert result.possibly_alive is True
    assert process.wait_timeouts == [60.0, 10.0, 10.0]
    assert recorder.alive is True
    assert getattr(streams[0], "closed") is False
    process.poll_result = -signal.SIGKILL
    assert recorder.stop().termination_confirmed is True
    assert getattr(streams[0], "closed") is True


def test_recorder_process_unconfirmed_sigkill_signal_is_retryable_until_exit(
    tmp_path: Path,
) -> None:
    process = FakeProcess(
        wait_actions=(
            _wait_timeout(60.0),
            _wait_timeout(10.0),
            0,
        )
    )
    kill_calls: list[tuple[int, signal.Signals]] = []
    fail_first_sigkill = True

    def flaky_killpg(pid: int, sig: signal.Signals) -> None:
        nonlocal fail_first_sigkill
        kill_calls.append((pid, sig))
        if sig is signal.SIGKILL and fail_first_sigkill:
            fail_first_sigkill = False
            raise OSError("SIGKILL delivery uncertain")

    recorder, streams = _start_recorder_with_controls(
        tmp_path, process, flaky_killpg
    )

    uncertain = recorder.stop()

    assert uncertain.valid is False
    assert uncertain.returncode is None
    assert uncertain.reason == "sigkill_signal_failed"
    assert uncertain.termination_confirmed is False
    assert uncertain.possibly_alive is True
    assert recorder.alive is True
    assert getattr(streams[0], "closed") is False

    terminal = recorder.stop()

    assert terminal is not uncertain
    assert terminal.valid is False
    assert terminal.returncode == 0
    assert terminal.reason == "sigkill_signal_failed"
    assert terminal.termination_confirmed is True
    assert terminal.possibly_alive is False
    assert recorder.alive is False
    assert getattr(streams[0], "closed") is True
    assert recorder.stop() is terminal


def test_recorder_process_repeated_sigkill_uncertainty_remains_nonterminal(
    tmp_path: Path,
) -> None:
    process = FakeProcess(
        wait_actions=(
            _wait_timeout(60.0),
            _wait_timeout(10.0),
            _wait_timeout(60.0),
            _wait_timeout(10.0),
        )
    )

    def failed_sigkill(pid: int, sig: signal.Signals) -> None:
        if sig is signal.SIGKILL:
            raise OSError("SIGKILL delivery uncertain")

    recorder, streams = _start_recorder_with_controls(
        tmp_path, process, failed_sigkill
    )

    first = recorder.stop()
    second = recorder.stop()

    assert second is not first
    for result in (first, second):
        assert result.valid is False
        assert result.returncode is None
        assert result.termination_confirmed is False
        assert result.possibly_alive is True
    assert process.wait_timeouts == [60.0, 10.0, 60.0, 10.0]
    assert recorder.alive is True
    assert getattr(streams[0], "closed") is False
    process.poll_result = -signal.SIGKILL
    assert recorder.stop().termination_confirmed is True
    assert getattr(streams[0], "closed") is True


def test_recorder_process_unconfirmed_final_wait_failure_keeps_ownership(
    tmp_path: Path,
) -> None:
    process = FakeProcess(
        wait_actions=(
            _wait_timeout(60.0),
            _wait_timeout(10.0),
            OSError("SIGKILL wait failed"),
        )
    )
    recorder, streams = _start_recorder_with_controls(
        tmp_path, process, lambda pid, sig: None
    )

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode is None
    assert result.reason == "sigkill_wait_failed"
    assert result.termination_confirmed is False
    assert result.possibly_alive is True
    assert recorder.alive is True
    assert getattr(streams[0], "closed") is False
    process.poll_result = -signal.SIGKILL
    assert recorder.stop().termination_confirmed is True
    assert getattr(streams[0], "closed") is True


def test_recorder_process_pre_stop_zero_exit_is_unexpected_and_invalid(
    tmp_path: Path,
) -> None:
    process = FakeProcess(poll_result=0, wait_actions=())
    recorder, streams = _start_recorder_with_controls(
        tmp_path, process, lambda pid, sig: pytest.fail("must not signal")
    )

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode == 0
    assert result.reason == "unexpected_exit"
    assert result.termination_confirmed is True
    assert result.possibly_alive is False
    assert recorder.alive is False
    assert getattr(streams[0], "closed") is True


def test_recorder_process_signal_error_polls_observed_exit_once(
    tmp_path: Path,
) -> None:
    process = FakeProcess(wait_actions=(0,))
    kill_calls: list[tuple[int, signal.Signals]] = []

    def uncertain_sigint(pid: int, sig: signal.Signals) -> None:
        kill_calls.append((pid, sig))
        process.poll_result = 0
        raise OSError("SIGINT result uncertain")

    recorder, streams = _start_recorder_with_controls(
        tmp_path, process, uncertain_sigint
    )

    result = recorder.stop()

    assert result.valid is False
    assert result.returncode == 0
    assert result.reason == "sigint_signal_failed"
    assert result.termination_confirmed is True
    assert result.possibly_alive is False
    assert kill_calls == [(process.pid, signal.SIGINT)]
    assert process.wait_timeouts == []
    assert getattr(streams[0], "closed") is True


def _load_storage_bench() -> ModuleType:
    module_name = "vt_storage_bench_test_module"
    spec = importlib.util.spec_from_file_location(module_name, STORAGE_BENCH_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def test_fio_command_is_exact_sequential_direct_4mib_16gib_argv(
    tmp_path: Path,
) -> None:
    storage_bench = _load_storage_bench()

    command = storage_bench.build_fio_command(tmp_path)

    assert command == [
        "fio",
        "--name=vt_storage_bench",
        f"--filename={tmp_path / '.vt_storage_bench.data'}",
        "--rw=write",
        "--bs=4M",
        "--size=16G",
        "--direct=1",
        "--ioengine=libaio",
        "--iodepth=1",
        "--numjobs=1",
        "--group_reporting=1",
        "--output-format=json",
    ]
    assert isinstance(command, list)
    assert all(isinstance(argument, str) for argument in command)
    assert not any(argument in {"sh", "bash", "-c"} for argument in command)


def test_fio_command_rejects_relative_root() -> None:
    storage_bench = _load_storage_bench()

    with pytest.raises(ValueError, match="absolute"):
        storage_bench.build_fio_command(Path("relative-output"))


@pytest.mark.parametrize(
    "root",
    [
        pytest.param(Path("/"), id="filesystem-root"),
        pytest.param(REPOSITORY_ROOT, id="repository-root"),
        pytest.param(PACKAGE_ROOT, id="inside-repository"),
    ],
)
def test_fio_command_rejects_unsafe_roots(root: Path) -> None:
    storage_bench = _load_storage_bench()

    with pytest.raises(ValueError, match="unsafe|repository"):
        storage_bench.build_fio_command(root)


def test_fio_command_requires_real_nonsymlink_directory(tmp_path: Path) -> None:
    storage_bench = _load_storage_bench()
    regular_file = tmp_path / "file"
    regular_file.write_text("not a directory")
    linked_root = tmp_path / "linked"
    linked_root.symlink_to(tmp_path, target_is_directory=True)

    with pytest.raises(ValueError, match="directory"):
        storage_bench.build_fio_command(tmp_path / "missing")
    with pytest.raises(ValueError, match="directory"):
        storage_bench.build_fio_command(regular_file)
    with pytest.raises(ValueError, match="symlink"):
        storage_bench.build_fio_command(linked_root)


def test_fio_command_refuses_preexisting_benchmark_data(tmp_path: Path) -> None:
    storage_bench = _load_storage_bench()
    data_path = tmp_path / storage_bench.BENCHMARK_DATA_FILENAME
    data_path.write_text("unowned data")

    with pytest.raises(FileExistsError, match="benchmark data"):
        storage_bench.build_fio_command(tmp_path)

    assert data_path.read_text() == "unowned data"


def test_parse_fio_bandwidth_reads_write_bw_bytes() -> None:
    storage_bench = _load_storage_bench()
    payload = json.dumps(
        {"jobs": [{"write": {"bw_bytes": 600_000_000}}]}
    )

    assert storage_bench.parse_fio_bandwidth(payload) == 600_000_000.0


def test_parse_fio_bandwidth_requires_exact_bounded_integer() -> None:
    storage_bench = _load_storage_bench()

    assert storage_bench.parse_fio_bandwidth(
        '{"jobs": [{"write": {"bw_bytes": 540000000}}]}'
    ) == 540_000_000
    with pytest.raises(ValueError, match="fio"):
        storage_bench.parse_fio_bandwidth(
            '{"jobs": [{"write": {"bw_bytes": 539999999.99999999}}]}'
        )
    with pytest.raises(ValueError, match="fio"):
        storage_bench.parse_fio_bandwidth(
            '{"jobs": [{"write": {"bw_bytes": 18446744073709551616}}]}'
        )


@pytest.mark.parametrize(
    "payload",
    [
        pytest.param("not json", id="invalid-json"),
        pytest.param("{}", id="missing-jobs"),
        pytest.param('{"jobs": []}', id="empty-jobs"),
        pytest.param('{"jobs": {}}', id="jobs-not-list"),
        pytest.param('{"jobs": [{"read": {}}]}', id="missing-write"),
        pytest.param('{"jobs": [{"write": {}}]}', id="missing-bandwidth"),
        pytest.param(
            '{"jobs": [{"write": {"bw_bytes": true}}]}', id="boolean"
        ),
        pytest.param(
            '{"jobs": [{"write": {"bw_bytes": "600000000"}}]}',
            id="string",
        ),
        pytest.param(
            '{"jobs": [{"write": {"bw_bytes": -1}}]}', id="negative"
        ),
        pytest.param(
            '{"jobs": [{"write": {"bw_bytes": NaN}}]}', id="non-finite"
        ),
        pytest.param(
            '{"jobs": [{"write": {"bw_bytes": 1}}, '
            '{"write": {"bw_bytes": 2}}]}',
            id="multiple-jobs",
        ),
    ],
)
def test_parse_fio_bandwidth_rejects_invalid_payload(payload: str) -> None:
    storage_bench = _load_storage_bench()

    with pytest.raises(ValueError, match="fio"):
        storage_bench.parse_fio_bandwidth(payload)


@pytest.mark.parametrize(
    ("bandwidth_bytes", "passed"),
    [
        pytest.param(600_000_000, True, id="above-threshold"),
        pytest.param(540_000_000, True, id="at-threshold"),
        pytest.param(539_999_999, False, id="below-threshold"),
    ],
)
def test_storage_benchmark_runner_reports_decimal_threshold_and_cleans_data(
    tmp_path: Path, bandwidth_bytes: int, passed: bool
) -> None:
    storage_bench = _load_storage_bench()
    data_path = tmp_path / storage_bench.BENCHMARK_DATA_FILENAME
    report_path = tmp_path / "vt_storage_bench.json"
    neighbor = tmp_path / "keep-me"
    neighbor.write_text("preserve")
    calls: list[tuple[list[str], dict[str, object]]] = []

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess:
        calls.append((command, kwargs))
        assert data_path.is_file()
        data_path.write_bytes(b"mock fio data")
        return subprocess.CompletedProcess(
            command,
            0,
            stdout=json.dumps(
                {"jobs": [{"write": {"bw_bytes": bandwidth_bytes}}]}
            ),
            stderr="",
        )

    result = storage_bench.run_storage_benchmark(
        tmp_path,
        run_command=fake_run,
        now_utc=lambda: datetime(2026, 7, 19, 1, 2, 3, tzinfo=timezone.utc),
    )

    assert result.passed is passed
    assert result.bytes_per_second == float(bandwidth_bytes)
    assert result.megabytes_per_second == bandwidth_bytes / 1_000_000
    assert result.report_path == report_path
    assert result.error is None
    assert len(calls) == 1
    command, kwargs = calls[0]
    assert command == storage_bench.build_fio_command(tmp_path)
    assert kwargs == {
        "capture_output": True,
        "text": True,
        "check": False,
        "timeout": 900.0,
    }
    assert not data_path.exists()
    assert neighbor.read_text() == "preserve"

    report = json.loads(report_path.read_text())
    assert report == {
        "schema_version": 1,
        "timestamp_utc": "2026-07-19T01:02:03Z",
        "output_root": str(tmp_path),
        "data_path": str(data_path),
        "command": command,
        "command_facts": {
            "operation": "sequential_write",
            "block_size_bytes": 4 * 1024 * 1024,
            "size_bytes": 16 * 1024 * 1024 * 1024,
            "direct": True,
            "ioengine": "libaio",
            "iodepth": 1,
            "numjobs": 1,
        },
        "fio_returncode": 0,
        "measured_bytes_per_second": float(bandwidth_bytes),
        "measured_mb_per_second": bandwidth_bytes / 1_000_000,
        "minimum_mb_per_second": 540.0,
        "passed": passed,
        "error": None,
    }


@pytest.mark.parametrize(
    ("mode", "expected_returncode", "error_match"),
    [
        pytest.param("nonzero", 3, "status 3", id="fio-nonzero"),
        pytest.param("invalid-json", 0, "fio JSON", id="invalid-json"),
        pytest.param("exception", None, "fio unavailable", id="launch-error"),
    ],
)
def test_storage_benchmark_failures_are_reported_and_cleaned(
    tmp_path: Path,
    mode: str,
    expected_returncode: int | None,
    error_match: str,
) -> None:
    storage_bench = _load_storage_bench()
    data_path = tmp_path / storage_bench.BENCHMARK_DATA_FILENAME
    neighbor = tmp_path / "keep-me"
    neighbor.write_text("preserve")

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess:
        assert data_path.is_file()
        data_path.write_bytes(b"small mock")
        if mode == "exception":
            raise OSError("fio unavailable")
        if mode == "nonzero":
            return subprocess.CompletedProcess(
                command, 3, stdout="not json", stderr="device error"
            )
        return subprocess.CompletedProcess(
            command, 0, stdout="not json", stderr=""
        )

    result = storage_bench.run_storage_benchmark(
        tmp_path,
        run_command=fake_run,
        now_utc=lambda: datetime(2026, 7, 19, tzinfo=timezone.utc),
    )

    assert result.passed is False
    assert result.bytes_per_second is None
    assert result.megabytes_per_second is None
    assert result.error is not None and error_match in result.error
    assert not data_path.exists()
    assert neighbor.read_text() == "preserve"
    report = json.loads((tmp_path / "vt_storage_bench.json").read_text())
    assert report["timestamp_utc"] == "2026-07-19T00:00:00Z"
    assert report["fio_returncode"] == expected_returncode
    assert report["measured_bytes_per_second"] is None
    assert report["measured_mb_per_second"] is None
    assert report["passed"] is False
    assert error_match in report["error"]
    assert report["command_facts"]["size_bytes"] == 16 * 1024**3


@pytest.mark.parametrize(
    "bandwidth_token",
    [
        pytest.param("true", id="bool"),
        pytest.param("NaN", id="nan"),
        pytest.param("9" * 4000, id="huge-4000-digit-int"),
        pytest.param("539999999.99999999", id="rounded-looking-float"),
    ],
)
def test_storage_benchmark_schema_failures_write_audit_and_clean_owned_data(
    tmp_path: Path,
    bandwidth_token: str,
) -> None:
    storage_bench = _load_storage_bench()
    data_path = tmp_path / storage_bench.BENCHMARK_DATA_FILENAME

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess:
        assert data_path.is_file()
        data_path.write_bytes(b"owned temporary data")
        return subprocess.CompletedProcess(
            command,
            0,
            stdout=(
                '{"jobs": [{"write": {"bw_bytes": '
                + bandwidth_token
                + "}}]}"
            ),
            stderr="",
        )

    result = storage_bench.run_storage_benchmark(
        tmp_path,
        run_command=fake_run,
        now_utc=lambda: datetime(2026, 7, 19, tzinfo=timezone.utc),
    )

    assert result.passed is False
    assert result.bytes_per_second is None
    assert result.megabytes_per_second is None
    assert result.error is not None and "fio" in result.error
    assert not data_path.exists()
    report = json.loads((tmp_path / "vt_storage_bench.json").read_text())
    assert report["passed"] is False
    assert report["measured_bytes_per_second"] is None
    assert report["measured_mb_per_second"] is None
    assert "fio" in report["error"]


def _successful_fio(
    command: list[str], **kwargs: object
) -> subprocess.CompletedProcess:
    return subprocess.CompletedProcess(
        command,
        0,
        stdout='{"jobs": [{"write": {"bw_bytes": 600000000}}]}',
        stderr="",
    )


def test_storage_benchmark_report_uses_atomic_replace(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    storage_bench = _load_storage_bench()
    calls: list[tuple[Path, Path]] = []
    real_replace = os.replace

    def tracked_replace(source: object, target: object) -> None:
        calls.append((Path(source), Path(target)))
        real_replace(source, target)

    monkeypatch.setattr(os, "replace", tracked_replace)

    storage_bench.run_storage_benchmark(
        tmp_path, run_command=_successful_fio
    )

    assert len(calls) == 1
    temporary, report = calls[0]
    assert temporary.parent == report.parent == tmp_path
    assert report == tmp_path / "vt_storage_bench.json"
    assert temporary != report
    assert not temporary.exists()


def test_storage_benchmark_atomic_report_failure_cleans_all_owned_temps(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    storage_bench = _load_storage_bench()

    def failed_replace(source: object, target: object) -> None:
        raise OSError("report replace failed")

    monkeypatch.setattr(os, "replace", failed_replace)

    with pytest.raises(OSError, match="report replace failed"):
        storage_bench.run_storage_benchmark(
            tmp_path, run_command=_successful_fio
        )

    assert not (tmp_path / storage_bench.BENCHMARK_DATA_FILENAME).exists()
    assert not (tmp_path / "vt_storage_bench.json").exists()
    assert tuple(tmp_path.iterdir()) == ()


@pytest.mark.parametrize(
    ("passed", "expected_exit"),
    [
        pytest.param(True, 0, id="passed"),
        pytest.param(False, 1, id="failed"),
    ],
)
def test_storage_benchmark_cli_exit_reflects_result(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    passed: bool,
    expected_exit: int,
) -> None:
    storage_bench = _load_storage_bench()
    report_path = tmp_path / "vt_storage_bench.json"

    def fake_benchmark(output_root: Path) -> SimpleNamespace:
        assert output_root == tmp_path
        return SimpleNamespace(
            passed=passed,
            bytes_per_second=600_000_000.0 if passed else 500_000_000.0,
            megabytes_per_second=600.0 if passed else 500.0,
            report_path=report_path,
            error=None,
        )

    monkeypatch.setattr(
        storage_bench, "run_storage_benchmark", fake_benchmark
    )

    assert storage_bench.main(["--output-root", str(tmp_path)]) == expected_exit


def test_storage_benchmark_cli_returns_two_for_invalid_root(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    storage_bench = _load_storage_bench()

    def failed_benchmark(output_root: Path) -> object:
        raise ValueError("unsafe output root")

    monkeypatch.setattr(
        storage_bench, "run_storage_benchmark", failed_benchmark
    )

    assert storage_bench.main(["--output-root", str(tmp_path)]) == 2


def test_storage_benchmark_script_uses_system_python_entrypoint() -> None:
    source = STORAGE_BENCH_SCRIPT.read_text()

    assert source.startswith("#!/usr/bin/python3\n")
    assert 'if __name__ == "__main__":' in source
    assert "raise SystemExit(main())" in source
