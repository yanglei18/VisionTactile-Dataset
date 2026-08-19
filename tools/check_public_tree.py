#!/usr/bin/env python3
from __future__ import annotations

import os
import posixpath
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAX_TRACKED_BYTES = 5 * 1024 * 1024
REQUIRED_FILES = {
    ".github/ISSUE_TEMPLATE/bug_report.yml",
    ".github/ISSUE_TEMPLATE/config.yml",
    ".github/PULL_REQUEST_TEMPLATE.md",
    ".github/workflows/ci.yml",
    ".gitignore",
    ".gitmodules",
    "CHANGELOG.md",
    "CONTRIBUTING.md",
    "LICENSE",
    "README.md",
    "README.zh-CN.md",
    "SECURITY.md",
    "THIRD_PARTY_NOTICES.md",
    "docs/architecture.md",
    "docs/capture-guide.md",
    "docs/hardware-reference.md",
    "docs/interface-reference.md",
    "docs/release-checklist.md",
    "docs/tracker-linux-validation.md",
    "docs/tracker-camera-calibration.md",
    "docs/tracker-ros2-publisher.md",
    "docs/tracker-windows-map.md",
    "docs/troubleshooting.md",
    "docs/user-manual.md",
    "ros2_ws/src/vt_camera_msgs/package.xml",
    "ros2_ws/src/vt_realsense_capture/package.xml",
    "ros2_ws/src/vt_tracker_msgs/package.xml",
    "ros2_ws/src/vt_vive_tracker/package.xml",
    "ros2_ws/src/vt_vive_tracker_gui/README.md",
    "ros2_ws/src/vt_vive_tracker_gui/package.xml",
    "tools/check_public_tree.py",
    "tools/tracker_camera_calibration/README.md",
    "tools/tracker_camera_calibration/config/calibration.example.yaml",
    "tools/tracker_camera_calibration/pyproject.toml",
    "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/__init__.py",
    "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/bag_reader.py",
    "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/charuco.py",
    "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/cli.py",
    "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/config.py",
    "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/config_writer.py",
    "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/export.py",
    "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/handeye.py",
    "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/model.py",
    "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/pairing.py",
    "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/repeatability.py",
    "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/transforms.py",
    "tools/tracker_camera_calibration/tests/test_charuco.py",
    "tools/tracker_camera_calibration/tests/test_config_and_bag_helpers.py",
    "tools/tracker_camera_calibration/tests/test_config_writer.py",
    "tools/tracker_camera_calibration/tests/test_export.py",
    "tools/tracker_camera_calibration/tests/test_handeye.py",
    "tools/tracker_camera_calibration/tests/test_pairing.py",
    "tools/tracker_camera_calibration/tests/test_repeatability.py",
    "tools/tracker_camera_calibration/tests/test_transforms.py",
    "tools/vut_validation/config/70-vive-ultimate-tracker.rules",
    "tools/vut_validation/config/roles.example.json",
    "tools/vut_validation/pyproject.toml",
    "third_party/pyvut",
}
EXPECTED_SUBMODULES = {
    "third_party/pyvut": "7da6b081ad4ebfa0a0f2f242cdecb9ffc47341f1",
}
FORBIDDEN_DIRECTORY_COMPONENTS = {
    ".cache",
    ".idea",
    ".pytest_cache",
    ".superpowers",
    ".vscode",
    ".venv-calibration",
    ".worktrees",
    "__pycache__",
    "artifacts",
    "bags",
    "build",
    "install",
    "log",
}
FORBIDDEN_EXPLICIT_PREFIXES = ("docs/superpowers/",)
FORBIDDEN_BASENAMES = {".DS_Store"}
FORBIDDEN_BASENAME_PREFIXES = ("lark-", "rs-save-to-disk-output-")
FORBIDDEN_SUFFIXES = (
    ".bag",
    ".db3",
    ".mcap",
    ".pcap",
    ".pcapng",
    ".pyc",
    ".pyd",
    ".pyo",
    ".webm",
    ".zip",
)
REGULAR_FILE_MODES = {"100644", "100755"}
EXPECTED_MESSAGES = {
    "CameraDescriptor.msg",
    "CameraFrameTiming.msg",
    "CameraGroupStatus.msg",
    "CaptureCommand.msg",
    "CaptureEvent.msg",
    "CaptureStatus.msg",
    "SessionInfo.msg",
    "StreamProfile.msg",
    "StreamStatus.msg",
}
EXPECTED_TRACKER_MESSAGES = {
    "TrackerSample.msg",
    "TrackerStatus.msg",
}
DOCUMENT_TOKENS = {
    "README.md": (
        "README.zh-CN.md",
        "Ubuntu 24.04",
        "ROS 2 Jazzy",
        "RealSense ROS 4.58.1",
        "Apache-2.0",
        "tools/tracker_camera_calibration/README.md",
    ),
    "README.zh-CN.md": (
        "README.md",
        "Ubuntu 24.04",
        "ROS 2 Jazzy",
        "RealSense ROS 4.58.1",
        "Apache-2.0",
        "tools/tracker_camera_calibration/README.md",
    ),
    "THIRD_PARTY_NOTICES.md": (
        "PyVUT",
        "third_party/pyvut",
        "7da6b081ad4ebfa0a0f2f242cdecb9ffc47341f1",
        "Apache-2.0",
    ),
    ".gitmodules": (
        "third_party/pyvut",
        "https://github.com/yanglei18/pyvut.git",
    ),
    "docs/architecture.md": (
        "9 topics",
        "CameraFrameTiming",
        "keep-last depth 30",
        "Recorder process lifecycle",
    ),
    "docs/hardware-reference.md": (
        "260322278433",
        "260322276463",
        "408322071716",
    ),
    "docs/interface-reference.md": (
        "Camera launch arguments",
        "Tracker launch arguments",
        "Camera time semantics",
        "COMPLETE",
    ),
    "docs/release-checklist.md": (
        "Sanitize the public tree",
        "Run software verification",
        "Run reference hardware acceptance",
    ),
    "docs/capture-guide.md": (
        "triple_realsense.launch.py",
        "/capture/command",
        "ros2 bag info",
    ),
    "docs/troubleshooting.md": (
        "Incomplete timing group",
        "ros2 topic list",
        "RViz",
    ),
    "docs/tracker-windows-map.md": (
        "Windows mapping is mandatory",
        "C:\\vut-validation\\",
        "firmware update",
    ),
    "docs/tracker-linux-validation.md": (
        "7da6b081ad4ebfa0a0f2f242cdecb9ffc47341f1",
        "Pinned PyVUT submodule",
        "not proven device time",
    ),
    "docs/tracker-camera-calibration.md": (
        "tools/tracker_camera_calibration/",
        "^tracker T_camera",
        "group_host_realtime_ns",
        "纯离线",
        "compare",
    ),
    "docs/tracker-ros2-publisher.md": (
        "Windows mapping is mandatory",
        "vt-vive-write-role-map",
        "triple_tracker.launch.py",
        "vt-vive-validate-topics",
        "status=PASS roles=3 identity_swaps=0 dropped=0",
        "No TF is published",
    ),
    "docs/user-manual.md": (
        "最终用户手册",
        "交付验收矩阵",
        "数据与开源边界",
    ),
    "tools/tracker_camera_calibration/README.md": (
        "vt-tracker-camera-calibrate configure",
        "vt-tracker-camera-calibrate calibrate",
        "vt-tracker-camera-calibrate compare",
        "不需要也不读取抓包",
        "录制前输入门禁",
        "故障排查",
        "最终检查表",
    ),
}
DOCUMENT_FORBIDDEN_TOKENS = {
    "tools/tracker_camera_calibration/README.md": (
        "live_windows_bootstrap.py",
        "VT_CAPTURE_SHA256",
        "--execute-feature-writes",
        "private/vut/01_cold_reconnect.pcapng",
        "private/vut/live-bootstrap.json",
    ),
}
OBSOLETE_DOCUMENT_TOKENS = (
    "29-topic",
    "29 个 topic",
    "bag_validate",
    "message-Zstd",
    "EOF validator",
)
PUBLIC_DOCUMENT_PATHS = (
    "README.md",
    "README.zh-CN.md",
    "docs/architecture.md",
    "docs/capture-guide.md",
    "docs/hardware-reference.md",
    "docs/interface-reference.md",
    "docs/release-checklist.md",
    "docs/troubleshooting.md",
    "docs/tracker-linux-validation.md",
    "docs/tracker-camera-calibration.md",
    "docs/tracker-ros2-publisher.md",
    "docs/tracker-windows-map.md",
    "docs/user-manual.md",
    "ros2_ws/src/vt_vive_tracker_gui/README.md",
    "CHANGELOG.md",
    "CONTRIBUTING.md",
    "THIRD_PARTY_NOTICES.md",
    "tools/tracker_camera_calibration/README.md",
)
_MARKDOWN_LINK = re.compile(r"\[[^\]]*\]\(([^)]+)\)")


def git(*args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(ROOT), *args],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    ).stdout


def _forbidden_path_messages(path: str) -> tuple[str, ...]:
    path_components = path.split("/")
    directory_components = path_components[:-1]
    messages: list[str] = []
    if (
        any(
            component in FORBIDDEN_DIRECTORY_COMPONENTS
            for component in directory_components
        )
        or path.startswith(FORBIDDEN_EXPLICIT_PREFIXES)
    ):
        messages.append(f"forbidden tracked path: {path}")
    if any(
        component in FORBIDDEN_BASENAMES
        or component.startswith(FORBIDDEN_BASENAME_PREFIXES)
        or component.endswith(FORBIDDEN_SUFFIXES)
        for component in path_components
    ):
        messages.append(f"forbidden tracked artifact: {path}")
    return tuple(messages)


def validation_errors() -> list[str]:
    tracked: dict[str, tuple[str, str]] = {}
    errors: list[str] = []
    for entry in git("ls-files", "--stage", "-z").split("\0"):
        if not entry:
            continue
        metadata, separator, path = entry.partition("\t")
        fields = metadata.split()
        if not separator or len(fields) != 3:
            errors.append("malformed Git index entry")
            continue
        mode, object_id, stage = fields
        if stage != "0":
            errors.append(f"unmerged index entry is not allowed: {path}")
            continue
        tracked[path] = (mode, object_id)

    errors.extend(
        f"missing tracked file: {path}"
        for path in sorted(REQUIRED_FILES - set(tracked))
    )
    for path, (mode, object_id) in sorted(tracked.items()):
        errors.extend(_forbidden_path_messages(path))
        if mode == "160000":
            expected_object_id = EXPECTED_SUBMODULES.get(path)
            if expected_object_id is None and path.startswith("third_party/"):
                errors.append(f"unexpected tracked submodule: {path}")
            elif expected_object_id is None:
                errors.append(f"non-regular tracked entry is not allowed: {path}")
            elif object_id != expected_object_id:
                errors.append(
                    f"submodule revision mismatch: {path}: "
                    f"expected={expected_object_id} observed={object_id}"
                )
        elif path in EXPECTED_SUBMODULES:
            errors.append(f"expected Git submodule entry: {path}")
        elif mode == "120000":
            errors.append(f"tracked symlink is not allowed: {path}")
        elif mode not in REGULAR_FILE_MODES:
            errors.append(f"non-regular tracked entry is not allowed: {path}")
        elif int(git("cat-file", "-s", object_id)) > MAX_TRACKED_BYTES:
            errors.append(f"tracked file exceeds 5 MiB: {path}")

    message_root = "ros2_ws/src/vt_camera_msgs/msg/"
    observed_messages = {
        path.removeprefix(message_root)
        for path in tracked
        if path.startswith(message_root)
    }
    if observed_messages != EXPECTED_MESSAGES:
        errors.append(
            "public message set mismatch: "
            f"expected={sorted(EXPECTED_MESSAGES)} "
            f"observed={sorted(observed_messages)}"
        )

    tracker_message_root = "ros2_ws/src/vt_tracker_msgs/msg/"
    observed_tracker_messages = {
        path.removeprefix(tracker_message_root)
        for path in tracked
        if path.startswith(tracker_message_root)
    }
    if observed_tracker_messages != EXPECTED_TRACKER_MESSAGES:
        errors.append(
            "public tracker message set mismatch: "
            f"expected={sorted(EXPECTED_TRACKER_MESSAGES)} "
            f"observed={sorted(observed_tracker_messages)}"
        )

    for path, tokens in DOCUMENT_TOKENS.items():
        entry = tracked.get(path)
        if entry is None:
            continue
        mode, object_id = entry
        if mode not in REGULAR_FILE_MODES:
            continue
        text = git("cat-file", "blob", object_id)
        for token in tokens:
            if token not in text:
                errors.append(f"{path} is missing required text: {token}")

    for path, tokens in DOCUMENT_FORBIDDEN_TOKENS.items():
        entry = tracked.get(path)
        if entry is None or entry[0] not in REGULAR_FILE_MODES:
            continue
        text = git("cat-file", "blob", entry[1])
        for token in tokens:
            if token in text:
                errors.append(f"{path} contains forbidden text: {token}")

    for path in PUBLIC_DOCUMENT_PATHS:
        entry = tracked.get(path)
        if entry is None or entry[0] not in REGULAR_FILE_MODES:
            continue
        text = git("cat-file", "blob", entry[1])
        for token in OBSOLETE_DOCUMENT_TOKENS:
            if token in text:
                errors.append(
                    f"obsolete public documentation text: {path}: {token}"
                )

        for raw_target in _MARKDOWN_LINK.findall(text):
            target = raw_target.strip().strip("<>").split("#", 1)[0]
            if (
                not target
                or "://" in target
                or target.startswith("mailto:")
            ):
                continue
            resolved = posixpath.normpath(
                posixpath.join(posixpath.dirname(path), target)
            )
            if resolved not in tracked:
                errors.append(
                    f"broken relative Markdown link: {path}: {raw_target}"
                )

    # Alternate indexes are used by the checker unit tests to verify staged
    # content semantics. Normal release and CI runs also enforce that removed
    # private artifacts cannot be recovered from any commit reachable by HEAD.
    if "GIT_INDEX_FILE" not in os.environ:
        forbidden_history_paths: set[str] = set()
        for entry in git("rev-list", "--objects", "HEAD").splitlines():
            _, separator, path = entry.partition(" ")
            if separator and _forbidden_path_messages(path):
                forbidden_history_paths.add(path)
        errors.extend(
            f"forbidden path remains in reachable history: {path}"
            for path in sorted(forbidden_history_paths)
        )

    return errors


def main() -> int:
    errors = validation_errors()
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1
    print("public tree contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
