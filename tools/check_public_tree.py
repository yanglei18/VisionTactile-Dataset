#!/usr/bin/env python3
from __future__ import annotations

import json
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
    "tools/multisensor_alignment/README.md",
    "tools/multisensor_alignment/config/alignment.example.yaml",
    "tools/multisensor_alignment/pyproject.toml",
    "tools/multisensor_alignment/src/vt_multisensor_alignment/__init__.py",
    "tools/multisensor_alignment/src/vt_multisensor_alignment/aligner.py",
    "tools/multisensor_alignment/src/vt_multisensor_alignment/bag_reader.py",
    "tools/multisensor_alignment/src/vt_multisensor_alignment/cli.py",
    "tools/multisensor_alignment/src/vt_multisensor_alignment/clock_audit.py",
    "tools/multisensor_alignment/src/vt_multisensor_alignment/config.py",
    "tools/multisensor_alignment/src/vt_multisensor_alignment/export.py",
    "tools/multisensor_alignment/src/vt_multisensor_alignment/extrinsics.py",
    "tools/multisensor_alignment/src/vt_multisensor_alignment/matcher.py",
    "tools/multisensor_alignment/src/vt_multisensor_alignment/model.py",
    "tools/multisensor_alignment/tests/test_aligner.py",
    "tools/multisensor_alignment/tests/test_bag_reader.py",
    "tools/multisensor_alignment/tests/test_cli.py",
    "tools/multisensor_alignment/tests/test_clock_audit.py",
    "tools/multisensor_alignment/tests/test_config.py",
    "tools/multisensor_alignment/tests/test_export.py",
    "tools/multisensor_alignment/tests/test_extrinsics.py",
    "tools/multisensor_alignment/tests/test_matcher.py",
    "tools/multisensor_alignment/tests/test_model.py",
    "tools/multisensor_alignment/tests/test_synthetic_mcap.py",
    "tools/vut_validation/config/70-vive-ultimate-tracker.rules",
    "tools/vut_validation/config/roles.example.json",
    "tools/vut_validation/pyproject.toml",
    "pyvut",
}
EXPECTED_SUBMODULES = {"pyvut"}
FORBIDDEN_DIRECTORY_COMPONENTS = {
    ".local",
    ".runtime",
    ".codex-local",
    "visiontactile-data",
    "Window_reference",
    "private",
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
    "dist",
    "install",
    "log",
}
FORBIDDEN_EXPLICIT_PREFIXES = ("docs/superpowers/",)
FORBIDDEN_BASENAMES = {".DS_Store", ".env", "auth.json", "MANO_LEFT.pkl", "MANO_RIGHT.pkl",
                       "roles.yaml", "roles.json", "workstation.json", "tracker-bootstrap.json",
                       "live-bootstrap.json"}
FORBIDDEN_BASENAME_PREFIXES = ("lark-", "rs-save-to-disk-output-", "live-bootstrap-")
FORBIDDEN_SUFFIXES = (
    ".bag",
    ".db3",
    ".mcap",
    ".pcap",
    ".pcapng",
    ".pyc",
    ".pyd",
    ".pyo",
    ".exe",
    ".dll",
    ".msi",
    ".whl",
    ".so",
    ".dylib",
    ".webm",
    ".zip",
    ".tar", ".tgz", ".tar.gz", ".pth", ".pt", ".onnx", ".pkl",
)
REGULAR_FILE_MODES = {"100644", "100755"}
EXPECTED_MESSAGES = {
    "CaptureContext.msg",
    "CameraDescriptor.msg",
    "CameraFrameTiming.msg",
    "CameraTrackerCalibration.msg",
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
        "tools/multisensor_alignment/README.md",
    ),
    "README.zh-CN.md": (
        "README.md",
        "Ubuntu 24.04",
        "ROS 2 Jazzy",
        "RealSense ROS 4.58.1",
        "Apache-2.0",
        "tools/tracker_camera_calibration/README.md",
        "tools/multisensor_alignment/README.md",
    ),
    "THIRD_PARTY_NOTICES.md": (
        "PyVUT",
        "not relicensed",
        "respective upstream licenses",
    ),
    "docs/architecture.md": (
        "21 topics",
        "unified-dataset-v2",
        "extrinsics/depth_to_color",
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
        "fcfcd33f4c1f16b0d84f5f741dc1319abdc7942a",
        "Do not vendor PyVUT",
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
    "tools/multisensor_alignment/README.md": (
        "一个统一 MCAP bag",
        "21 个候选 Topic",
        "unified-dataset-v2",
        "vt-multisensor-align inspect",
        "vt-multisensor-align align",
        "vt-multisensor-align validate",
        "CLOCK_MONOTONIC_RAW",
        "动捕手套",
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
    "tools/multisensor_alignment/README.md": (
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
    "tools/multisensor_alignment/README.md",
)
_MARKDOWN_LINK = re.compile(r"\[[^\]]*\]\(([^)]+)\)")

# The workbench and dual-hand additions are part of the public source contract.
REQUIRED_FILES.update({
    ".gitattributes", ".editorconfig", ".github/workflows/capture-console.yml",
    ".github/workflows/distributions.yml",
    "Glove.py", "Ploy_IMU.py", "tools/release/README.md", "tools/release/prepare_release.py",
    "tools/release/verify_distributions.py", "tools/release/constraints-tested.txt",
    "tools/capture_console/README.md", "tools/capture_console/pyproject.toml",
    "tools/capture_console/requirements-tested.txt", "tools/capture_deploy/install.sh",
    "tools/capture_console/LICENSE", "tools/capture_console/THIRD_PARTY_NOTICES.md",
    "tools/fsglove12/LICENSE",
    "tools/capture_deploy/start_workbench.sh", "tools/fsglove12/README.md",
    "tools/fsglove12/THIRD_PARTY_NOTICES.md", "tools/glove_hand_pose/README.md",
    "tools/tracker_camera_calibration/run_gui.sh",
    *(f"ros2_ws/src/{name}/package.xml" for name in (
        "vt_capture_core", "vt_capture_manager", "vt_glove_msgs", "vt_glove_bridge",
        "vt_tactile_msgs", "vt_tactile_capture")),
})
# Each independently distributed package must carry the project's license,
# even when a consumer installs it without the repository-level LICENSE.
REQUIRED_FILES.update(
    f"tools/{name}/LICENSE" for name in (
        "capture_console", "fsglove12", "glove_hand_pose",
        "tracker_camera_calibration", "multisensor_alignment", "vut_validation",
    )
)
REQUIRED_FILES.update(
    f"ros2_ws/src/{name}/LICENSE" for name in (
        "vt_camera_msgs", "vt_realsense_capture", "vt_tracker_msgs",
        "vt_vive_tracker", "vt_vive_tracker_gui", "vt_capture_core",
        "vt_capture_manager", "vt_glove_msgs", "vt_glove_bridge",
        "vt_tactile_msgs", "vt_tactile_capture",
    )
)
EXPECTED_SERVICES = {"ManagedCaptureCommand.srv", "GetCaptureSnapshot.srv"}
PUBLIC_DOCUMENT_PATHS += (
    "tools/release/README.md", "tools/capture_console/README.md",
    "tools/capture_console/THIRD_PARTY_NOTICES.md", "tools/fsglove12/README.md",
    "tools/fsglove12/THIRD_PARTY_NOTICES.md", "tools/glove_hand_pose/README.md",
    "ros2_ws/src/vt_glove_bridge/README.md", "ros2_ws/src/vt_tactile_capture/README.md",
)


def _forbidden_directory(path: str) -> bool:
    parts = path.split("/")
    return (path.startswith(FORBIDDEN_EXPLICIT_PREFIXES)
            or any(p in FORBIDDEN_DIRECTORY_COMPONENTS or p.startswith(".venv") for p in parts[:-1]))


def _forbidden_artifact(path: str) -> bool:
    return any(
        p in FORBIDDEN_BASENAMES or p.startswith(FORBIDDEN_BASENAME_PREFIXES)
        or p.lower().endswith(FORBIDDEN_SUFFIXES)
        or re.search(r"\.so(?:\.\d+)+$", p.lower()) is not None
        or (p.startswith(".env.") and p != ".env.example")
        for p in path.split("/")
    )


def forbidden_path(path: str) -> bool:
    """Shared artifact boundary for the Git-index check and clean export."""
    return _forbidden_directory(path) or _forbidden_artifact(path)


def is_static_hand_model(path: str, content: bytes | str) -> bool:
    """Recognize exported model geometry without excluding ordinary JSON.

    The preview exporter materializes licensed model geometry. Such assets
    belong in a separately authorized deployment, never the source snapshot.
    Synthetic tests should construct their geometry in code or at runtime.
    """
    if not path.lower().endswith(".json"):
        return False
    try:
        value = json.loads(content)
    except (ValueError, UnicodeError):
        return False
    if not isinstance(value, dict):
        return False
    geometry = value.get("neutral_geometry")
    return (value.get("schema_version") == 1
            and isinstance(geometry, dict)
            and {"vertices_m", "faces", "joints_m"} <= geometry.keys()
            and isinstance(value.get("model_axes"), dict))


def git(*args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(ROOT), *args],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    ).stdout


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
        if _forbidden_directory(path):
            errors.append(f"forbidden tracked path: {path}")
        if _forbidden_artifact(path):
            errors.append(f"forbidden tracked artifact: {path}")
        if mode == "160000":
            if path not in EXPECTED_SUBMODULES:
                errors.append(f"non-regular tracked entry is not allowed: {path}")
        elif mode == "120000":
            errors.append(f"tracked symlink is not allowed: {path}")
        elif mode not in REGULAR_FILE_MODES:
            errors.append(f"non-regular tracked entry is not allowed: {path}")
        elif int(git("cat-file", "-s", object_id)) > MAX_TRACKED_BYTES:
            errors.append(f"tracked file exceeds 5 MiB: {path}")
        elif path.lower().endswith(".json") and is_static_hand_model(path, git("cat-file", "blob", object_id)):
            errors.append(f"static hand model must remain outside public sources: {path}")

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
    service_root = "ros2_ws/src/vt_camera_msgs/srv/"
    observed_services = {path.removeprefix(service_root) for path in tracked if path.startswith(service_root)}
    if observed_services != EXPECTED_SERVICES:
        errors.append("public capture service set mismatch: "
                      f"expected={sorted(EXPECTED_SERVICES)} observed={sorted(observed_services)}")
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
            if resolved not in tracked and not any(p.startswith(resolved.rstrip("/") + "/") for p in tracked):
                errors.append(
                    f"broken relative Markdown link: {path}: {raw_target}"
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
