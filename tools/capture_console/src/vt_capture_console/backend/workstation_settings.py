"""Private desktop settings and enumeration-only serial choices. No ROS/Qt."""
from __future__ import annotations

from copy import deepcopy
import json
import os
from pathlib import Path
import stat
import uuid

from vt_capture_core.workstation import validate_configuration


def repository_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "tools/capture_deploy/start_manager.sh").is_file():
            return parent
    raise ValueError("无法找到工作站仓库；请从仓库安装的桌面入口打开")


def private_directory(path):
    path = Path(path).expanduser().absolute()
    missing, parent = [], path
    while not parent.exists() and not parent.is_symlink():
        missing.append(parent)
        parent = parent.parent
    for item in reversed(missing):
        try:
            item.mkdir(mode=0o700)
        except FileExistsError:
            pass
    info = path.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise ValueError(f"设置目录必须为当前用户拥有的私有目录（0700），不能是链接：{path}")
    return path


def atomic_json(path, value):
    path = Path(path)
    temporary = path.parent / ("." + path.name + "." + uuid.uuid4().hex + ".tmp")
    fd = os.open(temporary, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        temporary.unlink(missing_ok=True)


def read_private_json(path):
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
    with os.fdopen(fd, "r", encoding="utf-8") as stream:
        info = os.fstat(stream.fileno())
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                or info.st_mode & 0o077 or info.st_size > 1024 * 1024):
            raise ValueError(f"拒绝读取非私有或过大的设置文件：{path}")
        return json.load(stream)


def default_configuration(repo_root=None):
    root = Path(repo_root).expanduser().absolute() if repo_root is not None else repository_root()

    def existing(path):
        return str(path) if path.exists() else ""

    output = Path.home() / "visiontactile-data"
    camera = root / "ros2_ws/src/vt_realsense_capture/config/cameras.yaml"
    # Only concrete, familiar paths are suggested, without reading their
    # contents. Ports and tracker startup bundles are never auto-assigned.
    baseline = root / "artifacts/glove_hand_pose_live/upstream_baselines_20260930"
    source = existing(baseline / "fsglove-a7988f0")
    assets = existing(baseline / "mano-assets")
    right_config = existing(root / ".runtime/recovery/right_current_live_20261005.json")
    role_map = next((str(path) for path in (output / "tracker-validation/roles.yaml",
                    root / "visiontactile-data/tracker-validation/roles.yaml") if path.is_file()), "")
    tracker_local = root / ".local/share/visiontactile"
    pyvut_source = existing(tracker_local / "pyvut-src/.worktrees/capture-locked-live-bootstrap")
    pyvut_site = existing(tracker_local / "pyvut-venv/lib/python3.12/site-packages")
    python = next((str(p) for p in (root / ".venv-fsglove-120hz/bin/python", root / ".venv-fsglove-restored/bin/python",
                                   root / ".venv-glove/bin/python", root / "tools/fsglove12/.venv/bin/python") if p.is_file()), "")
    profiles = [dict(id="cameras", label="三相机与录制控制器", preset="cameras", enabled=False,
                     startup_timeout_s=45., parameters=dict(config_path=existing(camera), output_root=existing(output))),
                dict(id="trackers", label="三 Tracker ROS 读取", preset="trackers", enabled=False,
                     startup_timeout_s=30., parameters=dict(bundle_path="", role_map_path=role_map,
                     pyvut_source=pyvut_source, pyvut_site_packages=pyvut_site))]
    for hand, label in (("right", "右手"), ("left", "左手")):
        profiles.append(dict(id=hand + "_glove", label=label + "姿态与标定窗口", preset="glove", enabled=False,
                             startup_timeout_s=120., parameters=dict(hand=hand, launch_gui="true", glove_python=python,
                             fsglove_source=source, mano_assets=assets, port="",
                             config=right_config if hand == "right" else "")))
    for hand, label in (("right", "右手"), ("left", "左手")):
        profiles.append(dict(id=hand + "_tactile", label=label + "触觉采集", preset="tactile", enabled=False,
                             startup_timeout_s=30., parameters=dict(hand=hand, port="")))
    scene = existing(root / "ros2_ws/src/vt_capture_manager/config/scene.example.yaml")
    return validate_configuration(dict(disk_path=existing(output) or str(root), quality_enabled=True,
                                       process_groups=profiles, scene_config_path=scene))


def serial_ports(dev_root=Path("/dev")):
    """List paths only; no serial library, termios, read, or open is used."""
    root = Path(dev_root)
    choices, targets = [], set()
    for category in ("by-id", "by-path"):
        directory = root / "serial" / category
        try:
            paths = sorted(directory.iterdir())
        except OSError:
            paths = []
        for path in paths:
            try:
                target = path.resolve(strict=True)
                if target in targets:
                    continue
                targets.add(target)
                choices.append(dict(path=str(path), label=f"{path.name} → {target.name}"))
            except (OSError, RuntimeError):
                continue
    for pattern in ("ttyACM*", "ttyUSB*"):
        for path in sorted(root.glob(pattern)):
            try:
                target = path.resolve(strict=True)
            except (OSError, RuntimeError):
                continue
            if target not in targets:
                targets.add(target)
                choices.append(dict(path=str(path), label=path.name + "（未指定设备身份）"))
    return choices


class WorkstationSettings:
    def __init__(self, repo_root=None, settings_dir=None):
        self.repo_root = Path(repo_root).expanduser().absolute() if repo_root is not None else repository_root()
        self.directory = private_directory(settings_dir or Path.home() / ".config/visiontactile/workbench")
        self.path = self.directory / "workstation.json"
        self._configuration = validate_configuration(read_private_json(self.path)) if self.path.exists() or self.path.is_symlink() else default_configuration(self.repo_root)

    @property
    def configuration(self):
        return deepcopy(self._configuration)

    def save(self, configuration):
        value = validate_configuration(configuration)
        atomic_json(self.path, value)
        self._configuration = value
        return self.configuration
