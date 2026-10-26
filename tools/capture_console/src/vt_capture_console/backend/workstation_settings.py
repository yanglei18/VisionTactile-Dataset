"""Private desktop settings and enumeration-only serial choices. No ROS/Qt."""
from __future__ import annotations

from copy import deepcopy
import json
import os
from pathlib import Path
import stat
import uuid

from vt_capture_core.workstation import validate_configuration

from .environment_discovery import default_data_root, discover_configuration
from .serial_ports import serial_ports, stable_serial_path


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


def default_configuration(repo_root=None, *, discover=True):
    """Build disabled profiles, optionally suggesting paths without writing."""
    root = Path(repo_root).expanduser().absolute() if repo_root is not None else repository_root()

    profiles = [dict(id="cameras", label="三相机与录制控制器", preset="cameras", enabled=False,
                     startup_timeout_s=45., parameters=dict(config_path="", output_root="")),
                dict(id="trackers", label="三 Tracker ROS 读取", preset="trackers", enabled=False,
                     startup_timeout_s=30., parameters=dict(bundle_path="", role_map_path="",
                     pyvut_source="", pyvut_site_packages=""))]
    for hand, label in (("right", "右手"), ("left", "左手")):
        profiles.append(dict(id=hand + "_glove", label=label + "姿态与标定窗口", preset="glove", enabled=False,
                             startup_timeout_s=120., parameters=dict(hand=hand, launch_gui="true", glove_python="",
                             fsglove_source="", mano_assets="", port="", config="")))
    for hand, label in (("right", "右手"), ("left", "左手")):
        profiles.append(dict(id=hand + "_tactile", label=label + "触觉采集", preset="tactile", enabled=False,
                             startup_timeout_s=30., parameters=dict(hand=hand, port="")))
    draft = dict(disk_path="", quality_enabled=True, process_groups=profiles, scene_config_path="")
    if discover:
        draft = discover_configuration(draft, repo_root=root).configuration
    return validate_configuration(draft)


def prepare_default_output(configuration):
    """Create only the documented default, when committing settings, never on detection."""
    try:
        output = default_data_root()
    except ValueError:
        # An invalid environment default must not prevent saving an existing,
        # explicit configuration. Discovery reports it when a default is needed.
        return
    uses_default = any(
        group["preset"] == "cameras"
        and group["parameters"].get("output_root")
        and Path(group["parameters"]["output_root"]).expanduser().absolute() == output
        for group in configuration.get("process_groups", ())
    )
    if not uses_default:
        return
    if output.is_symlink():
        raise ValueError(f"默认数据目录不能是符号链接：{output}")
    try:
        # Create parents privately too; Path.mkdir(parents=True) otherwise uses
        # the process umask rather than our requested mode for intermediate paths.
        missing, parent = [], output
        while not parent.exists() and not parent.is_symlink():
            missing.append(parent)
            parent = parent.parent
        for directory in reversed(missing):
            try:
                directory.mkdir(mode=0o700)
            except FileExistsError:
                pass
        if output.is_symlink() or not output.is_dir():
            raise ValueError(f"默认数据目录必须为真实目录：{output}")
        if not os.access(output, os.W_OK | os.X_OK):
            raise ValueError(f"默认数据目录不可写：{output}")
    except OSError as exc:
        raise ValueError(f"无法创建默认数据目录：{output}；{exc}") from exc


class WorkstationSettings:
    def __init__(self, repo_root=None, settings_dir=None):
        self.repo_root = Path(repo_root).expanduser().absolute() if repo_root is not None else repository_root()
        self.directory = private_directory(settings_dir or Path.home() / ".config/visiontactile/workbench")
        self.path = self.directory / "workstation.json"
        # Loading settings must not depend on optional software or prepare a
        # new launch while an existing backend may still be running.
        self._configuration = (validate_configuration(read_private_json(self.path))
                               if self.path.exists() or self.path.is_symlink()
                               else default_configuration(self.repo_root, discover=False))

    @property
    def configuration(self):
        return deepcopy(self._configuration)

    def detect_environment(self, configuration):
        return discover_configuration(configuration, repo_root=self.repo_root, replace_missing=True)

    def prepare_for_startup(self):
        """Persist discovered paths only after the caller confirms no backend is alive."""
        result = self.detect_environment(self.configuration)
        # A historical tty number may now name another device. Only an explicit
        # settings save may establish a new stable identity from that choice.
        self.save(result.configuration, bind_serial_identity=False)
        return result

    def save(self, configuration, *, bind_serial_identity=True):
        value = validate_configuration(configuration)
        if bind_serial_identity:
            for group in value["process_groups"]:
                parameters = group["parameters"]
                if group["preset"] in {"glove", "tactile"} and parameters.get("port"):
                    parameters["port"] = stable_serial_path(parameters["port"])
        prepare_default_output(value)
        atomic_json(self.path, value)
        self._configuration = value
        return self.configuration
