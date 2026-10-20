"""Bounded, read-only discovery of local workstation paths.

This module only inspects filesystem metadata. A ready path has the expected
type and structural markers; it does not certify imports, model licensing,
hardware identity, or calibration. No directories are created here.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import os
from pathlib import Path


@dataclass(frozen=True)
class PathFinding:
    group_id: str
    key: str
    status: str
    value: str = ""
    candidates: tuple[str, ...] = ()


@dataclass(frozen=True)
class DiscoveryResult:
    configuration: dict
    findings: tuple[PathFinding, ...]
    updated_fields: tuple[tuple[str, str], ...]


# These are the source files inspected by vt_fsglove12.upstream_core. Discovery
# checks their presence only; the actual adapter still verifies pinned hashes.
_FSGLOVE_MARKERS = (
    "src/hand_visualiser/modules/hand/calibration.py",
    "src/hand_visualiser/modules/utils.py",
    "src/hand_visualiser/modules/enumerates.py",
    "src/hand_visualiser/modules/datamodels.py",
)
_PYVUT_MARKERS = (
    "pyvut/live_hid.py", "pyvut/live_bootstrap_bundle.py", "pyvut/pose_decoder.py",
)


def _path(value, home):
    """Expand user notation only, never shell variables or relative paths."""
    if not isinstance(value, (str, os.PathLike)) or not str(value):
        return None
    text = str(value)
    if any(char in text for char in ("\0", "\n", "\r")):
        return None
    try:
        if text == "~":
            path = home
        elif text.startswith("~/"):
            path = home / text[2:]
        else:
            path = Path(text).expanduser()
        return path if path.is_absolute() else None
    except (OSError, RuntimeError, ValueError):
        return None


def _matches(path, predicate):
    if path is None:
        return False
    try:
        return bool(predicate(path))
    except (OSError, RuntimeError, ValueError):
        return False


def _file(path):
    return path.is_file()


def _directory(path):
    return path.is_dir()


def _output_directory(path):
    return path.is_dir() and not path.is_symlink()


def _python(path):
    return path.is_file() and os.access(path, os.X_OK)


def _markers(names):
    return lambda path: path.is_dir() and all((path / name).is_file() for name in names)


def default_data_root(*, home=None, environ=None) -> Path:
    """Return the configured output location without creating it."""
    environment = os.environ if environ is None else environ
    local_home = Path.home() if home is None else Path(home).expanduser().absolute()
    if "VT_DATA_ROOT" not in environment:
        return local_home / "visiontactile-data"
    path = _path(environment["VT_DATA_ROOT"], local_home)
    if path is None:
        raise ValueError("VT_DATA_ROOT 必须是绝对目录路径；可使用 ~，不执行 shell 变量展开。")
    return path


class _Discovery:
    def __init__(self, configuration, root, home, environment, replace_missing):
        self.configuration = deepcopy(configuration)
        self.root, self.home = root, home
        self.environment = environment
        self.creation_default = _path(
            environment.get("VT_DATA_ROOT", str(home / "visiontactile-data")), home)
        self.replace_missing = replace_missing
        self.findings = []
        self.updated_fields = []

    def _explicit(self, names):
        for name in names:
            if name in self.environment:
                return True, _path(self.environment[name], self.home)
        return False, None

    def _unique(self, candidates, predicate):
        paths, seen = [], set()
        for candidate in candidates:
            path = _path(candidate, self.home)
            if not _matches(path, predicate):
                continue
            try:
                # Different venvs may symlink Python to the same system binary.
                # Their invocation directories still select different packages.
                identity = (path.parent.resolve() / path.name if predicate is _python
                            else path.resolve())
            except (OSError, RuntimeError, ValueError):
                continue
            if identity not in seen:
                seen.add(identity)
                paths.append(str(path))
        return tuple(paths)

    def record(self, group_id, key, status, value="", candidates=()):
        self.findings.append(PathFinding(group_id, key, status, value, tuple(candidates)))

    def set(self, target, group_id, key, value):
        if target.get(key) != value:
            target[key] = value
            self.updated_fields.append((group_id, key))

    def select(self, target, group_id, key, *, predicate=_file, candidates=(),
               environment=(), repair=False, manual=False):
        value = target.get(key, "")
        current = _path(value, self.home)
        if value and _matches(current, predicate):
            self.record(group_id, key, "ready", value)
            return
        if value and not (repair and self.replace_missing):
            self.record(group_id, key, "missing", value)
            return
        explicit, selected = self._explicit(environment)
        choices = self._unique((selected,) if selected is not None else (), predicate) if explicit else self._unique(candidates, predicate)
        if len(choices) == 1:
            self.set(target, group_id, key, choices[0])
            self.record(group_id, key, "ready", choices[0], choices)
        elif len(choices) > 1:
            self.record(group_id, key, "ambiguous", value, choices)
        else:
            self.record(group_id, key, "manual" if manual and not value and not explicit else "missing", value)

    def output(self, target, group_id, key, default):
        value = target.get(key, "")
        if not value:
            value = str(default)
            self.set(target, group_id, key, value)
        path = _path(value, self.home)
        if _matches(path, _output_directory):
            status = "ready"
        elif (path is not None and path == self.creation_default
              and _matches(path, lambda p: not p.exists() and not p.is_symlink())):
            status = "default"
        else:
            status = "missing"
        self.record(group_id, key, status, value)
        return path

    def site_candidates(self):
        explicit, venv = self._explicit(("VT_PYVUT_VENV",))
        if explicit and venv is None:
            return (), True
        if not explicit:
            venv = self.home / ".local/share/visiontactile/pyvut-venv"
        try:
            # Bounded to the one documented venv, with no recursive traversal.
            return tuple(sorted((venv / "lib").glob("python*/site-packages"))), explicit
        except (OSError, RuntimeError, ValueError):
            return (), explicit

    def glove(self, parameters, group_id, hand, data):
        self.select(parameters, group_id, "glove_python", predicate=_python, repair=True,
                    environment=("VT_FSGLOVE_PYTHON",), candidates=(
                        self.root / ".venv-fsglove-120hz/bin/python",
                        self.root / ".venv-fsglove/bin/python",
                        self.root / ".venv-glove/bin/python",
                        self.root / "tools/fsglove12/.venv/bin/python",
                        self.home / ".venvs/vt-fsglove/bin/python",
                    ))
        self.select(parameters, group_id, "fsglove_source", predicate=_markers(_FSGLOVE_MARKERS),
                    repair=True, environment=("VT_FSGLOVE_SOURCE", "FSGLOVE12_UPSTREAM_CHECKOUT", "FSGLOVE_SOURCE"),
                    candidates=(self.root / "external/FSGlove", self.home / "visiontactile-deps/FSGlove",
                                self.home / ".local/share/visiontactile/fsglove"))
        if hand in {"left", "right"}:
            self.select(parameters, group_id, "mano_assets",
                        predicate=_markers((f"models/MANO_{hand.upper()}.pkl",)), repair=True,
                        environment=("VT_MANO_ASSETS", "FSGLOVE12_MANO_ASSETS", "MANO_ASSETS"),
                        candidates=(self.root / "external/mano-assets",
                                    data / "models/mano-assets", data / "models/mano",
                                    self.home / "visiontactile-data/models/mano-assets",
                                    self.home / "visiontactile-deps/mano-assets"))

    def tracker(self, parameters, group_id, data):
        self.select(parameters, group_id, "pyvut_source", predicate=_markers(_PYVUT_MARKERS),
                    repair=True, environment=("VT_PYVUT_ROOT",), candidates=(
                        self.home / ".local/share/visiontactile/pyvut-src", self.root / "external/pyvut"))
        sites, _ = self.site_candidates()
        self.select(parameters, group_id, "pyvut_site_packages", predicate=_directory,
                    repair=True, environment=("VT_PYVUT_SITE",), candidates=sites)
        self.select(parameters, group_id, "bundle_path",
                    predicate=lambda path: path.is_file() and not path.resolve().is_relative_to(self.root.resolve()),
                    environment=("VT_BUNDLE",), manual=True)
        self.select(parameters, group_id, "role_map_path", environment=("VT_ROLE_MAP",),
                    candidates=(data / "tracker-validation/roles.yaml",), manual=True)


def discover_configuration(configuration, *, repo_root, home=None, environ=None,
                           replace_missing=False) -> DiscoveryResult:
    """Suggest missing local paths, preserving the caller's configuration.

    ``replace_missing`` permits repair of missing software dependencies only.
    Explicit hardware configuration files, port identity, and calibrated scene
    selection are never replaced. Results are proposals until the caller saves.
    """
    local_home = Path.home() if home is None else Path(home).expanduser().absolute()
    root = Path(repo_root).expanduser().absolute()
    environment = dict(os.environ if environ is None else environ)
    discovery = _Discovery(configuration, root, local_home, environment, replace_missing)
    groups = discovery.configuration.get("process_groups", ())

    # An explicit camera output location is also the root for its local inputs.
    data = None
    for group in groups:
        if group.get("preset") == "cameras":
            current = _path(group.get("parameters", {}).get("output_root", ""), local_home)
            if _matches(current, _output_directory):
                data = current
                break
    if data is None:
        data = default_data_root(home=local_home, environ=environment)
    output = None
    for group in groups:
        group_id, preset = group.get("id", ""), group.get("preset")
        if preset not in {"cameras", "trackers", "glove", "tactile"}:
            continue
        parameters = group.setdefault("parameters", {})
        if preset == "cameras":
            candidate = discovery.output(parameters, group_id, "output_root", data)
            output = output or candidate
            local_config = data / "config/cameras.yaml"
            camera = (local_config if _matches(local_config, _file)
                      else root / "ros2_ws/src/vt_realsense_capture/config/cameras.yaml")
            discovery.select(parameters, group_id, "config_path", candidates=(camera,))
        elif preset == "trackers":
            discovery.tracker(parameters, group_id, data)
        elif preset == "glove":
            discovery.glove(parameters, group_id, parameters.get("hand"), data)
        if preset in {"glove", "tactile"}:
            value = parameters.get("port", "")
            status = "manual" if not value else ("ready" if _matches(_path(value, local_home), Path.exists) else "missing")
            discovery.record(group_id, "port", status, value)
            if parameters.get("config"):
                discovery.select(parameters, group_id, "config")

    current_disk = discovery.configuration.get("disk_path", "")
    if current_disk and replace_missing and not _matches(_path(current_disk, local_home), _output_directory):
        discovery.set(discovery.configuration, "", "disk_path", str(output or data))
    discovery.output(discovery.configuration, "", "disk_path", output or data)
    discovery.select(discovery.configuration, "", "scene_config_path",
                     candidates=(root / "ros2_ws/src/vt_capture_manager/config/scene.example.yaml",))
    return DiscoveryResult(discovery.configuration, tuple(discovery.findings),
                           tuple(discovery.updated_fields))
