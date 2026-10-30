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
import re

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
    "pyvut/live_hid.py", "pyvut/live_probe.py",
    "pyvut/live_bootstrap_bundle.py", "pyvut/pose_decoder.py",
)


def managed_tracker_installation(bundle="", *, home=None, current=True):
    """Read optional Linux deployment metadata without coupling portable tools.

    An absent deployment package is normal for the standalone tactile viewer.
    Errors inside an installed package and unsafe metadata remain explicit.
    """
    try:
        from vt_tracker_deploy.manifest import for_bundle, load_installation
    except ModuleNotFoundError as exc:
        if exc.name in {"vt_tracker_deploy", "vt_tracker_deploy.manifest"}:
            return None
        raise
    installation = for_bundle(bundle, home=home) if bundle else None
    if installation is None and current:
        installation = load_installation(home=home)
    return installation


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

    def tracker_sources(self):
        """Shared installation locations for the publisher and bootstrap UI."""
        return (self.home / ".local/share/visiontactile/pyvut-src",
                self.root / "external/pyvut")

    def tracker_installation(self, bundle=""):
        """A configured managed bundle pins its version even after activation changes."""
        installation = managed_tracker_installation(bundle, home=self.home)
        if installation is not None and installation.bundle.resolve().is_relative_to(self.root.resolve()):
            return None  # Private Tracker inputs may never be selected from a checkout.
        return installation

    def tracker_venv(self):
        explicit, venv = self._explicit(("VT_PYVUT_VENV",))
        if not explicit:
            venv = self.home / ".local/share/visiontactile/pyvut-venv"
        return venv, explicit

    def site_candidates(self):
        venv, explicit = self.tracker_venv()
        if venv is None:
            return (), explicit
        try:
            # Bounded to the one documented venv, with no recursive traversal.
            return tuple(sorted((venv / "lib").glob("python*/site-packages"))), explicit
        except (OSError, RuntimeError, ValueError):
            return (), explicit

    def workstation_bootstrap_software(self, parameters):
        """Use the selected publisher's software, with a bounded venv mapping.

        A configured site directory can identify a venv only through its standard
        layout and pyvenv.cfg marker. Keep the invocation path: resolving Python's
        symlink would discard the environment that supplies its dependencies.
        """
        source = _path(parameters.get("pyvut_source", ""), self.home)
        if (not _matches(source, _markers(_PYVUT_MARKERS))
                or not _matches(source / "scripts/live_windows_bootstrap.py" if source else None, _file)):
            source = None
        site = _path(parameters.get("pyvut_site_packages", ""), self.home)
        python = None
        if (_matches(site, _directory) and site.name == "site-packages"
                and re.fullmatch(r"python[0-9]+\.[0-9]+", site.parent.name)
                and site.parent.parent.name == "lib"):
            venv = site.parents[2]
            candidate = venv / "bin/python"
            if _matches(venv / "pyvenv.cfg", _file) and _matches(candidate, _python):
                python = candidate
        return python, source

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
        if parameters.get("backend") == "vivehub":
            # Never auto-import legacy bootstrap material into a Hub recipe.
            self.select(parameters, group_id, "hub_bindings_path", predicate=_file, manual=True)
            self.select(parameters, group_id, "hub_reader_path", predicate=_file, manual=True)
            return
        bundle = parameters.get("bundle_path") or self.environment.get("VT_BUNDLE", "")
        # A complete manual configuration does not depend on an unrelated
        # installation pointer, which may be absent or awaiting repair.
        complete = all(_matches(_path(parameters.get(key, ""), self.home), predicate)
                       for key, predicate in (
                           ("pyvut_source", _markers(_PYVUT_MARKERS)),
                           ("pyvut_site_packages", _directory),
                           ("bundle_path", _file), ("role_map_path", _file)))
        installation, invalid_installation = None, False
        if not complete:
            try:
                installation = self.tracker_installation(bundle)
            except ValueError:
                # Damaged optional Tracker metadata must not disable cameras or
                # other sensors. Do not substitute private inputs from elsewhere.
                invalid_installation = True
                self.record(group_id, "installation_manifest", "missing",
                            str(self.home / ".local/share/visiontactile/tracker-deployment/current.json"))
        self.select(parameters, group_id, "pyvut_source", predicate=_markers(_PYVUT_MARKERS),
                    repair=True, environment=("VT_PYVUT_ROOT",),
                    candidates=(installation.source,) if installation else self.tracker_sources())
        sites, explicit_venv = self.site_candidates()
        if installation and not explicit_venv:
            sites = (installation.site_packages,)
        self.select(parameters, group_id, "pyvut_site_packages", predicate=_directory,
                    repair=True, environment=("VT_PYVUT_SITE",), candidates=sites)
        bound = (installation is not None
                 and ((not parameters.get("bundle_path") and "VT_BUNDLE" not in self.environment)
                      or _path(bundle, self.home) == installation.bundle))
        self.select(parameters, group_id, "bundle_path",
                    predicate=lambda path: path.is_file() and not path.resolve().is_relative_to(self.root.resolve()),
                    environment=("VT_BUNDLE",),
                    candidates=(installation.bundle,) if bound else (), manual=True)
        self.select(parameters, group_id, "role_map_path", environment=("VT_ROLE_MAP",),
                    candidates=(installation.roles,) if bound else
                               () if invalid_installation else
                               (data / "tracker-validation/roles.yaml",), manual=True)


def discover_bootstrap_configuration(configuration, *, repo_root, home=None, environ=None,
                                     replace_missing=False, workstation_parameters=None) -> DiscoveryResult:
    """Suggest initialization software and one coherent managed input group.

    Source candidates share the publisher's structural checks. A workstation's
    selected source and identifiable virtual environment precede generic paths.
    Existing explicit private inputs remain untouched; empty inputs can use an
    approved installation only when every nonempty bundle/capture/digest matches
    that same installation. Final environment overrides belong to the caller.
    """
    local_home = Path.home() if home is None else Path(home).expanduser().absolute()
    root = Path(repo_root).expanduser().absolute()
    environment = dict(os.environ if environ is None else environ)
    discovery = _Discovery(configuration, root, local_home, environment, replace_missing)
    complete = (all(configuration.get(key) for key in ("bundle", "capture", "expected_sha256"))
                and _matches(_path(configuration.get("python", ""), local_home), _python)
                and _matches(_path(configuration.get("script", ""), local_home), _file))
    installation = None if complete else discovery.tracker_installation(configuration.get("bundle", ""))
    # An unrelated active deployment must not outrank the workstation's own
    # explicitly selected software for another bundle.
    bound = (installation is not None and (not configuration.get("bundle")
             or _path(configuration["bundle"], local_home) == installation.bundle))
    configured_python, configured_source = discovery.workstation_bootstrap_software(workstation_parameters or {})
    venv, explicit_venv = discovery.tracker_venv()
    if explicit_venv:
        python = (venv / "bin/python",) if venv else ()
    elif bound:
        python = (installation.python,)
    elif configured_python:
        python = (configured_python,)
    else:
        python = (venv / "bin/python",) if venv else ()
    discovery.select(discovery.configuration, "", "python", predicate=_python,
                     repair=True, candidates=python)
    explicit, source = discovery._explicit(("VT_PYVUT_ROOT",))
    if explicit:
        sources = (source,)
    elif bound:
        sources = (installation.source,)
    elif configured_source:
        sources = (configured_source,)
    else:
        sources = discovery.tracker_sources()
    scripts = tuple(path / "scripts/live_windows_bootstrap.py" for path in sources
                    if _matches(path, _markers(_PYVUT_MARKERS)))
    discovery.select(discovery.configuration, "", "script", repair=True, candidates=scripts)
    if installation is not None:
        private = {"bundle": str(installation.bundle), "capture": str(installation.capture),
                   "expected_sha256": installation.capture_sha256}
        compatible = all(not configuration.get(key) or configuration[key] == value
                         for key, value in private.items())
        if compatible:
            for key, value in private.items():
                if not configuration.get(key):
                    discovery.set(discovery.configuration, "", key, value)
                discovery.record("", key, "ready", discovery.configuration[key])
    return DiscoveryResult(discovery.configuration, tuple(discovery.findings),
                           tuple(discovery.updated_fields))


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
