"""Load the pinned, public FSGlove numerical implementation without its GUI.

This module contains no replacement FK or calibration optimizer. It compiles
unaltered AST nodes from a separately obtained, trusted FSGlove checkout. Its
four source files must match the recorded upstream commit byte for byte. Only
dependency wiring and MANO asset-path arguments differ from upstream execution.

Importing this adapter does not import torch, deserialize MANO assets, install
dependencies, change the working directory, or start any hardware/GUI service.
The checkout and MANO pickle files must be trusted; neither is redistributed.
"""

from __future__ import annotations

import ast
from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum
import hashlib
import importlib
from pathlib import Path
import queue
import subprocess
import threading
from types import ModuleType
from typing import Any, Iterator

import numpy as np
from scipy.spatial.transform import Rotation


UPSTREAM_COMMIT = "a7988f051c35f5ee6157353ee4022d73aa1e4cc8"
_PREFIX = "src/hand_visualiser/modules/"
_PINNED_SOURCES = {
    "calibration": (_PREFIX + "hand/calibration.py", "7b5c4d00cd94a4fa3570b023147bc448f296f8f50a4f4358ea1fec6a50ddd1a5"),
    "utils": (_PREFIX + "utils.py", "e2402273b77c4b329de4937b0cf646b8b3dcf262d6345f2655c711f91498d67b"),
    "enumerates": (_PREFIX + "enumerates.py", "de2bf6e61643add183a41aef944f039b6454b68195f48770b1542e7927439722"),
    "datamodels": (_PREFIX + "datamodels.py", "064ce91160f1363c2f013b2a9d3fec642c90800cef15ed203b698585f7a0d0eb"),
}
CALIBRATION_SYMBOLS = (
    "WristPoseCalibrator", "CalibrationQuatSet", "Raw2MANOCalibrator",
    "GPRInstallationCalibrator", "get_vr_rotation", "InstallationParams",
    "point_match", "keypoints_verts_ids", "get_optimized_shape_params",
)
UTILS_SYMBOLS = (
    "_constant_frame_mano_to_world_rot", "frame_mano_to_world", "get_loss",
    "get_optimized_shape_params", "get_optimized_shape_params_from_keypoints",
)
_RNG_LOCK = threading.RLock()


class UpstreamSourceError(RuntimeError):
    """The external FSGlove source does not match the audited public commit."""


class UpstreamDependencyError(RuntimeError):
    """An optional numerical dependency is missing or incompatible."""


def _node_name(node: ast.AST) -> str | None:
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
        return node.name
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
        return node.targets[0].id
    return None


def _select_nodes(source: str, filename: str, names: tuple[str, ...]) -> tuple[ast.Module, dict[str, str]]:
    """Select entire original nodes, including decorators and default values."""
    tree = ast.parse(source, filename=filename)
    wanted = set(names)
    selected = [node for node in tree.body if _node_name(node) in wanted]
    found = [_node_name(node) for node in selected]
    if len(found) != len(wanted) or set(found) != wanted:
        raise UpstreamSourceError(f"Missing or duplicate upstream numerical symbols in {filename}: {names}")
    hashes = {
        _node_name(node): hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest()
        for node in selected
    }
    # The original objects are inserted without transformation, including line
    # numbers. Compilation cannot inherit this adapter's future annotations.
    return ast.Module(body=selected, type_ignores=[]), hashes


def inspect_upstream_sources(checkout: str | Path) -> dict[str, Any]:
    """Verify source bytes without importing optional packages or model files."""
    root = Path(checkout).expanduser().resolve()
    sources: dict[str, Any] = {}
    for key, (relative, expected) in _PINNED_SOURCES.items():
        path = root / relative
        if not path.is_file():
            raise UpstreamSourceError(f"Required FSGlove source is missing: {path}")
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != expected:
            raise UpstreamSourceError(
                f"FSGlove source hash mismatch for {path}; expected commit {UPSTREAM_COMMIT}. "
                "The adapter will not silently execute a different or modified algorithm."
            )
        sources[key] = {"path": str(path), "relative_path": relative, "sha256": actual}
    try:
        result = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"], check=True,
            capture_output=True, text=True, timeout=5,
        )
        git_commit = result.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        git_commit = None
    return {
        "upstream_project": "davidliyutong/fsglove",
        "upstream_commit": UPSTREAM_COMMIT,
        "checkout": str(root),
        "checkout_head": git_commit,
        "sources": sources,
        "source_validation": "exact_pinned_file_sha256",
        "numerical_ast_modified": False,
        "adapter_changes": [
            "Select original numerical definitions; do not import GUI, network, or hardware modules.",
            "Supply explicit MANO asset paths through local real-manotorch factories.",
            "Provide opt-in isolated CPU random-number seeding around official initialization/optimization.",
        ],
        "public_core_limitations": [
            "Raw2MANOCalibrator requires all 16 quaternion slots; sparse completion is an external adapter.",
            "The official HandStateMachine installation CALC branch is a TODO, not a published joint optimizer.",
            "Official pinch shape optimization updates MANO beta only, not pose and installation jointly.",
            "Upstream numerical quirks are preserved; this is not a claim of complete paper reproduction.",
        ],
        "redistribution_note": "External checkout is executed locally; no FSGlove source or MANO assets are bundled.",
    }


def _compile_selected(key: str, names: tuple[str, ...], provenance: dict[str, Any], namespace: dict[str, Any]) -> ModuleType:
    record = provenance["sources"][key]
    filename = record["path"]
    tree, symbol_hashes = _select_nodes(Path(filename).read_text(encoding="utf-8"), filename, names)
    module = ModuleType(f"_vt_fsglove_official_{key}_{record['sha256'][:12]}")
    module.__file__ = filename
    module.__dict__.update(namespace)
    exec(compile(tree, filename, "exec", dont_inherit=True), module.__dict__)
    record["symbols_ast_sha256"] = symbol_hashes
    return module


def _dependency(name: str) -> ModuleType:
    try:
        return importlib.import_module(name)
    except Exception as exc:
        raise UpstreamDependencyError(
            f"Cannot import official-core dependency {name}: {exc}. "
            "Install the tested optional upstream-core environment; nothing was installed automatically."
        ) from exc


def _asset_factory(real_class: Any, assets_root: Path):
    def factory(*args, **kwargs):
        kwargs["mano_assets_root"] = str(assets_root)
        return real_class(*args, **kwargs)

    factory.__name__ = real_class.__name__
    factory.__wrapped__ = real_class
    return factory


@dataclass
class UpstreamCore:
    """Original upstream symbols plus explicit execution/provenance helpers."""

    calibration: ModuleType
    utils: ModuleType
    enumerates: ModuleType
    datamodels: ModuleType
    provenance: dict[str, Any]
    mano_assets_root: Path
    torch: Any

    @contextmanager
    def seeded(self, seed: int = 0) -> Iterator[None]:
        """Reproduce upstream's CPU random initialization and restore RNG state.

        Official optimizers default to CPU. This scope does not initialize or
        seed CUDA and does not promise reproducibility for a caller's CUDA work.
        Our scopes are serialized; unrelated threads using the global RNG must
        not be run concurrently if their own random stream must be preserved.
        """
        if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**32:
            raise ValueError("seed must be an integer in [0, 2**32)")
        with _RNG_LOCK:
            numpy_state = np.random.get_state()
            with self.torch.random.fork_rng(devices=[]):
                try:
                    np.random.seed(seed)
                    self.torch.random.default_generator.manual_seed(seed)
                    yield
                finally:
                    np.random.set_state(numpy_state)

    def require_model(self, side: str = "right") -> Path:
        if side not in ("right", "left"):
            raise ValueError("side must be 'right' or 'left'")
        path = self.mano_assets_root / "models" / f"MANO_{side.upper()}.pkl"
        if not path.is_file():
            raise UpstreamDependencyError(f"Trusted, separately obtained MANO model is missing: {path}")
        return path

    def create_calibrator(self, base_pose, z_rot_pose_1, z_rot_pose_2, *, side: str = "right", seed: int = 0):
        """Validate the 16-slot boundary, then call the unmodified constructor."""
        self.require_model(side)
        poses = {}
        for name, value in (
            ("base_pose", base_pose), ("z_rot_pose_1", z_rot_pose_1), ("z_rot_pose_2", z_rot_pose_2),
        ):
            array = np.asarray(value, dtype=float)
            if array.shape != (16, 4) or not np.isfinite(array).all():
                raise ValueError(f"{name} must be a finite (16, 4) xyzw quaternion array")
            if not np.allclose(np.linalg.norm(array, axis=1), 1, atol=1e-5, rtol=0):
                raise ValueError(f"{name} must contain unit xyzw quaternions")
            poses[name] = array.copy()
        with self.seeded(seed):
            return self.calibration.Raw2MANOCalibrator(self.calibration.CalibrationQuatSet(**poses), side=side)

    def optimize_shape(self, joints, finger_category, hand_category: str = "right", *, seed: int = 0, **kwargs):
        """Call the official pinch-contact beta optimizer, without replacing it."""
        self.require_model(hand_category)
        with self.seeded(seed):
            return self.utils.get_optimized_shape_params(joints, finger_category, hand_category, **kwargs)


def load_upstream_core(checkout: str | Path, mano_assets_root: str | Path) -> UpstreamCore:
    """Load all published calibration numerics, not the upstream GUI state loop.

    The model is not deserialized until a caller constructs a calibrator or
    executes an optimizer. Real dependency imports are lazy at this boundary.
    """
    provenance = inspect_upstream_sources(checkout)
    assets = Path(mano_assets_root).expanduser().resolve()
    torch = _dependency("torch")
    manolayer = _dependency("manotorch.manolayer")
    axislayer = _dependency("manotorch.axislayer")
    pydantic = _dependency("pydantic")
    gaussian_process = _dependency("sklearn.gaussian_process")
    kernels = _dependency("sklearn.gaussian_process.kernels")
    tqdm = _dependency("tqdm")
    scipy_optimize = _dependency("scipy.optimize")
    namespace = {
        "torch": torch, "np": np, "R": Rotation, "nn": torch.nn, "Any": Any,
        "Enum": Enum, "queue": queue, "tqdm": tqdm.tqdm, "leastsq": scipy_optimize.leastsq,
        "ManoLayer": _asset_factory(manolayer.ManoLayer, assets), "MANOOutput": manolayer.MANOOutput,
        "AxisLayerFK": _asset_factory(axislayer.AxisLayerFK, assets),
        "GaussianProcessRegressor": gaussian_process.GaussianProcessRegressor,
        "ConstantKernel": kernels.ConstantKernel, "RBF": kernels.RBF,
    }
    for name in ("BaseModel", "model_validator", "ConfigDict", "field_serializer"):
        try:
            namespace[name] = getattr(pydantic, name)
        except AttributeError as exc:
            raise UpstreamDependencyError(f"pydantic must supply {name}; the pinned upstream requires pydantic v2") from exc
    enumerates = _compile_selected("enumerates", ("MoCapHandIDEnum",), provenance, namespace)
    datamodels = _compile_selected("datamodels", ("WristTrackerMsg",), provenance, namespace)
    namespace.update(MoCapHandIDEnum=enumerates.MoCapHandIDEnum, WristTrackerMsg=datamodels.WristTrackerMsg)
    utils = _compile_selected("utils", UTILS_SYMBOLS, provenance, namespace)
    namespace["frame_mano_to_world"] = utils.frame_mano_to_world
    calibration = _compile_selected("calibration", CALIBRATION_SYMBOLS, provenance, namespace)
    provenance["mano_assets_root"] = str(assets)
    provenance["runtime_dependencies"] = {
        name: {"version": getattr(module, "__version__", None), "file": getattr(module, "__file__", None)}
        for name, module in (("torch", torch), ("manotorch.manolayer", manolayer),
                             ("manotorch.axislayer", axislayer), ("pydantic", pydantic),
                             ("sklearn", _dependency("sklearn")), ("tqdm", tqdm))
    }
    return UpstreamCore(calibration, utils, enumerates, datamodels, provenance, assets, torch)
