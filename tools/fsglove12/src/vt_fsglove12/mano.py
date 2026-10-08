"""Optional, independently written bridge to manotorch's real MANO model.

No FSGlove implementation is copied here.  Importing this module does not import
torch/manotorch, load a pickle, accept a license, or acquire model assets.  MANO
pickle files must be trusted files obtained separately by the user.

The 16 pose slots use MANO order, whereas the 21 returned joints use manotorch's
wrist/thumb/index/middle/ring/little order. Outputs default to JSON-compatible
lists; explicit compact live output keeps only surface arrays in NumPy form.
All geometry is wrist-relative and in metres. The legacy Euler path uses zero
shape parameters; render_mano forwards official axis-angle and explicit betas.
"""

from __future__ import annotations

import importlib
import importlib.metadata
import importlib.util
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation


FSGLOVE_REFERENCE_COMMIT = "a7988f051c35f5ee6157353ee4022d73aa1e4cc8"


class DependencyError(RuntimeError):
    """An optional package or user-provided model asset is unavailable."""


class ModelResourceError(DependencyError):
    """A model asset is missing, unreadable, or cannot be initialized."""


class BackendCompatibilityError(RuntimeError):
    """The installed backend does not satisfy the checked MANO API contract."""


def _asset_paths(assets_root: str | Path, hand: str) -> tuple[Path, Path]:
    if hand not in ("right", "left"):
        raise ValueError("hand must be 'right' or 'left'")
    root = Path(assets_root).expanduser().resolve()
    return root, root / "models" / f"MANO_{hand.upper()}.pkl"


def doctor(assets_root: str | Path | None = None, hand: str = "right") -> dict[str, Any]:
    """Inspect presence only; do not import optional modules or load a model.

    A successful result is a prerequisites check, not a successful model load or
    a validation of an installed package's version/API compatibility.
    """
    if hand not in ("right", "left"):
        raise ValueError("hand must be 'right' or 'left'")
    issues: list[str] = []
    modules: dict[str, dict[str, Any]] = {}
    for name in ("torch", "manotorch"):
        try:
            spec = importlib.util.find_spec(name)
        except (ImportError, ValueError, AttributeError) as exc:
            spec = None
            issues.append(f"Cannot inspect optional package {name}: {exc}")
        modules[name] = {"available": spec is not None, "path": getattr(spec, "origin", None)}
        if spec is None:
            issues.append(f"Optional package {name} is unavailable; no automatic installation is performed.")
    root: Path | None = None
    model: Path | None = None
    if assets_root is None:
        issues.append("Provide assets_root containing models/MANO_RIGHT.pkl (or MANO_LEFT.pkl for left hand).")
    else:
        root, model = _asset_paths(assets_root, hand)
        if not model.is_file():
            issues.append(f"MANO model is missing: {model}. Obtain licensed assets separately; nothing is downloaded.")
    return {
        "ok": not issues,
        "check": "presence_only_not_model_validation",
        "issues": issues,
        "assets_root": str(root) if root else None,
        "model_file": str(model) if model else None,
        "modules": modules,
    }


def _numpy(value: Any) -> np.ndarray:
    return np.asarray(value.detach().cpu().numpy())


def _checked_array(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite array with shape {shape}; got {result.shape}")
    return result


class _ReadOnlyGeometryArray(np.ndarray):
    """Small IPC array retaining read-only ownership across deepcopy/pickle.

    NumPy's ordinary deepcopy makes read-only arrays writable. Snapshot copies
    remain independent here without expanding thousands of Python numbers.
    """
    def __deepcopy__(self, memo):
        result = _readonly_geometry_array(self)
        memo[id(self)] = result
        return result

    def __reduce__(self):
        return _readonly_geometry_array, (np.asarray(self),)


def _readonly_geometry_array(value, dtype=None):
    # Never retain a tensor-backed view: a later backend forward may reuse it.
    storage = np.array(value, dtype=dtype, order="C", copy=True)
    storage.setflags(write=False)
    result = storage.view(_ReadOnlyGeometryArray)
    result.setflags(write=False)
    return result


class ManoBackend:
    """CPU/inference-only real MANO bridge with explicit root-rotation handling.

    ``assets_root`` is the directory *containing* ``models/``, not the models
    directory itself. Construction validates that the selected model exists;
    optional imports and model deserialization wait until the first operation.
    There is deliberately no synthetic fallback or inferred hand shape.
    """

    def __init__(self, assets_root: str | Path, hand: str = "right", *,
                 compact_live_geometry: bool = False) -> None:
        if type(compact_live_geometry) is not bool:
            raise ValueError("compact_live_geometry must be an explicit boolean")
        self.assets_root, self.model_file = _asset_paths(assets_root, hand)
        self.hand = hand
        self.compact_live_geometry = compact_live_geometry
        self._compact_faces = None
        if not self.model_file.is_file():
            raise ModelResourceError(
                f"MANO model is missing: {self.model_file}. assets_root must contain models/. "
                "Obtain the licensed MANO assets separately; this bridge does not download or authorize them."
            )
        self._torch: Any = None
        self._mano: Any = None
        self._axis: Any = None
        self._provenance: dict[str, Any] | None = None

    def _ensure_loaded(self) -> None:
        if self._mano is not None:
            return
        imported: dict[str, Any] = {}
        for name in ("torch", "manotorch", "manotorch.manolayer", "manotorch.axislayer"):
            try:
                imported[name] = importlib.import_module(name)
            except (ImportError, OSError) as exc:
                raise DependencyError(
                    f"Cannot import optional MANO dependency {name}: {exc}. "
                    "Install a compatible backend separately; no automatic installation is performed."
                ) from exc

        torch = imported["torch"]
        try:
            mano = imported["manotorch.manolayer"].ManoLayer(
                mano_assets_root=str(self.assets_root),
                side=self.hand,
                use_pca=False,
                flat_hand_mean=True,
                rot_mode="axisang",
                center_idx=0,
            ).eval()
            # AxisLayerFK builds its own zero-pose, zero-beta anatomical frame.
            axis = imported["manotorch.axislayer"].AxisLayerFK(
                mano_assets_root=str(self.assets_root), side=self.hand
            ).eval()
        except ImportError as exc:
            raise DependencyError(f"MANO initialization needs an unavailable dependency: {exc}") from exc
        except Exception as exc:
            raise ModelResourceError(
                f"Failed to initialize MANO/AxisLayerFK using {self.model_file}: "
                f"{type(exc).__name__}: {exc}. Check the supplied model and backend compatibility; "
                "this is not a successful model load."
            ) from exc

        package = imported["manotorch"]
        try:
            version = importlib.metadata.version("manotorch")
        except importlib.metadata.PackageNotFoundError:
            version = getattr(package, "__version__", None) or "unknown (no installed distribution metadata)"
        self._provenance = {
            "backend": "manotorch.ManoLayer+AxisLayerFK",
            "fsglove_reference_commit": FSGLOVE_REFERENCE_COMMIT,
            "fsglove_source_copied": False,
            "manotorch_path": str(Path(package.__file__).resolve()) if getattr(package, "__file__", None) else None,
            "manotorch_version": version,
            "manolayer_path": str(Path(imported["manotorch.manolayer"].__file__).resolve()),
            "axislayer_path": str(Path(imported["manotorch.axislayer"].__file__).resolve()),
            "torch_version": getattr(torch, "__version__", "unknown"),
            "model_file": str(self.model_file),
            "hand": self.hand,
            "betas": [0.0] * 10,
            "shape_calibrated": False,
            "quaternion_order": "xyzw",
            "anatomical_euler_convention": "intrinsic XYZ radians",
            "geometry_units": "metres",
            "geometry_origin": "wrist (joint 0)",
            "parents_root_sentinel": -1,
        }
        self._torch, self._mano, self._axis = torch, mano, axis

    def _tensor(self, array: Any) -> Any:
        return self._torch.as_tensor(array, dtype=self._torch.float32, device="cpu")

    def _surface_output(self, vertices, faces):
        if not self.compact_live_geometry:
            return vertices.tolist(), faces.astype(np.int64).tolist()
        if self._compact_faces is None:
            # MANO model topology is immutable for this backend's lifetime.
            self._compact_faces = _readonly_geometry_array(faces, dtype=np.int64)
        return _readonly_geometry_array(vertices), self._compact_faces

    def export_rest(self) -> dict[str, Any]:
        """Return actual model-derived global anatomical rest frames, not a guess.

        Parents come from AxisLayerFK, with root's self-parent changed to -1.
        Rotations are expressed in the neutral MANO coordinate system; they are
        not measured IMU orientations and carry no global heading information.
        """
        self._ensure_loaded()
        with self._torch.no_grad():
            output = self._mano(self._tensor(np.zeros((1, 48))), self._tensor(np.zeros((1, 10))))
            anatomical_transforms = self._axis(output.transforms_abs)[0]
        transforms = _numpy(anatomical_transforms)
        if transforms.shape != (1, 16, 4, 4) or not np.isfinite(transforms).all():
            raise BackendCompatibilityError("AxisLayerFK must produce finite transforms shaped (1, 16, 4, 4)")
        matrices = transforms[0, :, :3, :3]
        if not (np.allclose(matrices.transpose(0, 2, 1) @ matrices, np.eye(3), atol=1e-4)
                and np.allclose(np.linalg.det(matrices), 1.0, atol=1e-4)):
            raise BackendCompatibilityError("AxisLayerFK rest frames are not proper rotation matrices")
        parents = np.asarray(self._axis.transf_parent_mapping, dtype=np.int64).copy()
        if parents.shape != (16,) or parents[0] not in (0, -1):
            raise BackendCompatibilityError("AxisLayerFK parent mapping is not a 16-slot rooted tree")
        if np.any(parents[1:] < 0) or np.any(parents[1:] >= np.arange(1, 16)):
            raise BackendCompatibilityError("AxisLayerFK parents must precede their children")
        parents[0] = -1
        return {
            "rest_global_xyzw": Rotation.from_matrix(matrices).as_quat().tolist(),
            "parents": parents.tolist(),
            "provenance": dict(self._provenance),
        }

    def render(self, anatomical_euler_rad: Any, root_quat_xyzw: Any = None) -> dict[str, Any]:
        """Render the supplied pose, without estimating missing sensors or beta.

        Slots 1..15 are intrinsic XYZ anatomical local angles in radians. If no
        explicit root is supplied, slot 0 denotes a global MANO-root XYZ rotation.
        ``root_quat_xyzw`` overrides slot 0 and is a delta rotation of the MANO
        root, NOT an absolute anatomical sensor-frame orientation. For an observed
        anatomical root, the caller must first compute R_obs @ inv(R_rest_root).

        AxisLayerFK.compose cancels its self-parented root. We compose only the
        child rotations, then explicitly restore the global root axis-angle in
        the ManoLayer input; geometry is not rotated a second time afterwards.
        """
        angles = _checked_array(anatomical_euler_rad, (16, 3), "anatomical_euler_rad").copy()
        if root_quat_xyzw is None:
            root = Rotation.from_euler("XYZ", angles[0])
        else:
            quaternion = _checked_array(root_quat_xyzw, (4,), "root_quat_xyzw")
            norm = np.linalg.norm(quaternion)
            if not np.isfinite(norm) or norm < 1e-12:
                raise ValueError("root_quat_xyzw must have nonzero finite norm")
            root = Rotation.from_quat(quaternion / norm)
        angles[0] = 0.0
        self._ensure_loaded()
        with self._torch.no_grad():
            composed = self._axis.compose(self._tensor(angles[None]))
            pose = _numpy(composed).copy()
            if pose.shape != (1, 16, 3) or not np.isfinite(pose).all():
                raise BackendCompatibilityError("AxisLayerFK.compose must produce a finite (1, 16, 3) pose")
            pose[0, 0] = root.as_rotvec()
            output = self._mano(self._tensor(pose.reshape(1, 48)), self._tensor(np.zeros((1, 10))))
        vertices, joints, faces = _numpy(output.verts), _numpy(output.joints), _numpy(self._mano.th_faces)
        if vertices.shape != (1, 778, 3) or joints.shape != (1, 21, 3):
            raise BackendCompatibilityError("ManoLayer must return vertices (1, 778, 3) and joints (1, 21, 3)")
        if not np.isfinite(vertices).all() or not np.isfinite(joints).all():
            raise BackendCompatibilityError("ManoLayer returned non-finite geometry")
        if (faces.ndim != 2 or faces.shape[1] != 3 or not np.isfinite(faces).all()
                or np.any(faces != np.floor(faces)) or np.any(faces < 0) or np.any(faces >= 778)):
            raise BackendCompatibilityError("ManoLayer returned invalid triangular face indices")
        surface, triangles = self._surface_output(vertices[0], faces)
        return {
            "vertices_m": surface,
            "joints_m": joints[0].tolist(),
            "faces": triangles,
            "mano_pose_rotvec": pose[0].tolist(),
            "provenance": dict(self._provenance),
        }

    def render_mano(self, mano_pose_rotvec: Any, shape_params: Any = None) -> dict[str, Any]:
        """Render official MANO parameters directly, without re-solving joints.

        This is intentionally separate from anatomical Euler composition. The
        FSGlove root is already present in its MANO output; do not apply it twice.
        The default output is JSON-compatible. ``compact_live_geometry=True``
        on construction instead returns independent read-only NumPy surface
        arrays for local snapshots/IPC; joints and pose remain JSON lists.
        """
        pose = _checked_array(mano_pose_rotvec, (16, 3), "mano_pose_rotvec")
        betas = _checked_array(np.zeros(10) if shape_params is None else shape_params, (10,), "shape_params")
        self._ensure_loaded()
        with self._torch.no_grad():
            output = self._mano(self._tensor(pose.reshape(1, 48)), self._tensor(betas.reshape(1, 10)))
        vertices, joints, faces = _numpy(output.verts), _numpy(output.joints), _numpy(self._mano.th_faces)
        if (vertices.shape != (1, 778, 3) or joints.shape != (1, 21, 3)
                or not np.isfinite(vertices).all() or not np.isfinite(joints).all()):
            raise BackendCompatibilityError("ManoLayer returned invalid official-pose geometry")
        surface, triangles = self._surface_output(vertices[0], faces)
        return dict(vertices_m=surface, joints_m=joints[0].tolist(),
                    faces=triangles, mano_pose_rotvec=pose.tolist(),
                    provenance={**self._provenance, "betas": betas.tolist(),
                                "pose_input": "upstream_mano_axis_angle_direct"})
