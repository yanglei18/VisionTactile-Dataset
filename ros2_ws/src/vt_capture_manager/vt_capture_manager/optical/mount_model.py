"""Neutral wrist-relative hand landmarks from the selected workbench glove bundle."""
from dataclasses import dataclass
import hashlib
import json

import numpy as np


@dataclass(frozen=True)
class MountModel:
    joints_m: np.ndarray
    rest_sha256: str


def load_mount_model(resources_path, hand):
    """Use the same verified bundle and canonical rest identity as the live glove.

    Only numeric JSON is loaded. The manifest verifier hashes the associated model
    asset as part of the normal service contract and never executes it.
    """
    if hand not in ("left", "right"):
        raise ValueError("请选择左手或右手。")
    side = "左手" if hand == "left" else "右手"
    if not resources_path:
        raise ValueError(f"请先在工作台中为{side}选择动捕模型资源包。")
    try:
        from vt_fsglove12.calibration import validate_rest
        from vt_fsglove12.config import digest
        from vt_fsglove12.hand_service_resources import MAX_JSON_BYTES, validate_bundle_manifest
    except ImportError as exc:
        raise ValueError("无法载入手套模型校验组件，请通过候选工作台打开标定窗口。") from exc

    try:
        root, manifest = validate_bundle_manifest(resources_path, hand=hand)

        def checked_document(name):
            with (root / name).open("rb") as stream:
                raw = stream.read(MAX_JSON_BYTES + 1)
            if len(raw) > MAX_JSON_BYTES or hashlib.sha256(raw).hexdigest() != manifest["files"][name]:
                raise ValueError("Model source changed during loading")
            return json.loads(raw)

        rest = checked_document("rest.json")
        display = checked_document("hand.json")
        validate_rest(rest)
        geometry = display["neutral_geometry"]
        provenance = geometry["provenance"]
        rest_origin = rest.get("provenance", {})
        if (checked_document("config.json").get("hand", "right") != hand
                or display["schema_version"] != 1 or display["model_axes"]["hand"] != hand
                or provenance.get("hand") != hand or rest_origin.get("hand", "right") != hand
                or provenance.get("geometry_units", "metres") != "metres"
                or any(provenance[k] != rest_origin[k] for k in
                       ("geometry_units", "geometry_origin", "betas", "backend")
                       if k in provenance and k in rest_origin)):
            raise ValueError("Mismatched neutral model")
        data = np.asarray(geometry["joints_m"])
        if (data.shape != (21, 3) or data.dtype.kind not in "fiu"
                or not np.isfinite(data).all() or np.any(np.abs(data.astype(np.float64)) > 2.)):
            raise ValueError("Invalid joint geometry")
        joints = np.array(data, dtype=np.float64) - data[0]
        joints.setflags(write=False)
        return MountModel(joints, digest(rest))
    except (OSError, TypeError, KeyError, ValueError, OverflowError, AttributeError, RuntimeError) as exc:
        raise ValueError(f"{side}模型资源未通过校验，请核对工作台选择的资源包。") from exc
