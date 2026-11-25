"""Estimate a hand's glove mount from three labelled, metric dorsal landmarks.

Positions are in metres in the optical frame of the frozen colour image.
The clicked dorsal wrist is a surface reference for the model wrist. Its
unknown offset inside the glove is not observable from these three clicks.
"""
import fcntl
import json
from pathlib import Path

import cv2
import numpy as np

from vt_capture_core.spatial.transforms import RigidTransform

from ..calibration.storage import atomic_json
from .hand_fusion import parse_mounts


def _points(value, shape, label):
    try:
        array = np.asarray(value)
        if array.shape != shape or array.dtype.kind not in "fiu" or not np.isfinite(array).all():
            raise ValueError
        return array.astype(np.float64, copy=False)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label}坐标无效") from exc


def _palm_frame(points):
    wrist, index, little = points
    forward = (index + little) * .5 - wrist
    across = index - little
    length, width = np.linalg.norm(forward), np.linalg.norm(across)
    if min(length, width) < .001:
        raise ValueError("标点太近，请按顺序点手腕、食指根和小指根")
    forward = forward / length
    normal = np.cross(forward, across / width)
    area = np.linalg.norm(normal)
    if area < .15:
        raise ValueError("三个标点接近一条直线，请核对两个指根的位置")
    normal /= area
    # A proper basis on either side, including for mirrored model geometry.
    basis = np.column_stack((forward, np.cross(normal, forward), normal))
    return basis, np.array([length, width])


def fit_mount(hand, neutral_joints_m, camera_points_m, camera_from_cube):
    """Return T_cube_model_wrist for wrist, index MCP, little MCP in click order.

    Use the selected hand's actual neutral MANO points, including its original
    wrist offset. Only the orientation and wrist origin are transferred; the
    registered hand retains the scale of the model used by the glove solver.
    """
    if hand not in ("left", "right") or not isinstance(camera_from_cube, RigidTransform):
        raise ValueError("请先选择左手或右手并取得对应方块的位姿")
    joints = _points(neutral_joints_m, (21, 3), "手模")
    measured = _points(camera_points_m, (3, 3), "标点")
    if np.any(measured[:, 2] <= 0) or camera_from_cube.translation_m[2] <= 0:
        raise ValueError("标点和方块都需要在相机前方")
    reference, reference_size = _palm_frame(joints[[0, 5, 17]])
    observed, observed_size = _palm_frame(measured)
    ratio = observed_size / reference_size
    if np.any(ratio < .4) or np.any(ratio > 2.):
        raise ValueError("标点间距与手掌不符，请核对腕点和指根的深度")
    centre = measured.mean(axis=0)
    dorsal = observed[:, 2] if hand == "right" else -observed[:, 2]
    if dorsal @ (-centre) <= 0:
        raise ValueError("请让手背朝向相机，并核对左右手和两个指根的顺序")
    if np.linalg.norm(centre - camera_from_cube.translation_m) > max(observed_size) + .03:
        raise ValueError("标点离选中的方块太远，请核对手别和标点深度")

    rotation = observed @ reference.T
    rvec = cv2.Rodrigues(rotation)[0].reshape(3)
    angle = np.linalg.norm(rvec)
    xyz = rvec * (.5 if angle == 0 else np.sin(angle / 2) / angle)
    camera_from_wrist = RigidTransform(measured[0], (*xyz, np.cos(angle / 2)))
    return camera_from_cube.inverse().compose(camera_from_wrist)


def save_mount(path, hand, layout_sha256, cube_from_wrist, *, model_rest_sha256=None):
    """Merge a complete validated calibration into a private two-hand file."""
    if hand not in ("left", "right") or not isinstance(cube_from_wrist, RigidTransform):
        raise ValueError("请先完成这只手的三个标点")
    target = Path(path).expanduser().resolve()
    target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    entry = dict(layout_sha256=layout_sha256, cube_from_model_wrist=cube_from_wrist.as_dict())
    if model_rest_sha256 is not None:
        entry["model_rest_sha256"] = model_rest_sha256

    # A persistent sidecar lock covers the read/merge/replace cycle, including
    # two first-time calibrations from separate left/right viewer processes.
    lock_path = target.with_name(target.name + ".lock")
    with lock_path.open("a+b") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            with target.open("rb") as stream:
                raw = stream.read(65537)
        except FileNotFoundError:
            document = {"schema_version": 1, "hands": {}}
        else:
            try:
                if len(raw) > 65536:
                    raise ValueError
                document = json.loads(raw)
                parse_mounts(document)
            except (UnicodeError, ValueError, TypeError) as exc:
                raise ValueError("已有安装参数文件无效，请更换保存位置") from exc

        document["hands"][hand] = entry
        try:
            parse_mounts(document)
        except (TypeError, ValueError) as exc:
            raise ValueError("本次安装参数不完整，请重新取得方块和手模数据") from exc

        atomic_json(target, document)
