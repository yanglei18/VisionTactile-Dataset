"""The MANO 21-point measurement without constructing the complete surface.

The five fingertips are skinned surface vertices, not extensions of bones.
This evaluates exactly those vertices together with the 16 articulated joints.
All buffers originate from the already loaded, hand-specific MANO layer.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

_PARENTS = (0, 0, 1, 2, 0, 4, 5, 0, 7, 8, 0, 10, 11, 0, 13, 14)
_ORDER = (0, 13, 14, 15, 16, 1, 2, 3, 17, 4, 5, 6, 18, 10, 11, 12, 19, 7, 8, 9, 20)


class ManoJointEvaluator:
    def __init__(self, layer, hand):
        if (hand not in ('left', 'right') or layer.center_idx != 0
                or layer.use_pca or not layer.flat_hand_mean or layer.rot_mode != 'axisang'
                or tuple(layer.kintree_parents[1:]) != _PARENTS[1:]):
            raise ValueError('Unsupported MANO measurement convention')
        def buffer(name, shape):
            value = np.array(getattr(layer, name).detach().cpu().numpy(), dtype=np.float32, copy=True)
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError(f'Invalid MANO measurement buffer: {name}')
            return value
        self.template = buffer('th_v_template', (1, 778, 3))[0]
        self.shapedirs = buffer('th_shapedirs', (778, 3, 10))
        self.regressor = buffer('th_J_regressor', (16, 778))
        tips = [745, 317, 445 if hand == 'left' else 444, 556, 673]
        self.tips = tips
        self.posedirs = buffer('th_posedirs', (778, 3, 135))[tips]
        self.weights = buffer('th_weights', (778, 16))[tips]
        self._shape_key = None

    def joints(self, pose, betas):
        # The reference backend consumes float32 axis angles and betas.
        pose = np.asarray(pose, dtype=np.float32)
        betas = np.asarray(betas, dtype=np.float32)
        key = betas.tobytes()
        if key != self._shape_key:
            shaped = self.template + np.einsum('vci,i->vc', self.shapedirs, betas)
            self._rest = self.regressor @ shaped
            self._tips = shaped[self.tips].copy()
            self._shape_key = key
        rotations = Rotation.from_rotvec(pose).as_matrix().astype(np.float32)
        transforms = np.tile(np.eye(4, dtype=np.float32), (16, 1, 1))
        transforms[:, :3, :3] = rotations
        transforms[0, :3, 3] = self._rest[0]
        for index in range(1, 16):
            parent = _PARENTS[index]
            transforms[index, :3, 3] = self._rest[index] - self._rest[parent]
            transforms[index] = transforms[parent] @ transforms[index]
        joints = transforms[:, :3, 3].copy()
        transforms[:, :3, 3] -= np.einsum('kij,kj->ki', transforms[:, :3, :3], self._rest)
        feature = (rotations[1:] - np.eye(3, dtype=np.float32)).reshape(135)
        posed_tips = self._tips + np.einsum('vci,i->vc', self.posedirs, feature)
        blend = np.einsum('vk,kij->vij', self.weights, transforms)
        tips = np.einsum('vij,vj->vi', blend[:, :3, :3], posed_tips) + blend[:, :3, 3]
        return np.concatenate((joints, tips))[_ORDER, :] - joints[0]
