"""Real asset opt-in: independently evaluate all 21 points for both hands."""
import os
from pathlib import Path

import numpy as np
import pytest

from vt_fsglove12.mano import ManoBackend


@pytest.mark.parametrize('hand', ['right', 'left'])
def test_minimal_points_equal_complete_surface_for_pose_shape_and_root(hand):
    root = os.environ.get(f'FSGLOVE12_MANO_{hand.upper()}_ASSETS')
    if not root or not (Path(root) / 'models' / f'MANO_{hand.upper()}.pkl').is_file():
        pytest.skip(f'Explicit {hand} model assets not supplied')
    import torch
    torch.set_num_threads(1)
    backend, rng = ManoBackend(root, hand), np.random.default_rng(23)
    for index in range(120):
        pose = rng.normal(0, .8, (16, 3))
        if index % 20 == 0:
            shape = rng.normal(0, 1.5, 10)
        full = backend.render_mano(pose, shape)
        measured = backend.measure_mano(pose, shape)
        np.testing.assert_allclose(measured['joints_m'], full['joints_m'], atol=1e-6, rtol=0)
        assert measured['provenance']['hand'] == hand
        assert measured['provenance']['betas'] == shape.tolist()
        assert 'vertices_m' not in measured
