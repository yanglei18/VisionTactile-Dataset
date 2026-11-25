from collections import Counter
import numpy as np
import pytest

from vt_capture_manager.optical.video_layout import _assemble, _seam


def test_adjacent_cube_faces_meet_with_opposite_corner_order():
    first = np.float32([[80,70],[220,85],[226,120],[70,110]])
    other = np.float32([[70,110],[226,120],[195,200],[90,195]])
    assert _seam(first, other) == (2, 0)
    assert _seam(first, other+[400,500]) is None


def test_layout_requires_all_six_and_rejects_contradictory_face_evidence():
    # Simulated fixture. Each entry counts independently observed opposing edges.
    votes = {key: Counter({edge: 7}) for key,edge in {
        (0,1):(3,1),(0,3):(1,3),(0,4):(2,3),(0,5):(0,3),(1,2):(3,1),(1,5):(0,0)}.items()}
    assert len(_assemble(votes).faces) == 6
    with pytest.raises(ValueError):
        _assemble({(2,5):Counter({(0,1):4})})
    votes[(1,5)][(2,1)] = 5
    with pytest.raises(ValueError, match="Inconsistent printed edges"):
        _assemble(votes)
