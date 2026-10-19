from types import SimpleNamespace as NS
import math
import pytest

from vt_capture_manager.spatial.ros_adapter import rotation_from_column_major
from vt_capture_manager.spatial.config import load_scene_config


def test_extrinsics_column_major_rotation_and_reflection_rejection():
    q = rotation_from_column_major([0., 1., 0., -1., 0., 0., 0., 0., 1.])
    assert q == pytest.approx((0., 0., math.sqrt(.5), math.sqrt(.5)))
    with pytest.raises(ValueError):
        rotation_from_column_major([1., 0., 0., 0., 1., 0., 0., 0., -1.])
    with pytest.raises(ValueError):
        rotation_from_column_major([2., 0., 0., 0., 1., 0., 0., 0., 1.])


def test_depth_units_require_named_publisher_contract(tmp_path):
    path = tmp_path/'scene.yaml'
    path.write_text('schema_version: 1\ndepth_units:\n  cam: {meters_per_unit: 0.0001, source: realsense_ros_z16_mm}\n')
    with pytest.raises(ValueError):
        load_scene_config(path)
