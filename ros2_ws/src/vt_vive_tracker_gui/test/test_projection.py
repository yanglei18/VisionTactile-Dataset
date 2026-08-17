import math

import pytest

from vt_vive_tracker_gui.projection import (
    Camera,
    camera_for_view,
    fit_camera,
    orientation_axes,
    project_point,
    project_points,
    rotate_vector,
)


def test_target_projects_to_view_center_with_positive_depth():
    camera = Camera()
    point = project_point(camera.target, camera, 800.0, 600.0)
    assert point is not None
    assert point[0] == pytest.approx(400.0)
    assert point[1] == pytest.approx(300.0)
    assert point[2] > 0.0


def test_point_at_or_behind_camera_is_not_projected():
    camera = Camera(target=(0.0, 0.0, 0.0), yaw=0.0, pitch=0.0, distance=1.0)
    assert project_point((1.0, 0.0, 0.0), camera, 800.0, 600.0) is None
    assert project_point((2.0, 0.0, 0.0), camera, 800.0, 600.0) is None


def test_batch_projection_matches_scalar_for_visible_and_near_plane_points():
    camera = Camera(yaw=0.0, pitch=0.0, distance=1.0)
    points = (
        (-0.2, 0.1, 0.2),
        (0.94, 0.0, 0.0),
        (0.96, 0.0, 0.0),
        (1.0, 0.0, 0.0),
    )
    scalar = tuple(
        project_point(point, camera, 800.0, 600.0) for point in points
    )
    assert project_points(points, camera, 800.0, 600.0) == scalar
    assert scalar[0] is not None
    assert scalar[1] is not None
    assert scalar[2:] == (None, None)


def test_quaternion_rotates_vector_about_world_z():
    half_angle = math.radians(45.0)
    quaternion = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))
    assert rotate_vector((1.0, 0.0, 0.0), quaternion) == pytest.approx(
        (0.0, 1.0, 0.0)
    )


@pytest.mark.parametrize("scale", [1e-300, 1e308])
def test_quaternion_rotation_is_stable_for_extreme_finite_scales(scale):
    assert rotate_vector(
        (1.0, 0.0, 0.0),
        (0.0, 0.0, scale, scale),
    ) == pytest.approx((0.0, 1.0, 0.0))


def test_identity_quaternion_axes_follow_world_xyz():
    axes = orientation_axes(
        (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), length=0.2
    )
    assert axes[0][1] == pytest.approx((1.2, 2.0, 3.0))
    assert axes[1][1] == pytest.approx((1.0, 2.2, 3.0))
    assert axes[2][1] == pytest.approx((1.0, 2.0, 3.2))
    assert tuple(axis[2] for axis in axes) == ("red", "green", "blue")


def test_fit_camera_centers_and_contains_all_tracker_points():
    camera = fit_camera(((-1.0, -2.0, 0.0), (3.0, 2.0, 2.0)), Camera())
    assert camera.target == pytest.approx((1.0, 0.0, 1.0))
    assert camera.distance >= 5.0


def test_fit_camera_preserves_exact_required_distance_for_large_span():
    camera = fit_camera(
        ((-1000.0, 0.0, 0.0), (1000.0, 0.0, 0.0)),
        Camera(),
    )

    assert camera.distance == pytest.approx(3000.0)


def test_fit_camera_leaves_camera_unchanged_without_points():
    camera = Camera(target=(1.0, 2.0, 3.0), distance=7.0)
    assert fit_camera((), camera) == camera


@pytest.mark.parametrize("distance", [math.inf, -math.inf, math.nan])
def test_camera_rejects_non_finite_distance(distance):
    with pytest.raises(ValueError, match="distance"):
        Camera(distance=distance)


def test_camera_orbit_zoom_and_presets_are_bounded():
    assert Camera().orbit(0.0, 100000.0).pitch < math.radians(90.0)
    assert Camera().zoom(1000.0).distance >= 0.25
    assert camera_for_view("top").pitch > math.radians(80.0)
    assert camera_for_view("front") != camera_for_view("side")
    with pytest.raises(ValueError, match="view"):
        camera_for_view("diagonal")
