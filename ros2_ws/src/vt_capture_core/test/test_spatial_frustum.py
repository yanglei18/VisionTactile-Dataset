import pytest

from vt_capture_core.spatial.frustum import frustum_vertices


def test_optical_frustum_uses_intrinsics_and_positive_z():
    vertices = frustum_vertices(width=640, height=480, fx=320, fy=240, cx=320, cy=240, near_m=.1, far_m=1)
    assert len(vertices) == 8
    assert vertices[0] == pytest.approx((-.1, -.1, .1))
    assert vertices[6] == pytest.approx((1, 1, 1))


@pytest.mark.parametrize('kwargs', [{'fx': 0}, {'far_m': .01}, {'distortion_model': 'fisheye'}, {'distortion': [.1]}])
def test_unsupported_or_invalid_projection_rejected(kwargs):
    parameters = dict(width=640, height=480, fx=320, fy=240, cx=320, cy=240)
    parameters.update(kwargs)
    with pytest.raises(ValueError):
        frustum_vertices(**parameters)


def test_intrinsics_dictionary_d_coefficients_are_not_silently_ignored():
    with pytest.raises(ValueError, match='distortion'):
        frustum_vertices(dict(width=640, height=480, fx=320, fy=240, cx=320, cy=240,
                              distortion_model='plumb_bob', d=[.1, 0, 0, 0, 0]))
