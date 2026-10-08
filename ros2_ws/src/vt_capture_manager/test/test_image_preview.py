import base64
from types import SimpleNamespace as NS

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

from vt_capture_manager.image_preview import jpeg_preview


def decode(value):
    return cv2.imdecode(np.frombuffer(base64.b64decode(value["data_b64"]), np.uint8), cv2.IMREAD_UNCHANGED)


def test_rgb_stride_padding_and_channel_order():
    # A 16x8 red image with non-image trailing padding on each row.
    row = bytes([255, 0, 0])*16 + bytes([0, 255, 0])*2
    preview = jpeg_preview(NS(width=16, height=8, step=54, encoding="rgb8", is_bigendian=0, data=row*8))
    pixels = decode(preview)
    assert pixels.shape == (8, 16, 3)
    assert pixels[..., 2].mean() > 240 and pixels[..., 0].mean() < 10


def test_uint16_endianness_has_same_visualization_no_metric_scale():
    raw = np.arange(64, dtype=np.uint16).reshape(8, 8)*20
    base = dict(width=8, height=8, step=16, encoding="16UC1")
    left = jpeg_preview(NS(**base, is_bigendian=0, data=raw.astype("<u2").tobytes()))
    right = jpeg_preview(NS(**base, is_bigendian=1, data=raw.astype(">u2").tobytes()))
    assert left == right
    assert set(left) == {"width", "height", "format", "data_b64"}


def test_downscale_and_invalid_shape():
    preview = jpeg_preview(NS(width=640, height=480, step=640, encoding="mono8", is_bigendian=0, data=bytes(640*480)))
    assert (preview["width"], preview["height"]) == (320, 240)
    with pytest.raises(ValueError):
        jpeg_preview(NS(width=20, height=20, step=1, encoding="rgb8", is_bigendian=0, data=bytes(20)))


def test_float_depth_invalid_pixels_endianness_and_no_units():
    raw = np.array([float("nan"), float("inf"), -2., 0., 1., 2., 3., 4.]*8, dtype="<f4").reshape(8, 8)
    base = dict(width=8, height=8, step=32, encoding="32FC1")
    result = jpeg_preview(NS(**base, is_bigendian=0, data=raw.tobytes()))
    big = jpeg_preview(NS(**base, is_bigendian=1, data=raw.astype(">f4").tobytes()))
    assert result == big
    pixels = decode(result)
    assert pixels[:, :4].max() < 15  # JPEG is lossy; invalid pixels stay dark.
    assert pixels[:, -1].mean() > 240
