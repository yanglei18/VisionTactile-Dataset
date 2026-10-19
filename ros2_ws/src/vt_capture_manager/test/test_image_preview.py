import base64
from array import array
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


@pytest.mark.parametrize("endian,flag", [("<", 0), (">", 1)])
def test_close_range_depth_contrast_is_independent_of_units(endian, flag):
    millimetres = np.tile(np.arange(100, 201, dtype=np.uint16), (32, 1))
    metres = millimetres.astype(np.float32) / 1000
    images = []
    for raw, encoding, dtype in ((millimetres, "16UC1", "u2"), (metres, "32FC1", "f4")):
        images.append(decode(jpeg_preview(NS(
            width=101, height=32, step=101*raw.itemsize, encoding=encoding,
            is_bigendian=flag, data=raw.astype(endian+dtype).tobytes()))))
    assert images[1].min() < 10
    assert images[1].max() > 240
    # JPEG rounding can differ slightly after the float conversion.
    assert np.abs(images[0].astype(float)-images[1]).mean() < 2


def test_constant_float_depth_and_invalid_values_remain_finite():
    raw = np.full((32, 32), .15, dtype=np.float32)
    raw[:, :16] = np.nan
    with np.errstate(divide="raise", invalid="raise"):
        result = decode(jpeg_preview(NS(width=32, height=32, step=128,
            encoding="32FC1", is_bigendian=0, data=raw.tobytes())))
    assert result.shape == (32, 32)
    assert result[:, :16].max() < 10


@pytest.mark.parametrize("encoding,kind", [("16UC1", "u2"), ("mono16", "u2"), ("32FC1", "f4")])
def test_downsampled_depth_preserves_invalid_blocks_padding_and_endianness(encoding, kind):
    invalid = [np.nan, np.inf, -2., 0.] if kind == "f4" else [0]*4
    small = np.array((invalid + [20, 40, 60, 80])*8, dtype=kind).reshape(8, 8)
    # Each overview pixel has a constant 4x4 source footprint. Padding has an
    # odd byte length, so depth values on later rows are deliberately unaligned.
    large = np.repeat(np.repeat(small, 4, axis=0), 4, axis=1)
    expected = jpeg_preview(NS(width=8, height=8, step=8*small.itemsize,
                               encoding=encoding, is_bigendian=0,
                               data=small.astype("<"+kind).tobytes()))
    for endian, flag in (("<", 0), (">", 1)):
        data = bytearray(b"".join(row.tobytes()+b"\xff\xff\xff"
                                for row in large.astype(endian+kind)))
        original = bytes(data)
        result = jpeg_preview(NS(width=32, height=32, step=32*small.itemsize+3,
                                 encoding=encoding, is_bigendian=flag, data=data),
                              max_width=8, max_height=8)
        assert result == expected
        assert bytes(data) == original  # Preview must not mutate the capture.
        assert decode(result)[:, :4].max() < 15


def test_depth_contrast_work_is_bounded_by_preview_size(monkeypatch):
    raw = np.arange(1280*720, dtype=np.float32).reshape(720, 1280) + 1
    calls = []
    percentile = np.percentile

    def count_values(values, *args, **kwargs):
        calls.append(values.size)
        return percentile(values, *args, **kwargs)

    monkeypatch.setattr(np, "percentile", count_values)
    result = jpeg_preview(NS(width=1280, height=720, step=1280*4,
                             encoding="32FC1", is_bigendian=0, data=raw.tobytes()))
    assert (result["width"], result["height"]) == (320, 180)
    assert calls == [320*180]


@pytest.mark.parametrize("encoding,color", [
    ("rgb8", [255, 0, 0]), ("bgr8", [0, 0, 255]),
    ("rgba8", [255, 0, 0, 12]), ("bgra8", [0, 0, 255, 12]),
])
@pytest.mark.parametrize("container", [array, list])
def test_downsampled_color_channel_order_and_message_buffers(encoding, color, container):
    data = bytes(color)*64+b"\xff\xff\xff"
    data = array("B", data*32) if container is array else list(data*32)
    original = bytes(data)
    result = jpeg_preview(NS(width=64, height=32, step=64*len(color)+3,
                             encoding=encoding, is_bigendian=0, data=data),
                          max_width=16, max_height=16)
    pixels = decode(result)
    assert pixels.shape == (8, 16, 3)
    assert pixels[..., 2].mean() > 240 and pixels[..., 0].mean() < 10
    assert bytes(data) == original
