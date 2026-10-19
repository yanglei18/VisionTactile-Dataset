import struct

import pytest

from vt_capture_core.spatial.cloud_protocol import (
    CloudProtocolError, MAX_POINTS, inspect_cloud_prefix, pack_cloud_frame, unpack_cloud_frame,
)


def header(count=2):
    return dict(camera_id='camera', frame_key={'camera_id': 'camera', 'shared_ros_timestamp_ns': 9},
                source_realtime_ns=123, time_epoch='t', coordinate_epoch='c',
                geometry_revision='g', geometry_sha256='0'*64, point_count=count,
                world_from_color={'translation_m': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1]},
                worker_instance_id='worker', cloud_sequence=7)


def test_cloud_roundtrip_and_block_layout():
    xyz = struct.pack('<6f', 1, 2, 3, 4, 5, 6)
    rgba = bytes((1, 2, 3, 255, 4, 5, 6, 255))
    wire = pack_cloud_frame(header(), xyz, rgba)
    frame = unpack_cloud_frame(wire)
    assert frame.xyz_bytes == xyz and frame.rgba_bytes == rgba
    assert frame.header['point_count'] == 2
    assert tuple(frame) == (frame.header, xyz, rgba)
    assert wire.endswith(xyz+rgba)


def test_empty_cloud_valid():
    frame = unpack_cloud_frame(pack_cloud_frame(header(0), b'', b''))
    assert frame.header['point_count'] == 0


def test_bad_lengths_nonfinite_and_oversized_count_rejected():
    for meta, xyz, rgba in [
        (header(), b'bad', b'bad'),
        (header(1), struct.pack('<3f', float('nan'), 1, 2), bytes(4)),
        (header(MAX_POINTS+1), b'', b''),
        (dict(header(), point_count=True), b'', b''),
    ]:
        with pytest.raises(CloudProtocolError):
            pack_cloud_frame(meta, xyz, rgba)


def test_corrupt_truncated_and_trailing_frames_rejected():
    valid = pack_cloud_frame(header(1), struct.pack('<3f', 1, 2, 3), bytes(4))
    for wire in (valid[:8], valid[:-1], valid+b'extra', b'FAIL'+valid[4:]):
        with pytest.raises(CloudProtocolError):
            unpack_cloud_frame(wire)


@pytest.fixture(params=['standard_library', 'numpy'])
def coordinate_validator(request, monkeypatch):
    from vt_capture_core.spatial import cloud_protocol
    module = pytest.importorskip('numpy') if request.param == 'numpy' else None
    monkeypatch.setattr(cloud_protocol, '_numpy', module)


def test_little_endian_boundary_values_are_identical_in_both_validators(coordinate_validator):
    # Positive/negative zero, subnormal, largest finite and an ordinary value.
    # Explicit bytes test wire endianness rather than the host's float layout.
    xyz = struct.pack('<6I', 0, 0x80000000, 1, 0x7f7fffff, 0xff7fffff, 0x3f800000)
    frame = unpack_cloud_frame(pack_cloud_frame(header(2), xyz, bytes(8)))
    assert frame.xyz_bytes == xyz
    assert unpack_cloud_frame(pack_cloud_frame(header(0), b'', b'')).xyz_bytes == b''


@pytest.mark.parametrize('bits', [0x7f800000, 0xff800000, 0x7fc00001, 0x7f800001])
@pytest.mark.parametrize('index', [0, 5, 11])
def test_every_coordinate_is_checked_on_send_and_receive(coordinate_validator, bits, index):
    xyz = bytearray(struct.pack('<12f', *range(12)))
    valid = pack_cloud_frame(header(4), xyz, bytes(16))
    struct.pack_into('<I', xyz, index*4, bits)
    with pytest.raises(CloudProtocolError, match='finite'):
        pack_cloud_frame(header(4), xyz, bytes(16))
    corrupted = valid[:-64] + xyz + valid[-16:]
    with pytest.raises(CloudProtocolError, match='finite'):
        unpack_cloud_frame(corrupted)


@pytest.mark.parametrize('factory', [bytes, bytearray, memoryview])
def test_contiguous_byte_buffers_keep_the_same_wire_contract(coordinate_validator, factory):
    xyz, rgba = struct.pack('<3f', -1, 0, 1), bytes(4)
    wire = pack_cloud_frame(header(1), factory(xyz), factory(rgba))
    assert unpack_cloud_frame(factory(wire)).xyz_bytes == xyz


@pytest.mark.parametrize('view', [
    memoryview(bytes(24))[::2],
    memoryview(bytes(12)).cast('f'),
    memoryview(bytes(12)).cast('B', shape=[3, 4]),
])
def test_ambiguous_or_noncontiguous_buffers_raise_protocol_errors(coordinate_validator, view):
    with pytest.raises(CloudProtocolError, match='buffers'):
        pack_cloud_frame(header(1), view, bytes(4))
    with pytest.raises(CloudProtocolError, match='buffers'):
        pack_cloud_frame(header(1), bytes(12), view)
    with pytest.raises(CloudProtocolError, match='buffers'):
        unpack_cloud_frame(view)
    with pytest.raises(CloudProtocolError, match='buffers'):
        inspect_cloud_prefix(view)


def test_released_memoryview_is_a_protocol_error():
    view = memoryview(bytes(12))
    view.release()
    with pytest.raises(CloudProtocolError, match='unavailable'):
        pack_cloud_frame(header(1), view, bytes(4))
    with pytest.raises(CloudProtocolError, match='unavailable'):
        unpack_cloud_frame(view)
