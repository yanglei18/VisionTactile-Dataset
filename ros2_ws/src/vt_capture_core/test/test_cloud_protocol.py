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


def test_driver_origin_roundtrip_requires_honest_counts_and_no_rgbd_sampling():
    origin=dict(source='realsense',topic='/cam/depth/color/points',native_frame_id='depth',input_points=4,invalid_points=2)
    meta=dict(header(),origin=origin)
    wire=pack_cloud_frame(meta,struct.pack('<6f',1,2,3,4,5,6),bytes(8))
    assert unpack_cloud_frame(wire).header['origin']==origin
    for bad in (dict(origin,invalid_points=1),dict(origin,source='unknown'),dict(origin,input_points=MAX_POINTS+1)):
        with pytest.raises(CloudProtocolError):pack_cloud_frame(dict(meta,origin=bad),bytes(24),bytes(8))
    with pytest.raises(CloudProtocolError):
        pack_cloud_frame(dict(meta,sampling=dict(width=2,height=2,stride=1,min_depth_m=.1,max_depth_m=3.)),bytes(24),bytes(8))


def test_full_720p_wire_roundtrip_and_sampling_metadata():
    count=1280*720
    meta=header(count)
    meta['sampling']=dict(width=1280,height=720,stride=1,min_depth_m=.1,max_depth_m=3.)
    xyz=struct.pack('<fff',0.,0.,1.)*count
    rgba=bytes([1,2,3,255])*count
    frame=unpack_cloud_frame(pack_cloud_frame(meta,xyz,rgba))
    assert frame.xyz_bytes==xyz and frame.rgba_bytes==rgba
    assert frame.header['sampling']==meta['sampling']


def test_malformed_sampling_metadata_is_rejected():
    for sampling in (None, {}, dict(width=1280,height=720,stride=0,min_depth_m=.1,max_depth_m=3.)):
        with pytest.raises(CloudProtocolError):
            pack_cloud_frame(dict(header(0),sampling=sampling),b'',b'')


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


def test_zero_copy_decode_reuses_immutable_wire_with_same_validation():
    xyz=struct.pack('<6f',1,2,3,4,5,6)
    rgba=bytes(range(8))
    wire=pack_cloud_frame(header(),xyz,rgba)
    frame=unpack_cloud_frame(wire,copy_payload=False)
    assert frame.xyz_bytes.obj is wire and frame.rgba_bytes.obj is wire
    assert frame.xyz_bytes.readonly and frame.rgba_bytes.readonly
    assert frame.xyz_bytes==xyz and frame.rgba_bytes==rgba
    corrupted=wire[:-32]+struct.pack('<f',float('nan'))+wire[-28:]
    with pytest.raises(CloudProtocolError,match='finite'):
        unpack_cloud_frame(corrupted,copy_payload=False)


@pytest.mark.parametrize('readonly', [False,True])
def test_zero_copy_mode_freezes_mutable_owner_before_exposing_views(readonly):
    xyz=struct.pack('<6f',1,2,3,4,5,6)
    wire=bytearray(pack_cloud_frame(header(),xyz,bytes(8)))
    supplied=memoryview(wire).toreadonly() if readonly else wire
    frame=unpack_cloud_frame(supplied,copy_payload=False)
    wire[-32:]=bytes(32)
    assert frame.xyz_bytes==xyz
    assert isinstance(frame.xyz_bytes.obj,bytes)
