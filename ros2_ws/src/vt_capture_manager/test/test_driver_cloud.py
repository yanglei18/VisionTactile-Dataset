from types import SimpleNamespace as NS
import struct

import numpy as np
import pytest

from vt_capture_core.spatial.transforms import RigidTransform
from vt_capture_manager.spatial.driver_cloud import adapt_driver_cloud


def cloud(points=((1.,2.,.05),(3.,4.,8.),(0.,0.,0.),(0.,0.,float('nan'))), *, big=False, packed_type=7):
    # Field padding, row padding, RGB as packed float/uint32 and both endians.
    width=2; height=len(points)//2; step=24; row=width*step+8
    raw=bytearray(row*height); endian='>' if big else '<'
    for i,point in enumerate(points):
        offset=(i//width)*row+(i%width)*step
        struct.pack_into(endian+'fff',raw,offset,*point)
        struct.pack_into(endian+'I',raw,offset+16,0x00123456+i)
    fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
    fields.append(NS(name='rgb',offset=16,datatype=packed_type,count=1))
    return NS(header=NS(frame_id='depth',stamp=NS(sec=0,nanosec=123)),
              width=width,height=height,point_step=step,row_step=row,
              is_bigendian=big,fields=fields,data=bytes(raw))


def geometry():
    return dict(config=dict(depth_frame_id='depth',cloud_topic='/cam/depth/color/points'),
                depth_to_color=RigidTransform((.01,.02,.03),(0,0,0,1)).as_dict())


@pytest.mark.parametrize('big',[False,True])
@pytest.mark.parametrize('packed_type',[6,7])
def test_driver_xyz_rgb_preserved_without_range_filter_or_resampling(big,packed_type):
    xyz,rgba,origin=adapt_driver_cloud(cloud(big=big,packed_type=packed_type),geometry())
    np.testing.assert_allclose(np.frombuffer(xyz,dtype='<f4').reshape(-1,3),
                               [[1.01,2.02,.08],[3.01,4.02,8.03]],rtol=1e-6)
    assert rgba==bytes([0x12,0x34,0x56,255,0x12,0x34,0x57,255])
    assert origin['input_points']==4 and origin['invalid_points']==2


def test_rotation_is_rigid_not_reprojected_and_black_rgb_is_retained():
    m=cloud(points=((1,0,1),(0,0,2)))
    raw=bytearray(m.data);struct.pack_into('<I',raw,16,0);m.data=bytes(raw)
    g=geometry();g['depth_to_color']=RigidTransform((0,0,0),(0,0,2**-.5,2**-.5)).as_dict()
    xyz,rgba,_=adapt_driver_cloud(m,g)
    np.testing.assert_allclose(np.frombuffer(xyz,'<f4').reshape(-1,3),[[0,1,1],[0,0,2]],atol=1e-6)
    assert rgba[:4]==bytes([0,0,0,255])


@pytest.mark.parametrize('big', [False, True])
def test_nonadjacent_xyz_preserves_padded_rows_and_original_order(big):
    original = cloud(big=big)
    expected = adapt_driver_cloud(original, geometry())
    raw = bytearray(original.data)
    for i in range(original.width * original.height):
        offset = i // original.width * original.row_step + i % original.width * original.point_step
        raw[offset+4:offset+8], raw[offset+12:offset+16] = raw[offset+12:offset+16], raw[offset+4:offset+8]
    original.fields[1].offset = 12
    original.data = bytes(raw)
    assert adapt_driver_cloud(original, geometry()) == expected


@pytest.mark.parametrize('bad',['frame','bytes','row','field','overlap','budget'])
def test_reject_malformed_or_over_budget_without_silent_decimation(bad):
    m=cloud(); kwargs={}
    if bad=='frame':m.header.frame_id='world'
    if bad=='bytes':m.data=m.data[:-1]
    if bad=='row':m.row_step=1
    if bad=='field':m.fields[0].datatype=8
    if bad=='overlap':m.fields[-1].offset=0
    if bad=='budget':kwargs['max_points']=2
    with pytest.raises(ValueError):adapt_driver_cloud(m,geometry(),**kwargs)
