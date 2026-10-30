from types import SimpleNamespace as NS
import numpy as np
import pytest
from vt_capture_core.spatial.transforms import RigidTransform
from vt_capture_manager.spatial.cloud_builder import build_cloud


def image(values, encoding, frame):
    value = np.asarray(values)
    height, width = value.shape[:2]
    return NS(width=width, height=height, step=value.strides[0], encoding=encoding,
              is_bigendian=False, data=value.tobytes(), header=NS(frame_id=frame))


def geometry():
    info=dict(width=2,height=2,fx=1.,fy=1.,cx=0.,cy=0.,d=[],distortion_model='none')
    return dict(config=dict(color_frame_id='color',depth_frame_id='depth',tracker_role='torso'),
                color_intrinsics=info,depth_intrinsics=info,
                depth_to_color=RigidTransform.identity().as_dict(),depth_units_m=.001,
                depth_units_source='realsense_ros_z16_mm')


def test_full_720p_preserves_every_valid_pixel_and_sparse_mode_is_explicit():
    from vt_capture_manager.spatial.cloud_builder import sampling_stride
    g=geometry()
    g['depth_intrinsics'].update(width=1280,height=720,fx=600.,fy=600.)
    depth=image(np.full((720,1280),1000,dtype=np.uint16),'16UC1','depth')
    xyz,rgba=build_cloud(depth,None,g,stride=1,max_points=2_097_152)
    assert len(xyz)//12 == 1280*720
    assert len(rgba)//4 == 1280*720
    assert sampling_stride(720,1280,1,2_097_152)==1
    assert sampling_stride(720,1280,4,50000)==5


@pytest.mark.parametrize('length',[0,4,5,8,12,14])
def test_projection_without_jacobians_matches_opencv(length):
    cv2=pytest.importorskip('cv2')
    from vt_capture_manager.spatial.cloud_builder import project_color
    rng=np.random.default_rng(42)
    points=rng.uniform(-.8,.8,(1000,3));points[:,2]=rng.uniform(.2,3.,1000)
    k=np.array([[640.,0,630.],[0,635.,355.],[0,0,1.]])
    d=rng.uniform(-.002,.002,length)
    expected=cv2.projectPoints(points,np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2)
    assert project_color(points,k,d)==pytest.approx(expected,abs=1e-8)


def test_raw_mm_depth_is_not_rescaled_twice_and_depth_color_transform_applies():
    g=geometry(); g['depth_to_color']=RigidTransform((.1,0.,0.),(0.,0.,0.,1.)).as_dict()
    depth=image(np.array([[1000,0],[2000,65535]],dtype=np.uint16),'16UC1','depth')
    xyz,rgba=build_cloud(depth,None,g,stride=1,min_depth_m=.1,max_depth_m=3.)
    points=np.frombuffer(xyz,dtype='<f4').reshape(-1,3)
    assert points==pytest.approx(np.array([[.1,0,1],[.1,2,2]]))
    assert len(rgba)==8


def test_float_depth_finite_range_and_bounded_sampling():
    g=geometry(); g['depth_units_m']=None; g['depth_units_source']=None
    depth=image(np.array([[1.,np.nan],[np.inf,2.]],dtype=np.float32),'32FC1','depth')
    xyz,_=build_cloud(depth,None,g,stride=1,max_points=1)
    assert len(xyz)==12


def test_rgb_samples_projected_color_and_handles_channel_order():
    depth=image(np.ones((2,2),dtype=np.uint16)*1000,'16UC1','depth')
    rgb=image(np.array([[[3,2,1],[6,5,4]],[[9,8,7],[12,11,10]]],dtype=np.uint8),'bgr8','color')
    xyz,rgba=build_cloud(depth,rgb,geometry(),stride=1,color_mode='rgb')
    assert np.frombuffer(rgba,dtype=np.uint8).reshape(-1,4).tolist()==[[1,2,3,255],[4,5,6,255],[7,8,9,255],[10,11,12,255]]


def test_rejects_missing_raw_geometry_units_and_malformed_images():
    depth=image(np.ones((2,2),dtype=np.uint16),'16UC1','depth')
    for key in ('depth_to_color','depth_intrinsics','depth_units_source'):
        g=geometry(); g[key]=None
        with pytest.raises(ValueError): build_cloud(depth,None,g)
    depth.step=1
    with pytest.raises(ValueError,match='step'): build_cloud(depth,None,geometry())

def test_rectified_depth_uses_projection_without_applying_raw_distortion_again():
    g=geometry(); g['config']['depth_topic']='/cam/depth/image_rect_raw'
    g['depth_intrinsics']=dict(g['depth_intrinsics'],d=[.5,0,0,0,0],distortion_model='plumb_bob',
        r=[1,0,0,0,1,0,0,0,1],p=[2,0,.5,0,0,2,.5,0,0,0,1,0])
    depth=image(np.ones((2,2),dtype=np.uint16)*1000,'16UC1','depth')
    xyz,_=build_cloud(depth,None,g,stride=1)
    assert np.frombuffer(xyz,dtype='<f4').reshape(-1,3)==pytest.approx(np.array([[-.25,-.25,1],[.25,-.25,1],[-.25,.25,1],[.25,.25,1]]))
    g['depth_intrinsics']['r']=[0,-1,0,1,0,0,0,0,1]
    with pytest.raises(ValueError,match='rotation'): build_cloud(depth,None,g,stride=1)
