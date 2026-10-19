"""Bounded RGB-D deprojection into the color optical frame.

Image bytes are decoded here, outside Manager and the GUI. Raw depth is never
assumed to already be aligned with color. World pose travels separately.
"""
import math
import numpy as np
from vt_capture_core.spatial.transforms import RigidTransform

SOURCE_COLORS = {'left_wrist': (34,211,238,255), 'right_wrist': (232,121,249,255), 'torso': (251,146,60,255)}


def decode_image(message):
    formats = {'16UC1': ('u2',1), '32FC1': ('f4',1), 'rgb8': ('u1',3),
               'bgr8': ('u1',3), 'rgba8': ('u1',4), 'bgra8': ('u1',4)}
    if message.encoding not in formats:
        raise ValueError('unsupported image encoding')
    dtype, channels = formats[message.encoding]
    dtype = np.dtype(('>' if message.is_bigendian else '<')+dtype)
    width, height, step = message.width, message.height, message.step
    if type(width) is not int or type(height) is not int or not 1 <= width <= 4096 or not 1 <= height <= 4096:
        raise ValueError('image dimensions exceed bounds')
    if type(step) is not int or step < width*channels*dtype.itemsize or step > width*channels*dtype.itemsize+65536:
        raise ValueError('invalid image step')
    if len(message.data) != step*height or len(message.data) > 128*1024*1024:
        raise ValueError('image byte length mismatch')
    shape = (height,width) if channels == 1 else (height,width,channels)
    strides = (step,dtype.itemsize) if channels == 1 else (step,channels*dtype.itemsize,dtype.itemsize)
    return np.ndarray(shape, dtype=dtype, buffer=message.data, strides=strides)


def _camera(info, message, *, rectified=False):
    if info is None:
        raise ValueError('missing camera intrinsics')
    if (info['width'],info['height']) != (message.width,message.height):
        raise ValueError('image/intrinsics dimensions mismatch')
    values = [info[k] for k in ('fx','fy','cx','cy')]
    if not all(math.isfinite(v) for v in values) or values[0] <= 0 or values[1] <= 0:
        raise ValueError('invalid intrinsics')
    if rectified:
        # RealSense depth/image_rect_raw is already rectified. Its P matrix
        # defines rays; applying raw D again would distort depth twice. A
        # nonidentity rectification also changes the meaning of axial Z and
        # is deliberately unsupported until that publisher contract is known.
        r=np.asarray(info.get('r',[]),dtype=np.float64)
        p=np.asarray(info.get('p',[]),dtype=np.float64)
        if r.size!=9 or p.size!=12 or not np.isfinite(r).all() or not np.isfinite(p).all():
            raise ValueError('rectified depth requires complete R/P calibration')
        p=p.reshape(3,4)
        if not np.allclose(r.reshape(3,3),np.eye(3),atol=1e-8,rtol=0) or not np.allclose(p[:,3],0,atol=1e-8,rtol=0):
            raise ValueError('unsupported rectified depth rotation/stereo translation')
        if not np.allclose(p[2],[0,0,1,0],atol=1e-8,rtol=0) or p[0,0]<=0 or p[1,1]<=0 or abs(p[0,1])+abs(p[1,0])>1e-8:
            raise ValueError('invalid rectified depth projection')
        return p[:,:3],np.empty(0,dtype=np.float64)
    d = np.asarray(info.get('d',[]),dtype=np.float64)
    if d.size not in (0,4,5,8,12,14) or not np.isfinite(d).all() or info.get('distortion_model','none') not in ('none','','plumb_bob','rational_polynomial'):
        raise ValueError('unsupported camera distortion')
    k = np.array([[values[0],0,values[2]],[0,values[1],values[3]],[0,0,1]],dtype=np.float64)
    if info.get('distortion_model','none') in ('none','') and np.any(d):
        raise ValueError('nonzero distortion without a declared model')
    if 'k' in info:
        original=np.asarray(info['k'],dtype=np.float64)
        if original.size!=9 or not np.allclose(original.reshape(3,3),k,rtol=0,atol=1e-8):
            raise ValueError('unsupported camera matrix skew/projection')
    return k,d


def _rigid_array(points, transform):
    pose = RigidTransform.from_dict(transform)
    origin = np.asarray(pose.translation_m)
    # Columns are rotated basis vectors; the implementation shares quaternion
    # semantics with Tracker interpolation, avoiding a second convention.
    matrix = np.asarray([np.asarray(pose.apply(axis))-origin for axis in ((1,0,0),(0,1,0),(0,0,1))]).T
    return points @ matrix.T + origin


def build_cloud(depth_message, color_message, geometry, *, stride=4, max_points=50000,
                min_depth_m=.1, max_depth_m=3., color_mode='source'):
    if type(stride) is not int or not 1 <= stride <= 64 or type(max_points) is not int or not 1 <= max_points <= 50000:
        raise ValueError('invalid cloud sampling bounds')
    if not 0 < min_depth_m < max_depth_m or not math.isfinite(max_depth_m) or color_mode not in ('source','rgb'):
        raise ValueError('invalid cloud depth/color bounds')
    config = geometry['config']
    if depth_message.header.frame_id != config['depth_frame_id']:
        raise ValueError('depth image frame mismatch')
    depth = decode_image(depth_message)
    rectified=config.get('depth_image_rectified','image_rect' in config.get('depth_topic',''))
    kd,dd = _camera(geometry.get('depth_intrinsics'),depth_message,rectified=rectified)
    if geometry.get('depth_to_color') is None:
        raise ValueError('missing raw depth to color extrinsic')
    if depth_message.encoding == '16UC1':
        # RealSense ROS publishes normalized millimetres even when hardware
        # depth units differ. An explicit publisher contract proves this unit.
        if geometry.get('depth_units_source') not in ('realsense_ros_z16_mm','rep118_16uc1_mm') or geometry.get('depth_units_m') != .001:
            raise ValueError('16UC1 depth units are unverified')
        scale = .001
    elif depth_message.encoding == '32FC1':
        scale = 1.
    else:
        raise ValueError('depth encoding must be 16UC1 or 32FC1')
    # Bound intermediate grids as well as the final wire payload. A 4K frame
    # with requested stride 1 must not allocate millions of temporary points.
    sample_stride=max(stride,math.ceil(math.sqrt(depth.shape[0]*depth.shape[1]/max_points)))
    while math.ceil(depth.shape[0]/sample_stride)*math.ceil(depth.shape[1]/sample_stride)>max_points:
        sample_stride+=1
    yy,xx=np.mgrid[0:depth.shape[0]:sample_stride,0:depth.shape[1]:sample_stride]
    z=depth[::sample_stride,::sample_stride].astype(np.float64).ravel()*scale
    good=np.isfinite(z)&(z>=min_depth_m)&(z<=max_depth_m)
    xx,yy,z=xx.ravel()[good],yy.ravel()[good],z[good]
    if z.size > max_points:
        indices=np.linspace(0,z.size-1,max_points,dtype=np.int64)
        xx,yy,z=xx[indices],yy[indices],z[indices]
    if np.any(dd):
        import cv2
        try:
            xy=cv2.undistortPoints(np.column_stack((xx,yy)).astype(np.float64).reshape(-1,1,2),kd,dd).reshape(-1,2) if z.size else np.empty((0,2))
        except cv2.error as exc:
            raise ValueError('depth distortion cannot be inverted') from exc
    else:
        xy=np.column_stack(((xx-kd[0,2])/kd[0,0],(yy-kd[1,2])/kd[1,1]))
    points=_rigid_array(np.column_stack((xy*z[:,None],z)),geometry['depth_to_color'])
    rgba=np.tile(np.asarray(SOURCE_COLORS.get(config.get('tracker_role'),(180,180,180,255)),dtype=np.uint8),(len(points),1))
    if color_mode == 'rgb':
        # Depth outside the color field of view has no measured RGB.
        rgba[:,:3]=128
        if color_message is None or color_message.header.frame_id != config['color_frame_id']:
            raise ValueError('missing/mismatched color image')
        color=decode_image(color_message)
        if color_message.encoding not in ('rgb8','bgr8','rgba8','bgra8'):
            raise ValueError('unsupported color image encoding')
        kc,dc=_camera(geometry.get('color_intrinsics'),color_message)
        infront=points[:,2]>0
        if np.any(dc):
            import cv2
            try:
                uv=cv2.projectPoints(points,np.zeros(3),np.zeros(3),kc,dc)[0].reshape(-1,2) if len(points) else np.empty((0,2))
            except cv2.error as exc:
                raise ValueError('color distortion cannot be projected') from exc
        else:
            uv=np.zeros((len(points),2)); uv[infront,0]=points[infront,0]/points[infront,2]*kc[0,0]+kc[0,2]; uv[infront,1]=points[infront,1]/points[infront,2]*kc[1,1]+kc[1,2]
        valid=infront & np.isfinite(uv).all(axis=1)
        uv=np.rint(np.where(np.isfinite(uv),uv,-1)).astype(np.int64)
        valid &= (uv[:,0]>=0)&(uv[:,0]<color.shape[1])&(uv[:,1]>=0)&(uv[:,1]<color.shape[0])
        values=color[uv[valid,1],uv[valid,0],:3]
        if color_message.encoding.startswith('bgr'):
            values=values[:,::-1]
        rgba[valid,:3]=values
    finite=np.isfinite(points).all(axis=1)
    return points[finite].astype('<f4').tobytes(),rgba[finite].tobytes()
