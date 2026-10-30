"""Adapt RealSense PointCloud2 without deprojection, recoloring or range clipping.

The driver owns geometry and RGB projection. Only invalid depth returns are
removed; raw depth-optical XYZ is rigidly expressed in the calibrated RGB frame.
This keeps the existing wire/world_from_color contract, without applying the
Tracker transform twice. No ROS imports are needed by this bounded decoder.
"""
import numpy as np

from vt_capture_core.spatial.cloud_protocol import MAX_POINTS
from .cloud_builder import _rigid_array


def adapt_driver_cloud(message, geometry, *, max_points=MAX_POINTS):
    config = geometry['config']
    if message.header.frame_id != config['depth_frame_id']:
        raise ValueError('driver point cloud must be in configured depth optical frame')
    width, height, step, row = message.width, message.height, message.point_step, message.row_step
    if (any(type(v) is not int for v in (width, height, step, row)) or
            width < 0 or not 1 <= height <= 4096 or not 12 <= step <= 256 or
            width * height > min(max_points, MAX_POINTS) or
            row < width * step or row > width * step + 65536 or
            len(message.data) != row * height or len(message.data) > 128 * 1024 * 1024):
        raise ValueError('invalid or oversized driver point cloud layout')
    fields = {}
    for field in message.fields:
        if field.name in fields:
            raise ValueError('duplicate PointCloud2 field')
        fields[field.name] = field
    color_name = 'rgb' if 'rgb' in fields else 'rgba'
    names = ('x', 'y', 'z', color_name)
    offsets = []
    for name in names:
        field = fields.get(name)
        if (field is None or field.count != 1 or
                field.datatype not in ((7,) if name in ('x','y','z') else (6,7)) or
                type(field.offset) is not int or not 0 <= field.offset <= step-4):
            raise ValueError('unsupported XYZ/RGB PointCloud2 field')
        offsets.append(field.offset)
    if any(abs(a-b) < 4 for i,a in enumerate(offsets) for b in offsets[i+1:]):
        raise ValueError('overlapping XYZ/RGB PointCloud2 fields')
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype(dict(names=names, formats=[endian+'f4']*3+[endian+'u4'],
                         offsets=offsets, itemsize=step))
    records = np.ndarray((height,width), dtype=dtype, buffer=message.data,
                         strides=(row,step))
    if offsets[:3] == [offsets[0],offsets[0]+4,offsets[0]+8]:
        # Native XYZ are adjacent inside padded point records. View them
        # directly; do not copy three whole strided channels and stack again.
        xyz = np.ndarray((height,width,3),dtype=endian+'f4',buffer=message.data,
                         offset=offsets[0],strides=(row,step,4))
    else:
        xyz = np.stack([records[n] for n in ('x','y','z')],axis=-1)
    # Ordered RealSense clouds use zero/NaN for pixels with no depth. Those
    # are not measured surfaces. Retain all positive finite depths, including
    # below 0.1m / beyond 3m if the driver actually produced them.
    valid = (np.isfinite(records['x']) & np.isfinite(records['y']) &
             np.isfinite(records['z']) & (records['z'] > 0))
    rgb = records[color_name][valid]
    rgba = np.empty((len(rgb),4), dtype=np.uint8)
    rgba[:,0]=(rgb>>16)&255; rgba[:,1]=(rgb>>8)&255; rgba[:,2]=rgb&255
    rgba[:,3]=255
    points = _rigid_array(xyz[valid], geometry['depth_to_color'])
    if not np.isfinite(points).all():
        raise ValueError('nonfinite transformed driver point')
    origin = dict(source='realsense', topic=config['cloud_topic'],
                  native_frame_id=message.header.frame_id, input_points=width*height,
                  invalid_points=int((~valid).sum()))
    return points.astype('<f4').tobytes(), rgba.tobytes(), origin
