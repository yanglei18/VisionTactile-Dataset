"""Validated camera geometry. Configured timing serials are not identity proof."""
from copy import deepcopy
from dataclasses import asdict, is_dataclass
import hashlib
import json
import math
import re
from pathlib import Path
import threading
import time

from vt_capture_core.spatial.transforms import RigidTransform, finite_vector
from vt_capture_core.spatial.frustum import frustum_vertices


def _finite(values, size, name):
    return list(finite_vector(values, size, name))


def _intrinsics(message, expected_frame):
    if message.header.frame_id != expected_frame:
        raise ValueError('camera info frame mismatch')
    width, height = message.width, message.height
    if type(width) is not int or type(height) is not int or not 1 <= width <= 16384 or not 1 <= height <= 16384:
        raise ValueError('invalid image dimensions')
    k = _finite(message.k, 9, 'camera matrix')
    d = _finite(message.d, len(message.d), 'distortion')
    if (len(d) > 16 or k[0] <= 0 or k[4] <= 0 or abs(k[8]-1) > 1e-8
            or any(abs(k[index]) > 1e-8 for index in (1, 3, 6, 7))):
        raise ValueError('invalid camera matrix/distortion')
    roi = message.roi
    if (message.binning_x not in (0, 1) or message.binning_y not in (0, 1) or roi.x_offset or roi.y_offset
            or roi.width not in (0, width) or roi.height not in (0, height)):
        raise ValueError('binned/cropped CameraInfo is unsupported')
    return dict(frame_id=expected_frame, width=width, height=height, fx=k[0], fy=k[4], cx=k[2], cy=k[5],
                k=k, d=d, distortion_model=message.distortion_model,
                r=_finite(message.r, 9, 'rectification'), p=_finite(message.p, 12, 'projection'),
                binning_x=message.binning_x, binning_y=message.binning_y,
                roi=dict(x_offset=roi.x_offset, y_offset=roi.y_offset, width=roi.width,
                         height=roi.height, do_rectify=roi.do_rectify))


def _frustum(info):
    try:
        return [list(v) for v in frustum_vertices(info, distortion_model=info['distortion_model'], distortion=info['d'])]
    except ValueError:
        # Raw color corners must be undistorted before constructing rays.
        # This derived display geometry is cached when CameraInfo changes.
        if info['distortion_model'] not in ('plumb_bob', 'rational_polynomial') or len(info['d']) not in (4, 5, 8, 12, 14):
            return None
        try:
            import cv2
            import numpy as np
            width, height = info['width'], info['height']
            pixels = np.asarray([(0, 0), (width, 0), (width, height), (0, height)], dtype=np.float64)
            rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2), np.asarray(info['k']).reshape(3, 3), np.asarray(info['d'])).reshape(-1, 2)
            if not np.isfinite(rays).all() or np.abs(rays).max() > 1000:
                return None
            return [[float(x*z), float(y*z), z] for z in (.05, .5) for x, y in rays]
        except ImportError:
            return None
        except cv2.error:
            return None


class CameraGeometryRegistry:
    def __init__(self, cameras, *, world_frame='vive_map', identity_ttl_s=5.):
        self.world_frame, self.identity_ttl_ns = world_frame, int(identity_ttl_s*1e9)
        self._lock = threading.RLock()
        self._entries = {}
        for value in cameras:
            config = asdict(value) if is_dataclass(value) else dict(value)
            name = config.get('id', config.get('name'))
            if not name or name in self._entries:
                raise ValueError('missing or duplicate camera id')
            config['id'] = name
            config.setdefault('color_frame_id', config.get('frame_id', ''))
            config.setdefault('depth_frame_id', '')
            self._entries[name] = dict(config=config, extrinsic=None, color_intrinsics=None, color_frustum=None,
                depth_intrinsics=None, depth_to_color=None, depth_units_m=config.get('depth_units_m'),
                depth_units_source=config.get('depth_units_source'), identity=dict(verified=False,
                serial=None, observed_ns=None, reason='identity_unverified'), geometry_revision=0,
                geometry_sha256='', identity_revision=0, calibration_error='')
            self._revise(name)
            if config.get('extrinsic_path'):
                try:
                    self.load_extrinsic(name, config['extrinsic_path'])
                except (ValueError, OSError, KeyError, TypeError) as exc:
                    self._entries[name]['calibration_error'] = str(exc)[:256]

    def _revise(self, name):
        entry = self._entries[name]
        content = {k: entry[k] for k in ('config', 'extrinsic', 'color_intrinsics', 'depth_intrinsics',
                                        'depth_to_color', 'depth_units_m', 'depth_units_source')}
        digest = hashlib.sha256(json.dumps(content, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        if digest != entry['geometry_sha256']:
            entry['geometry_sha256'] = digest
            entry['geometry_revision'] += 1
            return True
        return False

    def observe_device_info(self, camera_id, response, observed_ns=None):
        with self._lock:
            entry = self._entries[camera_id]
            serial = response.serial_number
            if not isinstance(serial, str) or not serial or len(serial) > 256:
                raise ValueError('invalid observed serial')
            model = entry['config'].get('model')
            observed_model = getattr(response, 'device_name', '')
            verified = serial == str(entry['config']['serial']) and (not model or str(model).upper() in observed_model.upper())
            if (entry['identity'].get('verified'), entry['identity'].get('serial')) != (verified, serial):
                entry['identity_revision'] += 1
            entry['identity'] = dict(verified=verified, serial=serial,
                observed_ns=time.time_ns() if observed_ns is None else observed_ns,
                reason='' if verified else 'identity_mismatch', device_name=getattr(response, 'device_name', ''),
                firmware_version=getattr(response, 'firmware_version', ''))
            return verified

    def invalidate_identity(self, camera_id, reason):
        with self._lock:
            entry = self._entries[camera_id]
            if entry['identity'].get('verified'):
                entry['identity_revision'] += 1
            entry['identity'].update(verified=False, reason=str(reason)[:256], invalidated_ns=time.time_ns())

    def observe_camera_info(self, camera_id, message, stream='color'):
        if stream not in ('color', 'depth'):
            raise ValueError('unsupported camera info stream')
        with self._lock:
            entry = self._entries[camera_id]
            try:
                info = _intrinsics(message, entry['config'][f'{stream}_frame_id'])
            except (ValueError, TypeError, AttributeError, OverflowError):
                entry[f'{stream}_intrinsics'] = None
                if stream == 'color': entry['color_frustum'] = None
                self._revise(camera_id)
                raise
            if info == entry[f'{stream}_intrinsics']:
                return False
            entry[f'{stream}_intrinsics'] = info
            if stream == 'color': entry['color_frustum'] = _frustum(info)
            return self._revise(camera_id)

    def load_extrinsic(self, camera_id, path):
        from .config import _document
        return self.set_extrinsic(camera_id, _document(Path(path)))

    def set_extrinsic(self, camera_id, document):
        # Candidate validation precedes mutation. A failed replacement must not
        # revoke the last valid calibration or change its geometry revision.
        return self._set_extrinsic(camera_id, document)

    def replace_calibration_binding(self, camera_id, document):
        """Atomically replace a calibrated Tracker binding; camera identity is fixed."""
        with self._lock:
            entry = self._entries[camera_id]
            config = deepcopy(entry['config'])
            tracker = document['tracker']
            if tracker.get('role') not in ('torso', 'left_wrist', 'right_wrist'):
                raise ValueError('unsupported calibrated Tracker role')
            frame = tracker.get('frame_id')
            if not isinstance(frame, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_/]{0,127}', frame):
                raise ValueError('invalid calibrated Tracker frame')
            config.update(tracker_role=tracker['role'], tracker_frame_id=frame)
            config.pop('extrinsic_path', None)
            # A detached candidate uses the same validation as startup loading,
            # without exposing a partially rebound entry to rendering/recording.
            candidate = CameraGeometryRegistry([config], world_frame=self.world_frame)
            candidate.set_extrinsic(camera_id, document)
            accepted = candidate.geometry(camera_id)['extrinsic']
            entry.update(config=config, extrinsic=accepted, calibration_error='')
            return self._revise(camera_id)

    def _set_extrinsic(self, camera_id, document):
        with self._lock:
            entry, doc = self._entries[camera_id], deepcopy(document)
            if doc.get('schema_version') != 1 or doc.get('status') != 'VALID':
                raise ValueError('extrinsic schema 1 and status VALID required')
            config, camera, tracker, transform = entry['config'], doc['camera'], doc['tracker'], doc['transform']
            expected = [('camera.name', camera['name'], camera_id), ('camera.serial', camera['serial'], config['serial']),
                        ('camera.frame', camera['frame_id'], config['color_frame_id']),
                        ('tracker.role', tracker['role'], config['tracker_role']),
                        ('world.frame', doc['frames']['world'], self.world_frame),
                        ('transform.semantics', transform['semantics'], 'parent_from_child'),
                        ('transform.parent_frame', transform['parent_frame'], tracker['frame_id']),
                        ('transform.child_frame', transform['child_frame'], camera['frame_id'])]
            if config.get('tracker_frame_id'):
                expected.append(('tracker.frame_id', tracker['frame_id'], config['tracker_frame_id']))
            if config.get('model'):
                expected.append(('camera.model', camera.get('model'), config['model']))
            for label, actual, wanted in expected:
                if actual != wanted:
                    raise ValueError(f'{label} mismatch')
            if not isinstance(tracker['tracker_id'], str) or not tracker['tracker_id'] or len(tracker['tracker_id']) > 256:
                raise ValueError('invalid calibration tracker identity')
            RigidTransform(transform['translation_m'], transform['quaternion_xyzw'])
            json.dumps(doc, allow_nan=False)
            entry.update(extrinsic=doc, calibration_error='')
            return self._revise(camera_id)

    def invalidate_depth_geometry(self, camera_id, reason='invalid depth geometry'):
        with self._lock:
            self._entries[camera_id]['depth_to_color'] = None
            self._entries[camera_id]['depth_geometry_error'] = str(reason)[:256]
            self._revise(camera_id)

    def observe_depth_to_color(self, camera_id, transform):
        try:
            return self._observe_depth_to_color(camera_id, transform)
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError):
            self.invalidate_depth_geometry(camera_id)
            raise

    def _observe_depth_to_color(self, camera_id, transform):
        with self._lock:
            entry = self._entries[camera_id]
            if isinstance(transform, RigidTransform):
                pose = transform
            elif isinstance(transform, dict):
                if transform.get('parent_frame', entry['config']['color_frame_id']) != entry['config']['color_frame_id'] or transform.get('child_frame', entry['config']['depth_frame_id']) != entry['config']['depth_frame_id']:
                    raise ValueError('depth to color transform frame mismatch')
                pose = RigidTransform.from_dict({k: transform[k] for k in ('translation_m', 'quaternion_xyzw')})
            else:
                if transform.header.frame_id != entry['config']['color_frame_id'] or transform.child_frame_id != entry['config']['depth_frame_id']:
                    raise ValueError('depth to color transform frame mismatch')
                t, q = transform.transform.translation, transform.transform.rotation
                pose = RigidTransform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            entry['depth_to_color'] = pose.as_dict()
            return self._revise(camera_id)

    def set_depth_units(self, camera_id, meters, source):
        if type(meters) not in (int, float) or not math.isfinite(meters) or meters <= 0 or not isinstance(source, str) or not source:
            raise ValueError('invalid depth units')
        with self._lock:
            self._entries[camera_id].update(depth_units_m=meters, depth_units_source=source)
            return self._revise(camera_id)

    def geometry(self, camera_id):
        with self._lock:
            return deepcopy(self._entries[camera_id])

    def describe(self):
        with self._lock:
            return [dict(id=name, label=e['config'].get('label', name), serial=e['config']['serial'],
                         tracker_role=e['config']['tracker_role'], frame_id=e['config']['color_frame_id'],
                         geometry_revision=e['geometry_revision'], geometry_sha256=e['geometry_sha256'])
                    for name, e in self._entries.items()]

    def camera_entities(self, trackers, now_ns=None):
        now_ns = time.time_ns() if now_ns is None else now_ns
        if not isinstance(trackers, dict):
            trackers = {v.get('role', v['id']): v for v in trackers}
        result = []
        with self._lock:
            for name, e in self._entries.items():
                config, identity, doc, info = e['config'], e['identity'], e['extrinsic'], e['color_intrinsics']
                tracker = trackers.get(config['tracker_role'])
                entity = dict(id=name, label=config.get('label', name), serial=config['serial'],
                    tracker_role=config['tracker_role'], frame_id=config['color_frame_id'],
                    position_m=None, quaternion_xyzw=None, state='waiting', reason='', registration_state='waiting',
                    geometry_revision=e['geometry_revision'], geometry_sha256=e['geometry_sha256'],
                    intrinsics=deepcopy(info), frustum_vertices=None, source_realtime_ns=None)
                reason = ''
                if not identity['verified']:
                    reason = 'identity_mismatch' if identity['reason'] == 'identity_mismatch' else 'identity_unverified'
                elif identity['observed_ns'] is None or not 0 <= now_ns-identity['observed_ns'] <= self.identity_ttl_ns:
                    reason = 'identity_unverified'
                elif doc is None:
                    reason = 'missing_calibration'
                elif tracker is None or tracker.get('state') not in ('fresh', 'delayed') or tracker.get('position_m') is None:
                    reason = 'tracker_unavailable'
                elif tracker.get('tracker_id') != doc['tracker']['tracker_id']:
                    reason = 'tracker_mismatch'
                if not reason:
                    try:
                        local = e['color_frustum']
                        if local is None:
                            entity['reason'] = 'missing_intrinsics' if info is None else 'unsupported_frustum_distortion'
                        world = RigidTransform(tracker['position_m'], tracker['quaternion_xyzw']).compose(
                            RigidTransform(doc['transform']['translation_m'], doc['transform']['quaternion_xyzw']))
                        entity.update(position_m=list(world.translation_m), quaternion_xyzw=list(world.quaternion_xyzw),
                                      state=tracker['state'], registration_state='ready',
                                      frustum_vertices=[list(p) for p in local] if local else None, source_realtime_ns=tracker['source_realtime_ns'])
                    except (ValueError, TypeError, KeyError):
                        reason = 'unsupported_geometry'
                if reason:
                    entity.update(reason=reason, registration_state=reason,
                                  state='invalid' if reason in ('identity_mismatch', 'tracker_mismatch', 'unsupported_geometry') else 'waiting')
                result.append(entity)
        return result
